"""ROS2 node bridging /leg_controller/joint_trajectory to ODrive CAN.

See README / CLAUDE.md for the bring-up procedure. Safety posture:

- Starts in dry-run by default (no state or position frames sent to the
  bus).
- Motors are never armed until /real_leg/arm is called explicitly, even
  once dry_run is disabled.
- Rate limiter bounds commanded joint speed regardless of what the IK
  node asks for, so arming with a live stick or an IK glitch can't
  produce a snap.
- Every teardown path (clean shutdown, SIGINT, exception) sends IDLE to
  all three nodes before closing the serial port.
"""
from __future__ import annotations

import os
import threading
import time
from collections import deque
from dataclasses import dataclass

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

try:
    import serial  # pyserial
except ImportError as e:  # pragma: no cover
    raise SystemExit(
        'pyserial is required: sudo apt install python3-serial'
    ) from e

from byte_leg_hardware import can_waveshare as cw


_TAU = 6.283185307179586  # 2*pi


@dataclass
class _JointCfg:
    name: str
    node_id: int
    gear_ratio: float
    sign: float
    zero_offset_rev: float
    max_joint_rad: float
    max_rate_rad_s: float


class CanRelay(Node):
    def __init__(self) -> None:
        super().__init__('can_relay')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 2000000)
        self.declare_parameter('joint_names', ['hip_abduct', 'hip_pitch', 'knee'])
        self.declare_parameter('can_node_ids', [3, 5, 1])
        self.declare_parameter('gear_ratio', [8.0, 8.0, 8.0])
        self.declare_parameter('sign', [1.0, 1.0, 1.0])
        self.declare_parameter('zero_offset_rev', [0.0, 0.0, 0.0])
        self.declare_parameter('dry_run', True)
        self.declare_parameter('read_only', False)
        self.declare_parameter('max_joint_rad', [1.8, 1.8, 2.5])
        self.declare_parameter('max_rate_rad_s', [4.0, 4.0, 6.0])
        self.declare_parameter('command_timeout_s', 0.25)
        self.declare_parameter('arm_grace_s', 5.0)
        self.declare_parameter('calibration_file', '/tmp/byte_leg_calibration.yaml')
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('state_publish_hz', 100.0)
        self.declare_parameter('diag_publish_hz', 2.0)
        self.declare_parameter('tx_nice', -5)

        names = list(self.get_parameter('joint_names').value)
        node_ids = list(self.get_parameter('can_node_ids').value)
        gears = list(self.get_parameter('gear_ratio').value)
        signs = list(self.get_parameter('sign').value)
        offsets = list(self.get_parameter('zero_offset_rev').value)
        maxrad = list(self.get_parameter('max_joint_rad').value)
        maxrate = list(self.get_parameter('max_rate_rad_s').value)
        if not (len(names) == len(node_ids) == len(gears) == len(signs)
                == len(offsets) == len(maxrad) == len(maxrate)):
            raise ValueError('per-joint parameter arrays disagree in length')

        self._joints: list[_JointCfg] = [
            _JointCfg(name=names[i], node_id=int(node_ids[i]),
                      gear_ratio=float(gears[i]), sign=float(signs[i]),
                      zero_offset_rev=float(offsets[i]),
                      max_joint_rad=float(maxrad[i]),
                      max_rate_rad_s=float(maxrate[i]))
            for i in range(len(names))
        ]
        self._index_by_name = {j.name: i for i, j in enumerate(self._joints)}
        # Pre-computed for the 100 Hz state-publish hot path. Avoids
        # rebuilding the name list and re-doing kwargs function dispatch
        # on every tick — small per-call wins, but they add up at 100 Hz
        # × 3 joints, and matter more on the Jetson.
        self._joint_names_const = tuple(j.name for j in self._joints)
        # rad = (rev - zero) * (TAU / (sign * gear_ratio))
        self._rev_to_rad = tuple(
            _TAU / (j.sign * j.gear_ratio) for j in self._joints)
        # rev/s -> rad/s shares the same factor.
        self._zero_offsets = tuple(j.zero_offset_rev for j in self._joints)

        self._dry_run: bool = bool(self.get_parameter('dry_run').value)
        self._read_only: bool = bool(self.get_parameter('read_only').value)
        # read_only must be able to write axis-state frames (CLOSED_LOOP on
        # startup, IDLE on shutdown). It still suppresses SET_INPUT_POS via
        # the arm refusal below.
        if self._read_only and self._dry_run:
            self.get_logger().warn(
                'read_only:=true overrides dry_run:=true: axis-state '
                'writes will be sent so motors can be energized.')
            self._dry_run = False
        self._command_timeout_s: float = float(
            self.get_parameter('command_timeout_s').value)
        self._arm_grace_s: float = float(
            self.get_parameter('arm_grace_s').value)
        self._calibration_file: str = str(
            self.get_parameter('calibration_file').value)

        # --- State guarded by self._lock ----------------------------------
        # Single data lock, shared across measurement, command-target, and
        # diag counters. Hold times are microseconds — kept coarse so we
        # never have to think about ordering between two locks.
        self._lock = threading.Lock()
        # Serializes raw ser.write across the dedicated TX thread and
        # the executor-side service callbacks. Distinct from _lock — the
        # data lock guards python state, this one guards bytes-on-the-wire.
        self._ser_lock = threading.Lock()
        n = len(self._joints)
        self._meas_pos_rev: list[float | None] = [None] * n
        self._meas_vel_rev_s: list[float | None] = [None] * n
        # Monotonic timestamp of the last encoder frame per joint, for
        # the encoder_age_ms_max diag stat.
        self._meas_pos_time: list[float | None] = [None] * n

        # Desired joint angle fed by the IK node (clamped, not rate-limited).
        self._desired_rad: list[float] = [0.0] * n
        # Rate-limited joint angle actually sent to the motor each tick.
        self._current_cmd_rad: list[float] = [0.0] * n
        self._armed: bool = False
        self._last_traj_time: float | None = None
        self._traj_count: int = 0
        self._armed_at: float | None = None

        # --- Diagnostics counters (also under self._lock) ----------------
        # Ring buffer of the last N TX tick periods (seconds). Sized for
        # ~4 seconds at 50 Hz — comfortably more than the 0.5 s diag window.
        self._tx_tick_periods: deque[float] = deque(maxlen=200)
        self._last_tx_monotonic: float | None = None
        self._rx_frame_count: int = 0
        self._sync_resync_count: int = 0
        # Sampled by the diag publisher to compute rx_frames_per_s.
        self._diag_last_rx_count: int = 0
        self._diag_last_sample_time: float | None = None

        # Try to load a previously saved calibration.
        self._maybe_load_calibration()

        # --- Serial ------------------------------------------------------
        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        self.get_logger().info(f'Opening {port} @ {baud} baud')
        try:
            # exclusive=True posts an advisory flock so a stray pyserial
            # in another shell can't quietly corrupt our framing.
            self._ser = serial.Serial(port, baud, timeout=0.01, exclusive=True)
        except serial.SerialException as e:
            raise SystemExit(f'cannot open {port}: {e}') from e
        cw.init_waveshare(self._ser)
        try:
            self._ser.reset_input_buffer()
        except Exception:  # pragma: no cover
            pass

        # --- RX thread --------------------------------------------------
        self._rx_stop = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name='can_rx', daemon=True)
        self._rx_thread.start()

        # --- Pubs / Subs / Services ------------------------------------
        self._pub_joint_states = self.create_publisher(
            JointState, '/joint_states', 10)
        # Diagnostics: layout is
        #   [tx_tick_period_ms_p99, rx_frames_per_s,
        #    sync_resync_count, encoder_age_ms_max]
        # so a `ros2 topic echo /real_leg/diag` answers "is this janky?"
        # in one read.
        self._pub_diag = self.create_publisher(
            Float32MultiArray, '/real_leg/diag', 10)
        # Put the trajectory subscription and the services in their own
        # reentrant group so they can run on a different executor thread
        # than the timers. Best-effort QoS so a slow consumer can never
        # back-pressure ik_node — for a 50 Hz position setpoint stream,
        # stale messages are useless anyway.
        self._async_cb_group = ReentrantCallbackGroup()
        traj_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            JointTrajectory, '/leg_controller/joint_trajectory',
            self._on_trajectory, traj_qos,
            callback_group=self._async_cb_group)

        self.create_service(Trigger, '/real_leg/arm', self._srv_arm,
                            callback_group=self._async_cb_group)
        self.create_service(Trigger, '/real_leg/disarm', self._srv_disarm,
                            callback_group=self._async_cb_group)
        self.create_service(
            Trigger, '/real_leg/calibrate_home', self._srv_calibrate_home,
            callback_group=self._async_cb_group)

        cmd_rate = float(self.get_parameter('command_rate_hz').value)
        state_rate = float(self.get_parameter('state_publish_hz').value)
        diag_rate = float(self.get_parameter('diag_publish_hz').value)
        self._cmd_dt = 1.0 / cmd_rate
        # SET_INPUT_POS path runs in a dedicated thread (see _tx_loop)
        # outside the rclpy executor — Python's GIL combined with the
        # executor's single-callback dispatch was starving the trajectory
        # subscription to ~2 Hz when this ran as an executor timer.
        self.create_timer(1.0 / state_rate, self._publish_joint_states)
        self.create_timer(1.0 / diag_rate, self._publish_diag)

        # --- Dedicated TX thread (50 Hz command stream + watchdog) -----
        self._tx_stop = threading.Event()
        self._tx_thread = threading.Thread(
            target=self._tx_loop, name='can_tx', daemon=True)
        self._tx_thread.start()

        if self._read_only:
            banner = '[READ-ONLY] '
        elif self._dry_run:
            banner = '[DRY-RUN] '
        else:
            banner = ''
        self.get_logger().info(
            f'{banner}can_relay ready. Joints: ' + ', '.join(
                f'{j.name}->node{j.node_id}' for j in self._joints))

        if self._read_only:
            self.get_logger().warn(
                'READ-ONLY ENERGIZED MODE: setting nodes to CLOSED_LOOP. '
                'Arm service is disabled; no SET_INPUT_POS will be sent. '
                'Ctrl+C will return all nodes to IDLE.')
            # Let encoder estimates start arriving so /joint_states has
            # data the moment the motors come live.
            time.sleep(0.5)
            # Per-node IDLE -> sleep -> CLOSED_LOOP -> sleep, mirroring
            # tune.py. On CyberBeast 0.6.4 firmware, requesting CLOSED_LOOP
            # without an explicit IDLE transition first is silently ignored.
            for j in self._joints:
                self.get_logger().info(
                    f'energizing node {j.node_id} ({j.name})')
                with self._ser_lock:
                    cw.set_axis_state(self._ser, j.node_id, cw.AXIS_IDLE)
                time.sleep(0.5)
                with self._ser_lock:
                    cw.set_axis_state(
                        self._ser, j.node_id, cw.AXIS_CLOSED_LOOP)
                time.sleep(1.0)
            # Re-anchor the watchdog: the ~4.5 s of blocking sleeps above
            # would otherwise look like a stuck command timeout if the
            # user later flips into armed mode in the same session.
            with self._lock:
                self._last_traj_time = None

    # ================================================================== #
    #  RX thread
    # ================================================================== #
    def _rx_loop(self) -> None:
        buf = bytearray()
        while not self._rx_stop.is_set():
            try:
                # Blocking read with the timeout configured on the port
                # (10 ms). select() releases the GIL while waiting, so
                # this thread doesn't starve other Python work — unlike
                # the previous in_waiting + sleep busy-poll which kept
                # acquiring the GIL 500 times/sec.
                data = self._ser.read(256)
                if data:
                    buf.extend(data)
            except Exception as e:  # pragma: no cover - hw-only
                self.get_logger().error(f'serial read error: {e}')
                time.sleep(0.05)
                continue

            for arb_id, payload in cw.iter_frames(buf, on_resync=self._on_sync_resync):
                node_id, cmd_id = cw.split_arb_id(arb_id)
                if cmd_id == cw.CMD_GET_ENCODER_ESTIMATES and len(payload) == 8:
                    pos, vel = cw.decode_encoder_estimates(payload)
                    idx = self._index_for_node(node_id)
                    if idx is not None:
                        now = time.monotonic()
                        with self._lock:
                            self._meas_pos_rev[idx] = pos
                            self._meas_vel_rev_s[idx] = vel
                            self._meas_pos_time[idx] = now
                            self._rx_frame_count += 1

    def _on_sync_resync(self) -> None:
        # Called from the RX thread inside iter_frames every time a
        # corrupted byte is dropped. Cheap counter; visible on /real_leg/diag.
        with self._lock:
            self._sync_resync_count += 1

    def _index_for_node(self, node_id: int) -> int | None:
        for i, j in enumerate(self._joints):
            if j.node_id == node_id:
                return i
        return None

    # ================================================================== #
    #  Command path
    # ================================================================== #
    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        positions = msg.points[0].positions
        names = list(msg.joint_names)
        # Build the candidate desired vector outside the lock so the
        # critical section is just the snapshot swap.
        with self._lock:
            new_desired = list(self._desired_rad)
            armed = self._armed
            self._last_traj_time = time.monotonic()
            self._traj_count += 1
        # Don't overwrite the seeded target while disarmed: the value
        # seeded by _srv_arm (= measured position) must survive until
        # armed, or the leg jolts to the IK home pose the instant arm
        # completes.
        if not armed:
            return
        for k, jname in enumerate(names):
            idx = self._index_by_name.get(jname)
            if idx is None or k >= len(positions):
                continue
            lim = self._joints[idx].max_joint_rad
            new_desired[idx] = max(-lim, min(lim, float(positions[k])))
        with self._lock:
            self._desired_rad = new_desired

    def _tx_loop(self) -> None:
        """Dedicated 50 Hz writer thread, independent of the rclpy executor.

        Owns SET_INPUT_POS sending and the stale-command watchdog. Runs
        outside the executor so Python GIL contention with the timer
        callbacks can't starve it (which is what was capping the practical
        control rate to ~2 Hz when this ran as an executor timer).
        """
        # Best-effort: nudge the TX thread up the SCHED_OTHER ladder so
        # the 50 Hz tick survives a heavy IK callback. Needs CAP_SYS_NICE
        # or an /etc/security/limits.d entry — see byte_leg_hardware/README.
        # Stays SCHED_OTHER; not a substitute for PREEMPT-RT.
        nice_target = int(self.get_parameter('tx_nice').value)
        if nice_target < 0:
            try:
                os.nice(nice_target)
            except (PermissionError, OSError) as e:
                self.get_logger().warn(
                    f'os.nice({nice_target}) denied ({e}); TX runs at '
                    'default priority. See byte_leg_hardware/README for '
                    'the limits.d snippet.')

        next_tick = time.monotonic()
        while not self._tx_stop.is_set():
            # ---- Snapshot shared state under self._lock ---------------
            now = time.monotonic()
            with self._lock:
                armed = self._armed
                last_traj_time = self._last_traj_time
                armed_at = self._armed_at
                traj_count = self._traj_count
                desired = list(self._desired_rad)
                current = list(self._current_cmd_rad)
                if self._last_tx_monotonic is not None:
                    self._tx_tick_periods.append(
                        now - self._last_tx_monotonic)
                self._last_tx_monotonic = now

            # ---- Watchdog (only meaningful when armed) ----------------
            if armed and last_traj_time is not None:
                gap = now - last_traj_time
                if gap > self._command_timeout_s:
                    since_arm = (now - armed_at
                                 if armed_at is not None else float('nan'))
                    self.get_logger().warn(
                        f'command timeout; disarming. gap={gap:.3f}s, '
                        f'since_arm={since_arm:.2f}s, '
                        f'trajectories_received={traj_count}')
                    self._do_disarm()
                    # Re-snapshot armed flag after disarm.
                    armed = False

            if armed:
                # Rate-limit current command toward desired. Only when
                # armed — otherwise the seeded value from _srv_arm gets
                # eroded toward whatever IK was computing before arm
                # completed.
                new_current = list(current)
                for i, j in enumerate(self._joints):
                    max_step = j.max_rate_rad_s * self._cmd_dt
                    err = desired[i] - current[i]
                    step = max(-max_step, min(max_step, err))
                    new_current[i] = current[i] + step

                # Coalesce all three SET_INPUT_POS frames into one
                # ser.write so they hand off to the USB-CDC bulk endpoint
                # as a single transfer (1 syscall instead of 3, and no
                # sub-frame interleave with arm/disarm writes).
                if not self._dry_run:
                    tx_buf = bytearray()
                    for i, j in enumerate(self._joints):
                        rev = cw.joint_rad_to_motor_rev(
                            new_current[i],
                            gear_ratio=j.gear_ratio,
                            sign=j.sign,
                            zero_offset_rev=j.zero_offset_rev,
                        )
                        tx_buf += cw.build_set_input_pos(j.node_id, rev)
                    try:
                        with self._ser_lock:
                            self._ser.write(bytes(tx_buf))
                    except Exception as e:  # pragma: no cover - hw-only
                        self.get_logger().error(f'tx write failed: {e}')

                # Persist the rate-limited command back so _srv_arm and
                # the diag publisher see a coherent picture.
                with self._lock:
                    self._current_cmd_rad = new_current

            # Sleep until next tick. time.sleep releases the GIL so other
            # Python threads (including the rclpy executor) get to run.
            next_tick += self._cmd_dt
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Fell behind — reset the schedule rather than running
                # back-to-back ticks in catch-up mode.
                next_tick = time.monotonic()

    # ================================================================== #
    #  State publishing
    # ================================================================== #
    def _publish_joint_states(self) -> None:
        with self._lock:
            pos_rev = list(self._meas_pos_rev)
            vel_rev = list(self._meas_vel_rev_s)

        # Hot path — keep allocations and attribute lookups minimal so a
        # 100 Hz timer doesn't push the executor over the GIL switch
        # budget when the host is busy.
        n = len(self._joints)
        joints = self._joints
        rev_to_rad = self._rev_to_rad
        positions = [0.0] * n
        velocities = [0.0] * n
        for i in range(n):
            p = pos_rev[i]
            if p is None:
                continue
            scale = rev_to_rad[i]
            positions[i] = (p - joints[i].zero_offset_rev) * scale
            v = vel_rev[i]
            if v is not None:
                velocities[i] = v * scale

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names_const)
        msg.position = positions
        msg.velocity = velocities
        self._pub_joint_states.publish(msg)
        # Encoder data arrives via ODrive's cyclic broadcasts configured
        # by encoder_msg_rate_ms — see byte_leg_hardware/README. The bridge
        # never polls (the old zero-data request was a silent no-op).

    def _publish_diag(self) -> None:
        now = time.monotonic()
        with self._lock:
            tick_samples = list(self._tx_tick_periods)
            rx_count = self._rx_frame_count
            sync_resyncs = self._sync_resync_count
            pos_times = list(self._meas_pos_time)
            last_sample_time = self._diag_last_sample_time
            last_rx_count = self._diag_last_rx_count
            self._diag_last_sample_time = now
            self._diag_last_rx_count = rx_count

        if tick_samples:
            sorted_samples = sorted(tick_samples)
            # p99 via bisect on the sorted view; for our small sample
            # window this is cheaper to read than to think about.
            idx = max(0, min(len(sorted_samples) - 1,
                             int(0.99 * len(sorted_samples))))
            tx_p99_ms = sorted_samples[idx] * 1000.0
        else:
            tx_p99_ms = 0.0

        if last_sample_time is not None and now > last_sample_time:
            rx_rate = (rx_count - last_rx_count) / (now - last_sample_time)
        else:
            rx_rate = 0.0

        ages_ms = []
        for t in pos_times:
            if t is None:
                ages_ms.append(float('inf'))
            else:
                ages_ms.append((now - t) * 1000.0)
        encoder_age_ms_max = max(ages_ms) if ages_ms else 0.0
        # Float32 can't carry +inf cleanly via topic_echo; clamp.
        if encoder_age_ms_max == float('inf'):
            encoder_age_ms_max = 99999.0

        msg = Float32MultiArray()
        msg.data = [float(tx_p99_ms), float(rx_rate),
                    float(sync_resyncs), float(encoder_age_ms_max)]
        self._pub_diag.publish(msg)

    # ================================================================== #
    #  Services
    # ================================================================== #
    def _srv_arm(self, req: Trigger.Request,
                 res: Trigger.Response) -> Trigger.Response:
        if self._read_only:
            res.success = False
            res.message = ('read_only mode is on; relaunch without '
                           'read_only:=true to arm.')
            return res
        if self._dry_run:
            res.success = False
            res.message = ('dry_run is true; motors cannot be armed. Relaunch '
                           'with dry_run:=false after sign/offset checks.')
            return res
        with self._lock:
            already_armed = self._armed
            meas = list(self._meas_pos_rev)
        if already_armed:
            res.success = True
            res.message = 'already armed'
            return res
        if any(v is None for v in meas):
            res.success = False
            res.message = ('no encoder data received yet from all nodes; '
                           'check wiring and that the ODrives are powered. '
                           'Confirm odrv0.axisN.config.can.encoder_msg_rate_ms '
                           'is non-zero (see byte_leg_hardware/README).')
            return res

        # Seed the rate-limited target from the current measured position
        # so arming does not cause a snap, regardless of where IK is
        # asking us to go.
        seeded_rad = []
        for i, j in enumerate(self._joints):
            rad = cw.motor_rev_to_joint_rad(
                meas[i], gear_ratio=j.gear_ratio, sign=j.sign,
                zero_offset_rev=j.zero_offset_rev)
            seeded_rad.append(rad)
        with self._lock:
            self._current_cmd_rad = seeded_rad[:]
            # Override the IK desired for this tick so we don't immediately
            # start chasing an old stick direction.
            self._desired_rad = seeded_rad[:]

        for j in self._joints:
            self.get_logger().info(
                f'energizing node {j.node_id} ({j.name})')
            # Belt-and-suspenders IDLE first; CyberBeast 0.6.4 silently
            # ignores CLOSED_LOOP requests without an explicit IDLE
            # transition.
            with self._ser_lock:
                cw.set_axis_state(self._ser, j.node_id, cw.AXIS_IDLE)
            time.sleep(0.5)
            # Pre-seed input_pos to where the leg currently is so the
            # controller has nothing to slew toward when it enters
            # CLOSED_LOOP.
            rev = cw.joint_rad_to_motor_rev(
                seeded_rad[self._index_by_name[j.name]],
                gear_ratio=j.gear_ratio, sign=j.sign,
                zero_offset_rev=j.zero_offset_rev)
            with self._ser_lock:
                cw.set_input_pos(self._ser, j.node_id, rev)
                cw.set_axis_state(
                    self._ser, j.node_id, cw.AXIS_CLOSED_LOOP)
            time.sleep(1.0)
        # Stamp the watchdog into the future so the user has `arm_grace_s`
        # to grab the gamepad before the command-timeout disarm kicks in.
        # The first real /leg_controller/joint_trajectory will overwrite
        # this with a normal monotonic timestamp.
        now = time.monotonic()
        with self._lock:
            self._last_traj_time = now + self._arm_grace_s
            self._armed_at = now
            self._traj_count = 0
            self._armed = True
        self.get_logger().info(
            'armed; holding at ' + ', '.join(
                f'{j.name}={seeded_rad[i]:+.3f} rad'
                for i, j in enumerate(self._joints)))
        res.success = True
        res.message = 'armed'
        return res

    def _srv_disarm(self, req: Trigger.Request,
                    res: Trigger.Response) -> Trigger.Response:
        self._do_disarm()
        res.success = True
        res.message = 'disarmed'
        return res

    def _do_disarm(self) -> None:
        # Flip the flag first so the TX thread immediately stops sending
        # SET_INPUT_POS while we issue the IDLE frames below.
        with self._lock:
            self._armed = False
        if not self._dry_run:
            # Belt-and-suspenders: two passes with delays. Back-to-back
            # SET_AXIS_STATE writes without spacing have been observed to
            # leave a node still in CLOSED_LOOP, presumably from a dropped
            # frame on the bus. Coalesce each pass into a single ser.write
            # so the three frames hit the wire as one USB transfer.
            idle_burst = b''.join(
                cw.build_set_axis_state(j.node_id, cw.AXIS_IDLE)
                for j in self._joints)
            for _attempt in range(2):
                try:
                    with self._ser_lock:
                        self._ser.write(idle_burst)
                        self._ser.flush()
                except Exception as e:  # pragma: no cover - hw-only
                    self.get_logger().error(f'disarm write failed: {e}')
                time.sleep(0.1)
        self.get_logger().info('disarmed')

    def _srv_calibrate_home(self, req: Trigger.Request,
                            res: Trigger.Response) -> Trigger.Response:
        with self._lock:
            armed = self._armed
            meas = list(self._meas_pos_rev)
        if armed:
            res.success = False
            res.message = 'disarm before calibrating'
            return res
        if any(v is None for v in meas):
            res.success = False
            res.message = ('no encoder data yet; wait a moment and try again')
            return res
        new_offsets = [float(v) for v in meas]
        for i, j in enumerate(self._joints):
            j.zero_offset_rev = new_offsets[i]
        self._persist_calibration(new_offsets)
        self.get_logger().info(
            'calibrated home: ' + ', '.join(
                f'{self._joints[i].name}={new_offsets[i]:+.4f} rev'
                for i in range(len(self._joints))))
        res.success = True
        res.message = 'calibrated; /joint_states will now read ~0 here'
        return res

    # ================================================================== #
    #  Calibration persistence
    # ================================================================== #
    def _persist_calibration(self, offsets: list[float]) -> None:
        payload = {
            'zero_offset_rev': {
                self._joints[i].name: offsets[i]
                for i in range(len(self._joints))
            }
        }
        try:
            with open(self._calibration_file, 'w') as f:
                yaml.safe_dump(payload, f)
        except OSError as e:
            self.get_logger().warn(
                f'could not persist calibration to {self._calibration_file}: {e}')

    def _maybe_load_calibration(self) -> None:
        path = self._calibration_file
        if not os.path.exists(path):
            return
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except OSError as e:
            self.get_logger().warn(f'could not read {path}: {e}')
            return
        offsets = (data.get('zero_offset_rev') or {})
        for j in self._joints:
            if j.name in offsets:
                j.zero_offset_rev = float(offsets[j.name])
        self.get_logger().info(f'loaded calibration from {path}')

    # ================================================================== #
    #  Shutdown
    # ================================================================== #
    def shutdown(self) -> None:
        self.get_logger().info('disarming all nodes before exit')
        try:
            self._do_disarm()
        except Exception as e:  # pragma: no cover
            self.get_logger().error(f'disarm on shutdown failed: {e}')
        self._tx_stop.set()
        if self._tx_thread.is_alive():
            self._tx_thread.join(timeout=1.0)
        self._rx_stop.set()
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:  # pragma: no cover
            pass


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CanRelay()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
