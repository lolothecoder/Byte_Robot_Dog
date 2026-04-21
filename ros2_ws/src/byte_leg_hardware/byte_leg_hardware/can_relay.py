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
from dataclasses import dataclass

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
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
        self.declare_parameter('max_joint_rad', [1.8, 1.8, 2.5])
        self.declare_parameter('max_rate_rad_s', [4.0, 4.0, 6.0])
        self.declare_parameter('command_timeout_s', 0.25)
        self.declare_parameter('calibration_file', '/tmp/byte_leg_calibration.yaml')
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('state_publish_hz', 100.0)
        self.declare_parameter('iq_request_hz', 30.0)

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

        self._dry_run: bool = bool(self.get_parameter('dry_run').value)
        self._command_timeout_s: float = float(
            self.get_parameter('command_timeout_s').value)
        self._calibration_file: str = str(
            self.get_parameter('calibration_file').value)

        # --- State guarded by self._lock ----------------------------------
        self._lock = threading.Lock()
        n = len(self._joints)
        self._meas_pos_rev: list[float | None] = [None] * n
        self._meas_vel_rev_s: list[float | None] = [None] * n
        self._meas_iq: list[float | None] = [None] * n

        # --- State owned by the main (ROS) thread -------------------------
        # Desired joint angle fed by the IK node (clamped but not rate-limited).
        self._desired_rad: list[float] = [0.0] * n
        # Rate-limited joint angle actually sent to the motor each tick.
        self._current_cmd_rad: list[float] = [0.0] * n
        self._armed: bool = False
        self._last_traj_time: float | None = None

        # Try to load a previously saved calibration.
        self._maybe_load_calibration()

        # --- Serial ------------------------------------------------------
        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        self.get_logger().info(f'Opening {port} @ {baud} baud')
        try:
            self._ser = serial.Serial(port, baud, timeout=0.01)
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
        self._pub_iq = self.create_publisher(
            Float32MultiArray, '/real_leg/iq', 10)
        self.create_subscription(
            JointTrajectory, '/leg_controller/joint_trajectory',
            self._on_trajectory, 10)

        self.create_service(Trigger, '/real_leg/arm', self._srv_arm)
        self.create_service(Trigger, '/real_leg/disarm', self._srv_disarm)
        self.create_service(
            Trigger, '/real_leg/calibrate_home', self._srv_calibrate_home)

        cmd_rate = float(self.get_parameter('command_rate_hz').value)
        state_rate = float(self.get_parameter('state_publish_hz').value)
        iq_rate = float(self.get_parameter('iq_request_hz').value)
        self._cmd_dt = 1.0 / cmd_rate
        self.create_timer(self._cmd_dt, self._command_tick)
        self.create_timer(1.0 / state_rate, self._publish_joint_states)
        self.create_timer(1.0 / iq_rate, self._request_iq)

        banner = '[DRY-RUN] ' if self._dry_run else ''
        self.get_logger().info(
            f'{banner}can_relay ready. Joints: ' + ', '.join(
                f'{j.name}->node{j.node_id}' for j in self._joints))

    # ================================================================== #
    #  RX thread
    # ================================================================== #
    def _rx_loop(self) -> None:
        buf = bytearray()
        while not self._rx_stop.is_set():
            try:
                if self._ser.in_waiting > 0:
                    buf.extend(self._ser.read(self._ser.in_waiting))
            except Exception as e:  # pragma: no cover - hw-only
                self.get_logger().error(f'serial read error: {e}')
                time.sleep(0.05)
                continue

            for arb_id, payload in cw.iter_frames(buf):
                node_id, cmd_id = cw.split_arb_id(arb_id)
                if cmd_id == cw.CMD_GET_ENCODER_ESTIMATES and len(payload) == 8:
                    pos, vel = cw.decode_encoder_estimates(payload)
                    idx = self._index_for_node(node_id)
                    if idx is not None:
                        with self._lock:
                            self._meas_pos_rev[idx] = pos
                            self._meas_vel_rev_s[idx] = vel
                elif cmd_id == cw.CMD_GET_IQ and len(payload) == 8:
                    _sp, iq_meas = cw.decode_iq(payload)
                    idx = self._index_for_node(node_id)
                    if idx is not None:
                        with self._lock:
                            self._meas_iq[idx] = iq_meas

            time.sleep(0.002)

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
        new_desired = list(self._desired_rad)
        for k, jname in enumerate(names):
            idx = self._index_by_name.get(jname)
            if idx is None or k >= len(positions):
                continue
            lim = self._joints[idx].max_joint_rad
            new_desired[idx] = max(-lim, min(lim, float(positions[k])))
        self._desired_rad = new_desired
        self._last_traj_time = time.monotonic()

    def _command_tick(self) -> None:
        # Stale-command watchdog: if we're armed and haven't heard from IK
        # in a while, disarm. Skip if we've never received one yet.
        if self._armed and self._last_traj_time is not None:
            if time.monotonic() - self._last_traj_time > self._command_timeout_s:
                self.get_logger().warn(
                    'command timeout; disarming for safety')
                self._do_disarm()
                return

        # Rate-limit current command toward desired.
        for i, j in enumerate(self._joints):
            max_step = j.max_rate_rad_s * self._cmd_dt
            err = self._desired_rad[i] - self._current_cmd_rad[i]
            step = max(-max_step, min(max_step, err))
            self._current_cmd_rad[i] += step

        if not self._armed:
            return

        # Send SET_INPUT_POS to each motor.
        for i, j in enumerate(self._joints):
            rev = cw.joint_rad_to_motor_rev(
                self._current_cmd_rad[i],
                gear_ratio=j.gear_ratio,
                sign=j.sign,
                zero_offset_rev=j.zero_offset_rev,
            )
            if self._dry_run:
                self.get_logger().debug(
                    f'[DRY-RUN] SET_INPUT_POS node={j.node_id} rev={rev:.4f} '
                    f'(joint={self._current_cmd_rad[i]:+.3f} rad)')
            else:
                cw.set_input_pos(self._ser, j.node_id, rev)

    # ================================================================== #
    #  State publishing
    # ================================================================== #
    def _publish_joint_states(self) -> None:
        with self._lock:
            pos_rev = list(self._meas_pos_rev)
            vel_rev = list(self._meas_vel_rev_s)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [j.name for j in self._joints]
        positions = []
        velocities = []
        for i, j in enumerate(self._joints):
            if pos_rev[i] is None:
                # No data yet — report 0 to keep the array length consistent.
                positions.append(0.0)
                velocities.append(0.0)
                continue
            positions.append(cw.motor_rev_to_joint_rad(
                pos_rev[i], gear_ratio=j.gear_ratio, sign=j.sign,
                zero_offset_rev=j.zero_offset_rev))
            # velocity in rev/s -> joint rad/s, same scale factor
            v = 0.0 if vel_rev[i] is None else vel_rev[i]
            velocities.append(v * 6.283185307179586 / (j.sign * j.gear_ratio))
        msg.position = positions
        msg.velocity = velocities
        self._pub_joint_states.publish(msg)

        # Also request fresh encoder estimates from each node.
        for j in self._joints:
            try:
                cw.request_encoder_estimates(self._ser, j.node_id)
            except Exception as e:  # pragma: no cover - hw-only
                self.get_logger().error(f'encoder req failed: {e}')

    def _request_iq(self) -> None:
        for j in self._joints:
            try:
                cw.request_iq(self._ser, j.node_id)
            except Exception as e:  # pragma: no cover - hw-only
                self.get_logger().error(f'iq req failed: {e}')
        # Publish the last known iq values.
        with self._lock:
            iqs = [0.0 if v is None else float(v) for v in self._meas_iq]
        msg = Float32MultiArray()
        msg.data = iqs
        self._pub_iq.publish(msg)

    # ================================================================== #
    #  Services
    # ================================================================== #
    def _srv_arm(self, req: Trigger.Request,
                 res: Trigger.Response) -> Trigger.Response:
        if self._dry_run:
            res.success = False
            res.message = ('dry_run is true; motors cannot be armed. Relaunch '
                           'with dry_run:=false after sign/offset checks.')
            return res
        if self._armed:
            res.success = True
            res.message = 'already armed'
            return res

        # Seed the rate-limited target from the current measured position
        # so arming does not cause a snap, regardless of where IK is
        # asking us to go.
        with self._lock:
            meas = list(self._meas_pos_rev)
        if any(v is None for v in meas):
            res.success = False
            res.message = ('no encoder data received yet from all nodes; '
                           'check wiring and that the ODrives are powered')
            return res

        seeded_rad = []
        for i, j in enumerate(self._joints):
            rad = cw.motor_rev_to_joint_rad(
                meas[i], gear_ratio=j.gear_ratio, sign=j.sign,
                zero_offset_rev=j.zero_offset_rev)
            seeded_rad.append(rad)
        self._current_cmd_rad = seeded_rad[:]
        # Override the IK desired for this tick so we don't immediately
        # start chasing an old stick direction.
        self._desired_rad = seeded_rad[:]
        self._last_traj_time = time.monotonic()

        for j in self._joints:
            cw.set_axis_state(self._ser, j.node_id, cw.AXIS_CLOSED_LOOP)
            # Pre-arm sync: tell each ODrive its setpoint = where it is now.
            rev = cw.joint_rad_to_motor_rev(
                seeded_rad[self._index_by_name[j.name]],
                gear_ratio=j.gear_ratio, sign=j.sign,
                zero_offset_rev=j.zero_offset_rev)
            cw.set_input_pos(self._ser, j.node_id, rev)
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
        if not self._dry_run:
            for j in self._joints:
                try:
                    cw.set_axis_state(self._ser, j.node_id, cw.AXIS_IDLE)
                except Exception as e:  # pragma: no cover - hw-only
                    self.get_logger().error(f'disarm write failed: {e}')
        self._armed = False
        self.get_logger().info('disarmed')

    def _srv_calibrate_home(self, req: Trigger.Request,
                            res: Trigger.Response) -> Trigger.Response:
        if self._armed:
            res.success = False
            res.message = 'disarm before calibrating'
            return res
        with self._lock:
            meas = list(self._meas_pos_rev)
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
