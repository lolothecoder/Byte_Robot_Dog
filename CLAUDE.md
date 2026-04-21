# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Single-file Python tool (`tune.py`) for interactively tuning an ODrive motor controller over CAN, using a Waveshare USB-to-CAN adapter on a serial port. The script energizes the motor, locks it at its current position, then exposes a REPL for step-response tests and live gain adjustment.

## Running

```bash
python tune.py
```

Dependencies: `pyserial` (only non-stdlib import).

Hardware assumptions baked into the script (edit constants at the top of `tune.py`):
- `PORT = "/dev/ttyUSB0"` — the Waveshare adapter
- `NODE = 3` — ODrive CAN node ID
- `TORQUE_CONSTANT = 0.055` — motor KT (≈ 8.27 / KV); only affects displayed Nm
- Baud is hardcoded to `2000000`

ODrive must already be configured (via `odrivetool`) for `control_mode=POSITION(3)`, `input_mode=PASSTHROUGH(1)`, and a sane `current_lim` — the script does not set these.

REPL commands once running: `s <revs>` step test, `g <pos>` goto, `p/v/i <val>` set pos_gain / vel_gain / vel_integrator_gain, `h` help, `q` quit (sets motor to IDLE).

## Architecture

Three concerns are interleaved in `main()`:

1. **Waveshare framing layer** (`send_can`, `init_waveshare`, and the inline parser in the main loop). Every CAN frame is wrapped in a Waveshare envelope: `0xAA, 0xC0|len, arb_id_lo, arb_id_hi, <data...>, 0x55`. The arbitration ID is `(NODE << 5) | CMD_ID` — the ODrive CAN Simple convention. Any change to how frames are sent or parsed must touch both sides symmetrically. `init_waveshare` sends a one-time config packet that puts the adapter into the right mode — do not skip it.

2. **Main RX/TX loop** (bottom of `main()`). Polls `ser.in_waiting`, extracts full Waveshare packets from a rolling `bytearray` buffer, and dispatches by arbitration ID to update `position`/`velocity`/`iq_meas` (all wrapped in 1-element lists so the input thread can mutate them). It also periodically requests `Iq` (~33 Hz) and prints a status line (~10 Hz). The sync-byte recovery (`buffer.pop(0)` on mismatch) is how the parser re-aligns after a partial/corrupted frame — preserve it.

3. **Input thread** (`input_thread_fn`). A daemon thread reads stdin and mutates shared state (gains dict, `current_target`, `recording` flag). There is no lock — the code relies on GIL atomicity of single-slot list writes and dict updates. Keep shared mutations to single assignments; don't introduce multi-step read-modify-write on shared state without adding a lock.

Step-response recording is a small state machine gated by `recording[0]` and `record_start[0]`: during a 2-second window the main loop samples `position` and `iq_meas * TORQUE_CONSTANT` at fixed intervals into `pos_history`/`torque_history`, then `print_ascii_plot` renders them as terminal ASCII charts. Status-line printing is suppressed while recording so it doesn't clobber the plot.

Startup sequence in `main()` (IDLE → energize → read position → lock → apply gains) is order-sensitive; `read_position_reliable` requires the motor to already be emitting encoder estimates, which only happens after the energize step.

## ROS2 workspace (`ros2_ws/`)

Simulation-only (no hardware / no CAN bridge in this workspace) for a 3-DOF leg: hip-abduct + hip-pitch + knee, two 30 cm carbon rods, squash-ball foot. Targets ROS2 Jazzy + Gazebo Harmonic (`gz sim 8`).

### Launch

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch byte_leg_bringup sim.launch.py          # with GUI
ros2 launch byte_leg_bringup sim.launch.py gui:=false  # headless
```

Unit tests (analytical IK round-trip, 50 random reachable points):
```bash
colcon test --packages-select byte_leg_control --event-handlers console_direct+
```

### Package roles

- `byte_leg_description` — `urdf/leg.urdf.xacro`. The leg hangs from a fixed `rig_frame` at `z=1.0` via a prismatic `base_slide` joint (±0.38 m / +0.20 m), simulating the real test rig where the leg can translate vertically. Only `hip_abduct` / `hip_pitch` / `knee` are in `<ros2_control>`; `base_slide` is a passive physics joint. `rig_frame` **must keep its `<inertial>`** — without it, Gazebo's URDF-to-SDF converter prunes the link and the prismatic joint's `relative_to` graph breaks.
- `byte_leg_control` — analytical 3-DOF IK in `kinematics.py` (pure-python, no ROS deps, covered by `test/test_ik.py`). `ik_node.py` subscribes `/foot/target_point` (PointStamped in `base_link` frame, x fwd / y left / z up) and publishes `/leg_controller/joint_trajectory`. `joy_teleop.py` integrates Xbox stick deflection into the target point; **hold RB (button 5) as a deadman** or no motion occurs. Mapping is parameterised (`config/teleop.yaml`) so PS pads just need a YAML override.
- `byte_leg_gazebo` — empty `bench.sdf` world (ground plane + gravity) and the `/clock` bridge config. Joint commands/states flow through `gz_ros2_control`, not through `ros_gz_bridge`.
- `byte_leg_bringup` — `sim.launch.py`. Controllers are **auto-loaded and activated by `gz_ros2_control`** via the `<parameters>controllers.yaml</parameters>` block in the URDF `<gazebo><plugin>`. Do **not** add external `controller_manager/spawner` processes for `joint_state_broadcaster` / `leg_controller` — they duplicate the work and fail with "can not be configured from 'active' state".

### Conventions

- IK sign convention: `q_knee >= 0` means the knee flexes toward +x (elbow-down). `q_hip_pitch` comes out negative for a target directly below the hip. FK is in `kinematics.forward_kinematics` — use it as the authoritative reference when changing joint frames or axes.
- Control mode mirrors the real ODrive setup (`POSITION` + `PASSTHROUGH`), so in sim the controller is `joint_trajectory_controller` with a `position` command interface. On real hardware we bypass JTC and push `/leg_controller/joint_trajectory` positions directly to ODrive via CAN — see below.
- Frames: ROS REP-103 (x forward, y left, z up). Targets published to `/foot/target_point` are in `base_link` frame (the hip-abduct origin), not world frame.

## Real hardware (`byte_leg_hardware` + `real.launch.py`)

`ros2_ws/src/byte_leg_hardware/` provides a Python CAN bridge (`can_relay`) that subscribes to the same `/leg_controller/joint_trajectory` topic the sim's JTC consumes, converts joint radians to motor revolutions, and talks ODrive CAN Simple over a Waveshare USB-CAN adapter — identical framing to `tune.py`. Encoder + Iq are read back and published as `/joint_states` and `/real_leg/iq`. **No `ros2_control`, no JTC on the real side** — ODrive's own PASSTHROUGH position filter handles smoothing.

**Node-ID map (fixed)**: `knee = 1`, `hip_abduct = 3`, `hip_pitch = 5`. Configured in `byte_leg_hardware/config/hardware.yaml` as the `can_node_ids` list. GIM8010-8 is 8:1 motor-side, so `motor_rev = 8 × joint_rad / (2π)` — encoded in `gear_ratio: [8.0, 8.0, 8.0]`.

### Launch (defaults to dry-run)

```bash
ros2 launch byte_leg_bringup real.launch.py
# -> motors stay IDLE, /joint_states reports measured angles, any
#    SET_INPUT_POS that would be sent is logged instead.
ros2 launch byte_leg_bringup real.launch.py dry_run:=false
```

Bare launch is inherently safe: `dry_run:=true` is the default *and* `/real_leg/arm` refuses while dry-run is on. Motors only energize when the user **both** flips the flag **and** explicitly calls the arm service.

### Bring-up procedure (in order)

1. Leg at URDF-zero pose (hanging straight, knee straight).
2. `ros2 launch byte_leg_bringup real.launch.py` → motors IDLE, `/joint_states` reports angles.
3. Manually rotate each segment; watch `/joint_states` to verify sign/direction. Flip `sign[i]` in `hardware.yaml` (rebuild) for any reversed joint.
4. Rotate a joint by a known angle (e.g. 90°) and confirm `/joint_states` reports ≈1.57 rad. If off by 8×, fix `gear_ratio`.
5. Holding the leg at URDF-zero: `ros2 service call /real_leg/calibrate_home std_srvs/srv/Trigger`. Calibration persists to `/tmp/byte_leg_calibration.yaml` and is reloaded on next launch.
6. Relaunch with `dry_run:=false`.
7. **Don't touch the stick**, then `ros2 service call /real_leg/arm std_srvs/srv/Trigger`. Arm seeds the rate-limited target from current measured position, so there's no snap.
8. Drive with RB + left stick. Max speed bounded by `max_rate_rad_s` per joint.
9. `ros2 service call /real_leg/disarm std_srvs/srv/Trigger` or Ctrl+C — a shutdown hook always sends IDLE before closing the serial port.

### Safety invariants (do not remove without replacement)

- `dry_run:=true` is the launch default. Don't change it.
- `require_manual_arm` keeps motors IDLE until `/real_leg/arm`. The arm handler seeds `_current_cmd_rad` from measured encoder positions **before** flipping `_armed = True`, so the first SET_INPUT_POS written is the current pose.
- Rate limiter (`_command_tick`) caps Δrad per tick regardless of what IK asks for — the safety net against IK glitches, stick-held-at-arm, and calibration mismatches.
- Command-timeout watchdog auto-disarms after `command_timeout_s` without a trajectory. Protects against node crashes.
- `can_relay.shutdown()` sends IDLE to all three nodes; wired to both signal handlers and normal exit.
- `tune.py` stays standalone — `can_waveshare.py` duplicates the protocol so the bench tool never depends on the ROS workspace.

### Cannot run sim and real concurrently

Both publish `/joint_states` on the same default DDS namespace. Pick one. (Trivial to fix with a namespace remap once you want to, but out of scope today.)
