# hip_pitch (node 5) won't move — diagnostic guide

> All commands below run on the **Jetson** (the machine wired to the leg).
> This file lives on the dev machine and is pulled to the Jetson via git.

## TL;DR — the most likely cause

**In cartesian mode (`joy_teleop` + `ik_node`), only LY drives hip_pitch.**
LX → Y → hip_abduct. RY → Z → mostly knee. Pushing LX and RY produces
near-zero `q_hp` change, so hip_pitch looks dead. Push **left stick
forward/back all the way** and watch `/joint_states.position[1]`. If it
moves, you are not actually broken — that was the resolution last time.

Rate limit (`max_rate_rad_s[1] = 0.4 rad/s`) is fine — same value drives
hip_abduct and that one is responsive. Don't bump it as a first step.
hip_pitch carries more load (lifts the leg against gravity) than
hip_abduct, so at the same rate cap it *feels* sluggish but is not
actually capped — confirm by watching `/joint_states.position[1]` track
the stick.

## Quickest non-ROS sanity check

Before debugging the ROS stack, confirm the motor responds end-to-end
with no controller / IK / bridge in the loop:

```bash
python3 joy_drive.py
```

Standalone bench teleop. Hold RB, push RY → hip_pitch should move. If
it moves here but not under `real.launch.py`, the failure is somewhere
in joy → joy_teleop / ik_node → can_relay, not in the motor or CAN.
If it doesn't move here either, skip to ODrive health (Step 3 below).

## What we already know

`tune.py` and `small_walk.py` both drive node 5 directly over CAN — the
motor, ODrive axis, Waveshare adapter, and 60 Ω bus are healthy. The
encoder broadcast on node 5 also works (`/joint_states position[1]`
tracks the link when moved by hand with motors disarmed). So the failure
is somewhere in:

```
joy → joy_teleop / joy_per_joint → /leg_controller/joint_trajectory
                                         → can_relay → SET_INPUT_POS
```

The bridge code (`can_relay.py`) is symmetric — there is no per-node
branch for node 5. The only joint-specific config is in
`ros2_ws/src/byte_leg_hardware/config/hardware.yaml`:

- `sign: [1.0, -1.0, 1.0]` — hip_pitch reversed vs URDF axis
- `max_rate_rad_s: [0.4, 0.4, 2.0]` — hips slow on purpose
- `max_joint_rad: [0.2618, 0.5236, 0.7854]` — ±15° / ±30° / ±45°

## ⚠ STEP 0 — re‑calibrate before doing anything else

**Skip this and the leg will go crazy on arm.** A stale or wrong
`zero_offset_rev[1]` (e.g. left over from a prior calibrate at not-zero
pose) puts the seeded `_current_cmd_rad[1]` somewhere ODrive can't reach,
and the rate limiter quietly drives the motor to the limit.

On the Jetson:

1. Position the leg at **URDF zero** by hand: hanging straight down,
   knee straight, hip-abduct centered.
2. Launch in dry-run (default), motors stay IDLE:

   ```bash
   cd ~/byte_ws/ros2_ws
   source install/setup.bash
   ros2 launch byte_leg_bringup real.launch.py
   ```

3. In another terminal, with the leg held still at URDF zero:

   ```bash
   ros2 service call /real_leg/calibrate_home std_srvs/srv/Trigger
   ```

4. Confirm it persisted:

   ```bash
   cat /tmp/byte_leg_calibration.yaml
   ```

5. Stop the launch (Ctrl-C) before moving on.

## Step 1 — the per-joint A/B swap test (manual)

`byte_leg_control/joy_per_joint.py:27-32` is now back to the canonical
mapping (`LY → knee`, `RY → hip_pitch`, `LX → hip_abduct`). To re-run
the A/B swap diagnostic, **manually edit those three lines** to swap
`axis_knee` and `axis_hip_pitch`:

```python
self.declare_parameter('axis_knee', 4)         # RY (swapped, diagnostic only)
self.declare_parameter('axis_hip_pitch', 1)    # LY (swapped, diagnostic only)
self.declare_parameter('axis_hip_abduct', 0)   # LX
```

Rebuild + relaunch:

```bash
colcon build --symlink-install --packages-select byte_leg_control
source install/setup.bash
ros2 launch byte_leg_bringup real.launch.py dry_run:=false per_joint:=true
ros2 service call /real_leg/arm std_srvs/srv/Trigger
```

Hold **RB**, push **LY** — hip_pitch should move. Push **RY** — knee
should move.

### Interpret the result

| LY → hip_pitch? | RY → knee? | Conclusion |
|---|---|---|
| ✅ moves | ✅ moves | Axes are fine. Probably you weren't pushing the right stick in cartesian mode. **Revert the swap.** |
| ✅ moves | ❌ stuck | Axis 4 (RY) is dead in this joy stack. Stock `joy_node` doesn't see the F710 properly on Tegra. Fix: run `ros2 run byte_leg_hardware f710_usb_joy` in a separate terminal each session, with the F710 slider set to **D**. |
| ❌ stuck | ✅ moves | Axes fine, hip_pitch motor command path is broken. Skip to Step 2. |
| ❌ stuck | ❌ stuck | Joy stack is dead entirely. `ros2 topic echo /joy --once` while pushing sticks — should see non-zero `axes`. If empty, see `f710_usb_joy` note above. |

**Revert the swap when done** so future-you doesn't get confused by
non-canonical mappings hiding in source control.

## Step 2 — desired vs measured for hip_pitch

If the swap proves the joy stack is OK and node 5 is still stuck, find
out which side of the bridge fails:

```bash
ros2 topic echo /leg_controller/joint_trajectory --field points
ros2 topic echo /joint_states --field position
ros2 topic echo /real_leg/diag
```

While pushing the stick:

- `joint_trajectory.points[0].positions[1]` flat → bug is in
  `joy_per_joint` / `ik_node`. Trajectory never asks for hip_pitch
  motion.
- `positions[1]` changes but `/joint_states.position[1]` doesn't follow
  → bridge sends the command but the motor isn't executing it.
  ODrive-side issue (see Step 3).
- `/real_leg/diag` `tx_tick_period_ms_p99` should be ~20 ms (50 Hz). If
  it's much higher, TX is starved.

## Step 3 — ODrive node 5 health

Connect the Jetson to the ODrive USB:

```bash
odrivetool --path serial:/dev/ttyACM0
```

Then in the prompt:

```python
odrv0.axis0.current_state      # should be 8 (CLOSED_LOOP_CONTROL) after arm
odrv0.axis0.error
odrv0.axis0.motor.error
odrv0.axis0.controller.error
odrv0.axis0.controller.config.input_pos_limit
odrv0.axis0.config.can.encoder_msg_rate_ms   # must be 10
odrv0.axis0.config.can.node_id               # must be 5
```

If any error is non-zero: `dump_errors(odrv0, True)` to see + clear, then
`odrv0.save_configuration()`.

## Step 4 — sign sanity

With motors **disarmed**, hand-rotate the hip_pitch link in the URDF +y
direction (which is the convention `+q_hp` should produce). Watch
`/joint_states.position[1]`:

- Goes **positive** → `sign[1]` is correct.
- Goes **negative** → `sign[1]` in `hardware.yaml` is wrong. Flip it
  between `-1.0` and `+1.0`, rebuild (`colcon build --symlink-install
  --packages-select byte_leg_hardware`), redo Step 0 calibration, retest.

## Quick reference — files that matter

| File | Purpose |
|---|---|
| `joy_drive.py` | non-ROS bench teleop. Quickest "is the motor alive?" check. |
| `tune.py` | single-motor REPL (edit `NODE = 5` for hip_pitch). |
| `ros2_ws/src/byte_leg_hardware/config/hardware.yaml` | sign / gear / limits per joint |
| `ros2_ws/src/byte_leg_hardware/byte_leg_hardware/can_relay.py` | bridge — symmetric, no per-node logic |
| `ros2_ws/src/byte_leg_control/byte_leg_control/joy_per_joint.py` | direct stick → joint (canonical mapping; manually swap for A/B test) |
| `ros2_ws/src/byte_leg_control/config/teleop.yaml` | cartesian axis mapping |
| `/tmp/byte_leg_calibration.yaml` | persisted `zero_offset_rev` (Jetson only) |

## End-to-end verification

After whichever fix landed:

1. Re-run Step 0 (calibrate at URDF zero in dry-run).
2. Relaunch with `dry_run:=false`, do not touch the stick, then arm.
3. Push only the stick that should drive hip_pitch. Confirm
   `/joint_states.position[1]` follows within ~0.01 rad of the commanded
   shape.
4. Sanity-check the others still work.
5. Re-run `python tune.py` on node 5 to confirm the ODrive config wasn't
   perturbed.
