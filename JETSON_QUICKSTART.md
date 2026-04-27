# Jetson quick-start

Cheat sheet for bringing up the leg on the Orin Nano via Docker. For
deeper notes (safety invariants, full bring-up rationale, sim workspace),
see `CLAUDE.md`.

## One-time setup (already done — keep for reference)

```bash
# udev rule so pyusb can talk to the F710 receiver without sudo
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="046d", ATTR{idProduct}=="c219", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-logitech-f710.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

ODrive flash settings (one-time per ODrive, via odrivetool on dev box):
- `axis0.config.can.encoder_msg_rate_ms = 10` (or `20` — bridge handles either)
- `axis0.config.can.iq_msg_rate_ms = 0`
- `axis0.controller.config.control_mode = 3` (POSITION)
- `axis0.controller.config.input_mode = 1` (PASSTHROUGH)
- `axis0.motor.config.current_lim` — at least 10 A for hip_pitch under load
- `odrv0.save_configuration()`

## Hardware checklist before launching

1. Waveshare USB-CAN plugged in → `ls /dev/ttyUSB0`
2. Logitech F710 receiver plugged in, slider on **D** → `lsusb | grep DirectInput`
3. Leg at URDF-zero (hanging straight, knee straight)

## The launch dance — three shells

Everything runs **inside the container**. Each shell is one terminal tab.

### Shell 1 — main launch

```bash
cd ~/Documents/Byte_Robot_Dog
docker compose run --rm leg
```

Inside the container:
```bash
# only on first run after a code change:
colcon build --symlink-install --packages-skip byte_leg_gazebo
source install/setup.bash

# pick one:
ros2 launch byte_leg_bringup real.launch.py read_only:=true                        # encoder validation only
ros2 launch byte_leg_bringup real.launch.py dry_run:=false                         # cartesian teleop
ros2 launch byte_leg_bringup real.launch.py dry_run:=false per_joint:=true         # per-joint teleop
```

Default is `dry_run:=true` (motors stay IDLE, **no** encoder data published — ODrive only broadcasts encoders in CLOSED_LOOP).

### Shell 2 — F710 joy bridge

```bash
docker compose exec leg bash
source install/setup.bash
ros2 run byte_leg_hardware f710_usb_joy
```

Republishes the F710 over libusb as `/joy`. Needed because the Tegra
kernel ships without `xpad` and without `uinput`, so `joy_node` can't
see the controller through the normal kernel input path.

### Shell 3 — arm / disarm

```bash
docker compose exec leg bash
source install/setup.bash

# arm (don't touch the stick first):
ros2 service call /real_leg/arm std_srvs/srv/Trigger

# disarm:
ros2 service call /real_leg/disarm std_srvs/srv/Trigger

# calibrate home (with leg at URDF-zero, after any setup change):
ros2 service call /real_leg/calibrate_home std_srvs/srv/Trigger
```

Or just Ctrl+C the launch in Shell 1 — the shutdown hook IDLEs all
three motors before exiting.

## Driving

| input | cartesian mode | per_joint mode |
|-------|----------------|----------------|
| RB (deadman) | hold to enable motion | same |
| Left stick fwd/back (LY) | foot moves +X / -X | knee (node 1) |
| Left stick left/right (LX) | foot moves -Y / +Y | hip_abduct (node 3) |
| Right stick fwd/back (RY) | foot moves +Z / -Z | hip_pitch (node 5) |
| A | snap to home / zero | same |
| Release RB | foot decays back to home | each joint decays to 0 |

## Health monitoring

```bash
ros2 topic hz /joint_states            # ~100 Hz aggregate (or 150 if encoder rate = 20 ms)
ros2 topic echo /real_leg/diag --once  # [tx_p99_ms, rx_fps, sync_resync, encoder_age_ms]
```

Healthy targets:
- `tx_tick_period_ms_p99 < 25`
- `rx_frames_per_s ≈ 300` at 10 ms encoder rate, `≈ 150` at 20 ms
- `sync_resync_count` flat after warmup
- `encoder_age_ms_max < 30`

## Tuning a single motor

Stop the launch first (only one process can own `/dev/ttyUSB0`):

```bash
# Edit NODE in tune.py: knee=1, hip_abduct=3, hip_pitch=5
docker compose exec leg bash    # or: docker compose run --rm leg
python3 /tune.py
# REPL: s <revs> step | g <pos> goto | p/v/i <val> gains | h help | q quit
```

Persist gains via `odrivetool` on the dev box (tune.py does **not** save
to ODrive flash):
```python
odrv0.axis0.controller.config.pos_gain = ...
odrv0.axis0.controller.config.vel_gain = ...
odrv0.axis0.controller.config.vel_integrator_gain = ...
odrv0.save_configuration()
```

## Common gotchas

- **No `/joint_states` data?** ODrive only broadcasts encoder frames in
  CLOSED_LOOP_CONTROL. Use `read_only:=true` for validation, or arm
  via `/real_leg/arm` after `dry_run:=false`.
- **`save_configuration()` returns `False` in odrivetool?** Axis is in
  CLOSED_LOOP. Set `requested_state = 1` (IDLE) first.
- **`joy_node` errors about `/dev/input/js0`** — expected. The F710
  bridge in Shell 2 is the real source of `/joy`. Ignore the noise.
- **Stale x86_64 build artifacts after a fresh `git pull`** — inside
  the container: `rm -rf build install log` then rebuild.
- **F710 flashing red, no `/joy` data?** Slider on the back must be on
  **D** (DirectInput). XInput mode needs `xpad`, which Tegra strips.
