# byte_leg_hardware

ROS 2 bridge from `/leg_controller/joint_trajectory` to ODrive CAN Simple over a Waveshare USB-CAN adapter. See repo root `CLAUDE.md` for the bring-up procedure and safety invariants.

## Required ODrive configuration (one-time)

The bridge does **not** poll for encoder data. It relies on each ODrive
emitting cyclic frames. Without this configuration, `/joint_states`
will report zeros and `/real_leg/arm` will refuse with "no encoder data
received yet".

For each axis (knee = node 1, hip_abduct = node 3, hip_pitch = node 5),
in `odrivetool`:

```python
axis = odrv0.axisN  # N = 0 (only one axis on this firmware)
axis.config.can.encoder_msg_rate_ms = 10   # 100 Hz cyclic encoder broadcast
axis.config.can.iq_msg_rate_ms = 0         # off; bump to ~50 if you want diagnostics
odrv0.save_configuration()
```

(The control mode + input mode + current limit are still expected to be
configured by `tune.py` / `odrivetool` per the repo `CLAUDE.md`.)

## /real_leg/diag

Published at 2 Hz, layout `Float32MultiArray`:

| index | field                  | unit | meaning                                                    |
|-------|------------------------|------|------------------------------------------------------------|
| 0     | tx_tick_period_ms_p99  | ms   | 99th-percentile period between TX-thread iterations        |
| 1     | rx_frames_per_s        | Hz   | encoder frames decoded per second (sum across joints)      |
| 2     | sync_resync_count      | —    | cumulative bytes dropped resyncing the Waveshare envelope  |
| 3     | encoder_age_ms_max     | ms   | oldest per-joint encoder snapshot age                      |

Healthy bench numbers with `command_rate_hz: 50` and
`encoder_msg_rate_ms = 10`:

- `tx_tick_period_ms_p99 < 25` (target tick is 20 ms)
- `rx_frames_per_s ≈ 300` (3 joints × 100 Hz)
- `sync_resync_count` flat after warmup
- `encoder_age_ms_max < 30`

A growing `sync_resync_count` means the bus or USB link is dropping
bytes; either RFI, a flaky cable, or a second process accidentally
reading from the same `/dev/ttyUSB0` (the bridge now opens with
`exclusive=True` to make that fail loudly).

## Letting `tx_nice` actually take effect

The TX thread tries `os.nice(-5)` to reduce tail latency on stock
kernels. Without `CAP_SYS_NICE` it logs a warning and continues at
default niceness. To allow it for your user (no PREEMPT-RT needed):

```
sudo tee /etc/security/limits.d/byte-leg.conf <<'EOF'
@dialout    -    nice    -10
EOF
```

Log out and back in. Replace `@dialout` with the user/group that runs
`ros2 launch`. To disable the nudge entirely, set `tx_nice: 0` in
`config/hardware.yaml`.
