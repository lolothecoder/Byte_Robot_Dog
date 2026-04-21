import serial
import struct
import time
import sys
import threading

# --- Configuration ---
PORT = "/dev/ttyUSB0"
NODE = 3
TORQUE_CONSTANT = 0.055  # UPDATE for your motor: KT ≈ 8.27 / KV

# CAN Command IDs
CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_INPUT_POS = 0x0C
CMD_GET_IQ = 0x14
CMD_SET_POS_GAIN = 0x1A
CMD_SET_VEL_GAINS = 0x1B


def init_waveshare(ser):
    config = [0xAA, 0x55, 0x12, 0x03, 0x01] + [0x00] * 10
    checksum = sum(config[2:]) & 0xFF
    config.append(checksum)
    ser.write(bytes(config))
    time.sleep(0.5)


def send_can(ser, node_id, cmd_id, data):
    arb_id = (node_id << 5) | cmd_id
    packet = [0xAA, 0xC0 | len(data), arb_id & 0xFF, (arb_id >> 8) & 0xFF]
    packet.extend(data)
    packet.append(0x55)
    ser.write(bytes(packet))
    ser.flush()


def set_axis_state(ser, node_id, state):
    send_can(ser, node_id, CMD_SET_AXIS_STATE, struct.pack('<I', state))


def set_input_pos(ser, node_id, position, vel_ff=0.0, torque_ff=0.0):
    vel_ff_int = int(vel_ff * 1000)
    torque_ff_int = int(torque_ff * 1000)
    data = struct.pack('<fhh', position, vel_ff_int, torque_ff_int)
    send_can(ser, node_id, CMD_SET_INPUT_POS, data)


def set_pos_gain(ser, node_id, pos_gain):
    send_can(ser, node_id, CMD_SET_POS_GAIN, struct.pack('<f', pos_gain))


def set_vel_gains(ser, node_id, vel_gain, vel_integrator_gain):
    send_can(ser, node_id, CMD_SET_VEL_GAINS, struct.pack('<ff', vel_gain, vel_integrator_gain))


def request_iq(ser, node_id):
    arb_id = (node_id << 5) | CMD_GET_IQ
    packet = [0xAA, 0xC0 | 0, arb_id & 0xFF, (arb_id >> 8) & 0xFF, 0x55]
    ser.write(bytes(packet))
    ser.flush()


def read_position_reliable(ser, num_readings=5, timeout=10.0):
    expected_id = (NODE << 5) | CMD_GET_ENCODER_ESTIMATES
    buf = bytearray()
    readings = []
    start = time.time()

    while time.time() - start < timeout and len(readings) < num_readings:
        if ser.in_waiting > 0:
            buf.extend(ser.read(ser.in_waiting))

        while len(buf) >= 5:
            if buf[0] == 0xAA:
                data_len = buf[1] & 0x0F
                packet_len = 5 + data_len
                if len(buf) >= packet_len:
                    if buf[packet_len - 1] == 0x55:
                        aid = buf[2] | (buf[3] << 8)
                        if aid == expected_id and data_len == 8:
                            pos, vel = struct.unpack('<ff', buf[4:12])
                            readings.append(pos)
                        del buf[:packet_len]
                    else:
                        buf.pop(0)
                else:
                    break
            else:
                buf.pop(0)
        time.sleep(0.01)

    if len(readings) < num_readings:
        return None

    spread = max(readings) - min(readings)
    if spread > 0.5:
        return None

    return readings[-1]


def print_help():
    print("\n--- Commands ---")
    print("  s <revs>    Step test: move motor by <revs> from current pos")
    print("  g <pos>     Go to absolute position (revs)")
    print("  p <value>   Set pos_gain (stiffness)")
    print("  v <value>   Set vel_gain (damping)")
    print("  i <value>   Set vel_integrator_gain")
    print("  h           Show this help")
    print("  q           Quit (sets motor to IDLE)\n")


def main():
    ser = None
    position = [None]
    velocity = [None]
    iq_meas = [None]
    current_target = [None]
    buffer = bytearray()

    expected_encoder_id = (NODE << 5) | CMD_GET_ENCODER_ESTIMATES
    expected_iq_id = (NODE << 5) | CMD_GET_IQ

    gains = {
        'pos_gain': 20.0,
        'vel_gain': 0.1,
        'vel_integrator_gain': 0.0,
    }

    running = [True]

    history_len = 60
    pos_history = []
    torque_history = []
    record_start = [None]
    recording = [False]

    def do_step_test(step_size):
        if position[0] is None:
            print("  No position data yet, can't step.")
            return
        start_pos = position[0]
        target = start_pos + step_size
        current_target[0] = target
        set_input_pos(ser, NODE, target)
        print(f"  -> Step from {start_pos:.3f} to {target:.3f} ({step_size:+.3f} rev)")
        print(f"     Recording response for 2 seconds...\n")
        pos_history.clear()
        torque_history.clear()
        record_start[0] = time.time()
        recording[0] = True

    def print_ascii_plot():
        if not pos_history:
            return

        print("\n--- Step Response ---")

        target = current_target[0] if current_target[0] is not None else 0
        errors = [p - target for p in pos_history]
        if errors:
            p_min = min(errors)
            p_max = max(errors)
            p_range = p_max - p_min if p_max != p_min else 0.001

            print(f"  Position error (target={target:.3f}):")
            height = 10
            for row in range(height, -1, -1):
                val = p_min + (p_range * row / height)
                line = f"  {val:>8.4f} |"
                for e in errors:
                    norm = (e - p_min) / p_range * height
                    if abs(norm - row) < 0.5:
                        line += "*"
                    else:
                        line += " "
                print(line)
            print(f"           +{'─' * len(errors)}")
            print(f"            0s{'':>{len(errors)-4}}2s")

        if torque_history:
            t_min = min(torque_history)
            t_max = max(torque_history)
            t_range = t_max - t_min if t_max != t_min else 0.001

            print(f"\n  Torque (Nm):")
            height = 8
            for row in range(height, -1, -1):
                val = t_min + (t_range * row / height)
                line = f"  {val:>8.4f} |"
                for t in torque_history:
                    norm = (t - t_min) / t_range * height
                    if abs(norm - row) < 0.5:
                        line += "*"
                    else:
                        line += " "
                print(line)
            print(f"           +{'─' * len(torque_history)}")
        print()

    def input_thread_fn():
        while running[0]:
            try:
                line = input()
                parts = line.strip().split()
                if not parts:
                    continue
                cmd = parts[0].lower()

                if cmd == 'q':
                    running[0] = False
                    break
                elif cmd == 'h':
                    print_help()
                elif cmd == 's' and len(parts) == 2:
                    do_step_test(float(parts[1]))
                elif cmd == 'g' and len(parts) == 2:
                    target = float(parts[1])
                    current_target[0] = target
                    set_input_pos(ser, NODE, target)
                    print(f"  -> Go to {target:.3f} rev")
                elif cmd == 'p' and len(parts) == 2:
                    val = float(parts[1])
                    gains['pos_gain'] = val
                    set_pos_gain(ser, NODE, val)
                    print(f"  -> pos_gain = {val}")
                elif cmd == 'v' and len(parts) == 2:
                    val = float(parts[1])
                    gains['vel_gain'] = val
                    set_vel_gains(ser, NODE, val, gains['vel_integrator_gain'])
                    print(f"  -> vel_gain = {val}")
                elif cmd == 'i' and len(parts) == 2:
                    val = float(parts[1])
                    gains['vel_integrator_gain'] = val
                    set_vel_gains(ser, NODE, gains['vel_gain'], val)
                    print(f"  -> vel_integrator_gain = {val}")
                else:
                    print("  Unknown command. Type 'h' for help.")
            except (ValueError, EOFError):
                pass

    try:
        ser = serial.Serial(PORT, 2000000, timeout=0.01)
        init_waveshare(ser)
        ser.reset_input_buffer()

        print("\n=== ODrive Tuning Tool - Motor Node 1 ===")
        print(f"KT: {TORQUE_CONSTANT} Nm/A")
        print("\nPrerequisites (set once via odrivetool):")
        print("  control_mode = POSITION (3)")
        print("  input_mode = PASSTHROUGH (1)")
        print("  current_lim = 5.0 (or your desired limit)")

        # Step 1: IDLE
        print(f"\nStep 1: Setting Node {NODE} to IDLE...")
        set_axis_state(ser, NODE, 1)
        time.sleep(1.0)

        # Step 2: Energize
        print("Step 2: Energizing motor...")
        set_axis_state(ser, NODE, 8)
        time.sleep(2.0)

        # Step 3: Read position
        print("Step 3: Reading position...")
        ser.reset_input_buffer()
        initial_pos = read_position_reliable(ser, num_readings=5, timeout=10.0)
        if initial_pos is None:
            print("ERROR: Could not read position. Setting to IDLE.")
            set_axis_state(ser, NODE, 1)
            ser.close()
            return

        # Step 4: Lock to that position
        print(f"Step 4: Locking to position {initial_pos:.3f} rev...")
        set_input_pos(ser, NODE, initial_pos)
        time.sleep(0.05)
        current_target[0] = initial_pos
        position[0] = initial_pos

        # Step 5: Apply gains (pos_gain and vel_gains only — no set_limits!)
        print("Step 5: Applying gains...")
        set_pos_gain(ser, NODE, gains['pos_gain'])
        time.sleep(0.05)
        set_vel_gains(ser, NODE, gains['vel_gain'], gains['vel_integrator_gain'])
        time.sleep(0.05)

        print(f"\nMotor holding at {initial_pos:.3f} rev")
        print(f"Gains: pos={gains['pos_gain']}, vel={gains['vel_gain']}, "
              f"vel_int={gains['vel_integrator_gain']}")
        print("Current limit: set via odrivetool (not changed by this script)")
        print_help()

        # Start input thread
        t = threading.Thread(target=input_thread_fn, daemon=True)
        t.start()

        last_print_time = time.time()
        last_iq_request = time.time()

        while running[0]:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))

            while len(buffer) >= 5:
                if buffer[0] == 0xAA:
                    data_len = buffer[1] & 0x0F
                    packet_len = 5 + data_len

                    if len(buffer) >= packet_len:
                        if buffer[packet_len - 1] == 0x55:
                            arb_id = buffer[2] | (buffer[3] << 8)

                            if arb_id == expected_encoder_id and data_len == 8:
                                pos, vel = struct.unpack('<ff', buffer[4:12])
                                position[0] = pos
                                velocity[0] = vel

                            elif arb_id == expected_iq_id and data_len == 8:
                                iq_sp, iq_m = struct.unpack('<ff', buffer[4:12])
                                iq_meas[0] = iq_m

                            del buffer[:packet_len]
                        else:
                            buffer.pop(0)
                    else:
                        break
                else:
                    buffer.pop(0)

            current_time = time.time()

            if current_time - last_iq_request > 0.03:
                request_iq(ser, NODE)
                last_iq_request = current_time

            if recording[0] and record_start[0] is not None:
                elapsed = current_time - record_start[0]
                if elapsed < 2.0:
                    if position[0] is not None and len(pos_history) < history_len:
                        interval = 2.0 / history_len
                        if elapsed >= len(pos_history) * interval:
                            pos_history.append(position[0])
                            torque = iq_meas[0] * TORQUE_CONSTANT if iq_meas[0] is not None else 0.0
                            torque_history.append(torque)
                else:
                    recording[0] = False
                    print_ascii_plot()

            if current_time - last_print_time > 0.1 and not recording[0]:
                pos = position[0]
                iq = iq_meas[0]
                torque = iq * TORQUE_CONSTANT if iq is not None else None

                output = "\r"
                if pos is not None:
                    output += f"Pos: {pos:>8.3f} rev  "
                else:
                    output += f"Pos: NO DATA  "

                if torque is not None:
                    output += f"Torque: {torque:>7.4f} Nm ({iq:>6.3f} A)  "
                else:
                    output += f"Torque: ---  "

                if velocity[0] is not None:
                    output += f"Vel: {velocity[0]:>7.2f}"
                output += "    "

                sys.stdout.write(output)
                sys.stdout.flush()
                last_print_time = current_time

    except KeyboardInterrupt:
        running[0] = False

    print("\n\nStopping... setting motor to IDLE.")
    if ser and ser.is_open:
        set_axis_state(ser, NODE, 1)
        time.sleep(0.1)
        print("Motor disarmed.")
        ser.close()


if __name__ == "__main__":
    main()