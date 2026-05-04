import serial
import struct
import time
import math

PORT = "/dev/ttyUSB0"
NODE_KNEE = 1
NODE_HIP = 5

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
    send_can(ser, node_id, 0x07, struct.pack('<I', state))

def set_input_pos(ser, node_id, position):
    data = struct.pack('<fhh', position, 0, 0)
    send_can(ser, node_id, 0x0C, data)

def read_position(ser, node_id, timeout=10.0):
    expected_id = (node_id << 5) | 0x09
    buf = bytearray()
    readings = []
    start = time.time()
    while time.time() - start < timeout and len(readings) < 5:
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
    if len(readings) < 5:
        return None
    return readings[-1]

ser = serial.Serial(PORT, 2000000, timeout=0.01)
init_waveshare(ser)
ser.reset_input_buffer()

# Idle both first
set_axis_state(ser, NODE_KNEE, 1)
set_axis_state(ser, NODE_HIP, 1)
time.sleep(1.0)

# Energize both
print("Energizing motors...")
set_axis_state(ser, NODE_KNEE, 8)
set_axis_state(ser, NODE_HIP, 8)
time.sleep(2.0)

# Read current positions
print("Reading positions...")
ser.reset_input_buffer()
knee_pos = read_position(ser, NODE_KNEE)
hip_pos = read_position(ser, NODE_HIP)

if knee_pos is None or hip_pos is None:
    print("ERROR: Could not read positions.")
    set_axis_state(ser, NODE_KNEE, 1)
    set_axis_state(ser, NODE_HIP, 1)
    ser.close()
    exit()

print(f"Knee (M1): {knee_pos:.3f} rev — holding")
print(f"Hip  (M5): {hip_pos:.3f} rev — oscillating")

# Lock knee in place
set_input_pos(ser, NODE_KNEE, knee_pos)

# Lock hip to start position
set_input_pos(ser, NODE_HIP, hip_pos)
time.sleep(0.5)

print("Oscillating hip... Ctrl+C to stop")

try:
    t = 0
    while True:
        offset = 0.2 * math.sin(t * 4.0)
        set_input_pos(ser, NODE_HIP, hip_pos + offset)
        time.sleep(0.02)
        t += 0.02
except KeyboardInterrupt:
    print("\nStopping...")

# Return and idle
set_input_pos(ser, NODE_HIP, hip_pos)
time.sleep(1.0)
set_axis_state(ser, NODE_KNEE, 1)
set_axis_state(ser, NODE_HIP, 1)
print("Motors idle.")
ser.close()