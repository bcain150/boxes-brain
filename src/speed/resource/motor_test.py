#!/usr/bin/env python3
"""
ghost_motor_test.py

Bare-metal VESC UART implementation for Ghost Robot.
No pyvesc dependency — pure pyserial + struct.

Protocol ref: https://github.com/vedderb/bldc/blob/5.02/comm/commands.c
Packet format: [0x02][LEN][PAYLOAD...][CRC_HI][CRC_LO][0x03]
CRC: CRC-CCITT (poly 0x1021, init 0x0000) over payload only

Motors:
  Right: local (UART direct), ID 80
  Left:  CAN-forwarded, ID 96
"""

import serial
import struct
import time

# ── Serial config ──────────────────────────────────────────────────────────────
UART_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

# ── VESC command IDs (from datatypes.h @ fw 5.02) ─────────────────────────────
COMM_GET_VALUES   = 0x04
COMM_SET_DUTY     = 0x05
COMM_SET_CURRENT  = 0x06
COMM_SET_RPM      = 0x08
COMM_FORWARD_CAN  = 0x22  # decimal 34

LEFT_CAN_ID = 96


# ── CRC-CCITT ──────────────────────────────────────────────────────────────────
def crc16(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


# ── Packet builder ─────────────────────────────────────────────────────────────
def build_packet(payload: bytes) -> bytes:
    crc = crc16(payload)
    return bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])


def with_can_forward(can_id: int, payload: bytes) -> bytes:
    """Wrap a payload for CAN forwarding to another VESC on the bus."""
    return bytes([COMM_FORWARD_CAN, can_id]) + payload


# ── Command constructors ───────────────────────────────────────────────────────
def cmd_set_current(amps: float, can_id: int = None) -> bytes:
    # Current is int32, scaled by 1000 (milliamps)
    payload = struct.pack('>Bi', COMM_SET_CURRENT, int(amps * 1000))
    if can_id is not None:
        payload = with_can_forward(can_id, payload)
    return build_packet(payload)

def cmd_set_rpm(rpm: int, can_id: int = None) -> bytes:
    # RPM is int32, no scaling
    payload = struct.pack('>Bi', COMM_SET_RPM, rpm)
    if can_id is not None:
        payload = with_can_forward(can_id, payload)
    return build_packet(payload)

def cmd_set_duty(duty: float, can_id: int = None) -> bytes:
    # Duty is int32, scaled by 100000. Range [-1.0, 1.0]
    payload = struct.pack('>Bi', COMM_SET_DUTY, int(duty * 100000))
    if can_id is not None:
        payload = with_can_forward(can_id, payload)
    return build_packet(payload)

def cmd_get_values(can_id: int = None) -> bytes:
    payload = bytes([COMM_GET_VALUES])
    if can_id is not None:
        payload = with_can_forward(can_id, payload)
    return build_packet(payload)


# ── Response parser ────────────────────────────────────────────────────────────
# Field layout from bldc/comm/commands.c COMM_GET_VALUES case @ fw5.02:
#   float16/1e1  temp_fet
#   float16/1e1  temp_motor
#   float32/1e2  avg_motor_current
#   float32/1e2  avg_input_current
#   float32/1e2  avg_id
#   float32/1e2  avg_iq
#   float16/1e3  duty_now
#   float32/1e0  rpm          (int32 cast to float)
#   float16/1e1  v_in
#   float32/1e4  amp_hours
#   float32/1e4  amp_hours_charged
#   float32/1e4  watt_hours
#   float32/1e4  watt_hours_charged
#   int32        tachometer
#   int32        tachometer_abs
#   uint8        fault_code
# Total payload: 1 (cmd) + 2+2+4+4+4+4+2+4+2+4+4+4+4+4+4+1 = 58 bytes

def parse_get_values(data: bytes):
    """
    Parse a raw COMM_GET_VALUES response packet.
    data should be the full packet including framing bytes.
    Returns a dict or None on failure.
    """
    # Validate framing
    if len(data) < 7:
        return None
    if data[0] != 0x02 or data[-1] != 0x03:
        return None

    length = data[1]
    payload = data[2:2 + length]
    crc_recv = (data[2 + length] << 8) | data[3 + length]

    if crc16(payload) != crc_recv:
        print("  CRC mismatch!")
        return None
    if payload[0] != COMM_GET_VALUES:
        return None

    p = payload[1:]  # strip command byte
    try:
        idx = 0
        def read_i16():
            nonlocal idx
            v = struct.unpack_from('>h', p, idx)[0]
            idx += 2
            return v
        def read_i32():
            nonlocal idx
            v = struct.unpack_from('>i', p, idx)[0]
            idx += 4
            return v
        def read_u8():
            nonlocal idx
            v = p[idx]
            idx += 1
            return v

        return {
            'temp_fet':            read_i16() / 10.0,
            'temp_motor':          read_i16() / 10.0,
            'avg_motor_current':   read_i32() / 100.0,
            'avg_input_current':   read_i32() / 100.0,
            'avg_id':              read_i32() / 100.0,
            'avg_iq':              read_i32() / 100.0,
            'duty_now':            read_i16() / 1000.0,
            'rpm':                 read_i32(),
            'v_in':                read_i16() / 10.0,
            'amp_hours':           read_i32() / 10000.0,
            'amp_hours_charged':   read_i32() / 10000.0,
            'watt_hours':          read_i32() / 10000.0,
            'watt_hours_charged':  read_i32() / 10000.0,
            'tachometer':          read_i32(),
            'tachometer_abs':      read_i32(),
            'fault_code':          read_u8(),
        }
    except Exception as e:
        print(f"  Parse error: {e}")
        return None


# ── High-level helpers ─────────────────────────────────────────────────────────
def stop_all(ser):
    ser.write(cmd_set_current(0.0))
    time.sleep(0.05)
    ser.write(cmd_set_current(0.0, can_id=LEFT_CAN_ID))
    print("Motors stopped.")

def get_values(ser, can_id: int = None, timeout: float = 0.5) -> dict:
    ser.reset_input_buffer()
    ser.write(cmd_get_values(can_id=can_id))
    time.sleep(timeout)
    raw = ser.read(ser.in_waiting)
    if raw:
        return parse_get_values(raw)
    return None

def print_values(label: str, v: dict):
    if v:
        print(f"  {label}: RPM={v['rpm']:6}  VBAT={v['v_in']:.1f}V  "
              f"FET={v['temp_fet']:.1f}°C  Motor={v['temp_motor']:.1f}°C  "
              f"Fault={v['fault_code']}")
    else:
        print(f"  {label}: no response")

def telemetry_check(ser):
    print("\n--- Telemetry ---")
    print_values("Right (local) ", get_values(ser))
    print_values("Left  (CAN 96)", get_values(ser, can_id=LEFT_CAN_ID))

def ramp_test(ser):
    print("\n--- Ramp test: current control ---")
    print("Right motor ramping up...")
    for a in [0.5, 1.0, 1.5, 2.0]:
        ser.write(cmd_set_current(a))
        print(f"  Right: {a}A")
        time.sleep(0.5)

    print("Left motor ramping up (CAN 96)...")
    for a in [0.5, 1.0, 1.5, 2.0]:
        ser.write(cmd_set_current(a, can_id=LEFT_CAN_ID))
        print(f"  Left:  {a}A")
        time.sleep(0.5)

    print("Holding 2A on both for 2s...")
    time.sleep(2.0)

    print("Ramping down...")
    for a in [1.5, 1.0, 0.5, 0.0]:
        ser.write(cmd_set_current(a))
        ser.write(cmd_set_current(a, can_id=LEFT_CAN_ID))
        time.sleep(0.3)


# ── Main ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print(f"Opening {UART_PORT} @ {BAUD_RATE} baud...")
    with serial.Serial(UART_PORT, BAUD_RATE, timeout=0.5) as ser:
        time.sleep(0.1)
        try:
            telemetry_check(ser)
            input("\nPress Enter to start ramp test (motors will spin)...")
            ramp_test(ser)
        except KeyboardInterrupt:
            print("\nInterrupted.")
        finally:
            stop_all(ser)