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
  Usable ERPM range: 900–8700 (both motors)

Generated with claude
"""

import serial
import struct
import time

UART_PORT    = "/dev/ttyAMA0"
BAUD_RATE    = 115200
LEFT_CAN_ID  = 96

COMM_SET_RPM     = 0x08
COMM_SET_CURRENT = 0x06  # used only for stop (0A)
COMM_GET_VALUES  = 0x04
COMM_FORWARD_CAN = 0x22

ERPM_MIN  =  900
ERPM_MAX  = 8700
ERPM_STEP = 1500  # step size for ramp


# ── CRC-CCITT ──────────────────────────────────────────────────────────────────
def crc16(data: bytes) -> int:
    crc = 0x0000
    poly = 0x1021
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ poly if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


# ── Packet builder ─────────────────────────────────────────────────────────────
def build_packet(payload: bytes) -> bytes:
    crc = crc16(payload)
    return bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])

def with_can_forward(can_id: int, payload: bytes) -> bytes:
    return bytes([COMM_FORWARD_CAN, can_id]) + payload


# ── Commands ───────────────────────────────────────────────────────────────────
def cmd_set_rpm(rpm: int, can_id: int = None) -> bytes:
    payload = struct.pack('>Bi', COMM_SET_RPM, rpm)
    if can_id is not None:
        payload = with_can_forward(can_id, payload)
    return build_packet(payload)

def cmd_stop(can_id: int = None) -> bytes:
    """Zero current — cleanest way to stop."""
    payload = struct.pack('>Bi', COMM_SET_CURRENT, 0)
    if can_id is not None:
        payload = with_can_forward(can_id, payload)
    return build_packet(payload)

def cmd_get_values(can_id: int = None) -> bytes:
    payload = bytes([COMM_GET_VALUES])
    if can_id is not None:
        payload = with_can_forward(can_id, payload)
    return build_packet(payload)


# ── Response parser ────────────────────────────────────────────────────────────
def parse_get_values(data: bytes):
    if len(data) < 7 or data[0] != 0x02 or data[-1] != 0x03:
        return None
    length = data[1]
    payload = data[2:2 + length]
    crc_recv = (data[2 + length] << 8) | data[3 + length]
    if crc16(payload) != crc_recv:
        print("  CRC mismatch!")
        return None
    if payload[0] != COMM_GET_VALUES:
        return None
    p = payload[1:]
    idx = 0
    def read_i16():
        nonlocal idx; v = struct.unpack_from('>h', p, idx)[0]; idx += 2; return v
    def read_i32():
        nonlocal idx; v = struct.unpack_from('>i', p, idx)[0]; idx += 4; return v
    def read_u8():
        nonlocal idx; v = p[idx]; idx += 1; return v
    try:
        return {
            'temp_fet':          read_i16() / 10.0,
            'temp_motor':        read_i16() / 10.0,
            'avg_motor_current': read_i32() / 100.0,
            'avg_input_current': read_i32() / 100.0,
            'avg_id':            read_i32() / 100.0,
            'avg_iq':            read_i32() / 100.0,
            'duty_now':          read_i16() / 1000.0,
            'rpm':               read_i32(),
            'v_in':              read_i16() / 10.0,
            'amp_hours':         read_i32() / 10000.0,
            'amp_hours_charged': read_i32() / 10000.0,
            'watt_hours':        read_i32() / 10000.0,
            'watt_hours_charged':read_i32() / 10000.0,
            'tachometer':        read_i32(),
            'tachometer_abs':    read_i32(),
            'fault_code':        read_u8(),
        }
    except Exception as e:
        print(f"  Parse error: {e}")
        return None


# ── High-level helpers ─────────────────────────────────────────────────────────
def stop_all(ser):
    ser.write(cmd_stop())
    ser.write(cmd_stop(can_id=LEFT_CAN_ID))
    print("Motors stopped.")

def get_values(ser: serial.Serial, can_id: int = None, timeout: float = 0.5) -> dict:
    ser.reset_input_buffer()
    ser.write(cmd_get_values(can_id=can_id))
    time.sleep(timeout)
    raw = ser.read(ser.in_waiting)
    return parse_get_values(raw) if raw else None

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
    """
    Ramp both motors simultaneously in ERPM steps.
    Right and left commands are sent back-to-back with negligible delay —
    effectively simultaneous from the motors' perspective.
    """
    print("\n--- Ramp test: RPM control ---")
    ramp_up = list(range(ERPM_MIN, ERPM_MAX + 1, ERPM_STEP))

    print("Ramping up...")
    for erpm in ramp_up:
        ser.write(cmd_set_rpm(erpm))
        ser.write(cmd_set_rpm(erpm, can_id=LEFT_CAN_ID))
        print(f"  ERPM: {erpm}")
        time.sleep(1.0)

    print(f"Holding {ERPM_MAX} ERPM for 3s...")
    time.sleep(3.0)

    print("Ramping down...")
    for erpm in reversed(ramp_up):
        ser.write(cmd_set_rpm(erpm))
        ser.write(cmd_set_rpm(erpm, can_id=LEFT_CAN_ID))
        print(f"  ERPM: {erpm}")
        time.sleep(1.0)


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