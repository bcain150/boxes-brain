"""Interface layer for UART communication with flipsky VESC controller.
Implementation is based off of commands accepted by VESC firmware by 
Benjamin Vedder https://github.com/vedderb/bldc"""

import serial
import struct
import time

from typing import Optional
from enum import Enum
from pathlib import Path
import functools

UARTDevice = Path | str

# min and max erpms as measured from VESC tool testing
# NOTE: these are used motors, they had different max RPMS. I chose the lesser
# of each so that we didn't get conflicting actual rpms if we max out the controls
ERPM_MIN  =  900
ERPM_MAX  = 8700

class CommandValues(Enum):
    SET_RPM = 0x08
    SET_CURRENT = 0x06  # used only for stop (0A)
    GET_VALUES  = 0x04
    FORWARD_CAN = 0x22

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


def connection_guard(method):
    @functools.wraps(method)
    def wrapper(self: VescCommandInterface, *args, **kwargs):
        if self._serial is None or not self._serial.is_open:
            raise ConnectionError(
                "No connection to serial connection to VESC!"
            )
        return method(*args, **kwargs)
    return wrapper

class VescCommandInterface:

    def __init__(self, uart_port: UARTDevice, baud_rate: int, secondary_can_id: Optional[int] = None):
        self.uart_port = Path(uart_port)
        self.baud_rate = baud_rate
        self.secondary_can_id = secondary_can_id
        self.has_can = secondary_can_id is not None

        self._serial: Optional[serial.Serial] = None

        assert self.uart_port.exists(), f"UART port {self.uart_port} does not exist!"
        assert self.uart_port.is_char_device(), f"UART port {self.uart_port} is not a char device!"

    def connect(self):
        self._serial = serial.Serial(
            port=self.uart_port,
            baudrate=self.baud_rate,
            timout=0.5
        )

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        return

    def __enter__(self):
        self.connect()
        return
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return
    
    @connection_guard
    def stop_all(self):
        pass
    
    @connection_guard
    def set_rpm(self):
        pass
    
    @connection_guard
    def get_status(self):
        pass

    # ── Packet builder ─────────────────────────────────────────────────────────────
    def _build_packet(self, payload: bytes) -> bytes:
        crc = crc16(payload)
        return bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])

    def _with_can_forward(self, payload: bytes) -> bytes:
        return bytes([CommandValues.FORWARD_CAN, self.secondary_can_id]) + payload


    




    


    