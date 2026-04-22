"""Interface layer for UART communication with flipsky VESC controller.
Implementation is based off of commands accepted by VESC firmware written by 
Benjamin Vedder https://github.com/vedderb/bldc"""

import serial
import struct
import functools
import time
from typing import Optional, Callable, Set
from enum import IntEnum, IntFlag
from pathlib import Path
from dataclasses import dataclass

UARTDevice = Path | str
ConverterFunc = Callable[['FieldMask', Optional[int]], int|float]

# min and max erpms as measured from VESC tool testing
# NOTE: these are used motors, they had different max RPMS. I chose the lesser
# of each so that we didn't get conflicting actual rpms if we max out the controls
ERPM_MIN  =  900
ERPM_MAX  = 8700

BYTE_FORMAT = '>Bi'

class CommandByte(IntEnum):
    """Commands pulled from dataclasses.h from vedderb/bldc"""
    # https://github.com/vedderb/bldc/blob/a16ffc6d9cf469884703a84ba2ee87482b71fc4c/datatypes.h#L944

    SET_RPM = 0x08
    SET_CURRENT = 0x06  # used only for stop (0A)
    GET_VALUES  = 0x04  # unused currently
    GET_VALUES_SELECTIVE = 0x32
    FORWARD_CAN = 0x22
    # TODO: figure out if we can set negative rpms

@dataclass
class MotorStatusMsg:
    """All the possible values returned from a GetValues command message (matches FieldMask exactly)"""
    temp_fet: Optional[float] = None
    temp_motor: Optional[float] = None
    avg_motor_current: Optional[float] = None
    avg_input_current: Optional[float] = None
    avg_id: Optional[float] = None
    avg_iq: Optional[float] = None
    duty_now: Optional[float] = None
    rpm: Optional[int] = None
    v_in: Optional[float] = None
    amp_hours: Optional[float] = None
    amp_hours_charged: Optional[float] = None
    watt_hours: Optional[float] = None
    watt_hours_charged: Optional[float] = None
    tachometer: Optional[int] = None
    tachometer_abs: Optional[int] = None
    fault_code: Optional[int] = None

##### STATUS RESPONSE CONVERSION ####
def read_int16(payload: memoryview) -> tuple[int, int]:
    v = struct.unpack_from('>h', payload, 0)[0]
    return v, 2

def read_int32(payload: memoryview) -> tuple[int, int]:
    v = struct.unpack_from('>i', payload, 0)[0]
    return v, 4

def read_unsigned8(payload: memoryview) -> tuple[int, int]:
    v = payload[0]
    return v, 1

class StatusMsgConversionError(Exception):
    ...

class FieldMask(IntFlag):
    """Representation of possible status fields and their masks. This class also doubles
    with the ability to parse and emit a MotorStatusMsg given a payload."""

    converter: ConverterFunc
    scaling: Optional[int|float]

    def __new__(cls, flag: int, converter: ConverterFunc, scaling: Optional[int|float]=None):
        obj = int.__new__(cls, flag)
        obj._value_ = flag
        obj.converter = converter
        obj.scaling = scaling
        return obj

    TEMP_FET           = (1 << 0), read_int16, 10.0
    TEMP_MOTOR         = (1 << 1), read_int16, 10.0
    AVG_MOTOR_CURRENT  = (1 << 2), read_int32, 100.0
    AVG_INPUT_CURRENT  = (1 << 3), read_int32, 100.0
    AVG_ID             = (1 << 4), read_int32, 100.0
    AVG_IQ             = (1 << 5), read_int32, 100.0
    DUTY_NOW           = (1 << 6), read_int16, 1000.0
    RPM                = (1 << 7), read_int32
    V_IN               = (1 << 8), read_int16, 10.0
    AMP_HOURS          = (1 << 9), read_int32, 10000.0
    AMP_HOURS_CHARGED  = (1 << 10), read_int32, 10000.0
    WATT_HOURS         = (1 << 11), read_int32, 10000.0
    WATT_HOURS_CHARGED = (1 << 12), read_int32, 10000.0
    TACHOMETER         = (1 << 13), read_int32
    TACHOMETER_ABS     = (1 << 14), read_int32
    FAULT_CODE         = (1 << 15), read_unsigned8

    @classmethod
    def from_field(cls, field_name: str) -> 'FieldMask':
        """Get an instance of a FieldMask give a MotorStatusMsg field name"""
        upper = field_name.upper()
        try:
            return cls[upper]
        except KeyError:
            raise ValueError(f"No mask found for field {field_name}!")
    
    @classmethod
    def from_fields(cls, field_names: Set[str]) -> 'FieldMask':
        """Get a mask for a set of fields, this mask cannot be used to convert
        given bitwise or over an IntFlag"""
        result = cls(0)
        for name in field_names:
            result |= cls.from_field(name)
        return result
    
    @classmethod
    def parse_response(cls, payload:bytes, fields: Set[str]) -> MotorStatusMsg:
        """parse the response from a get status command. Convert each field using converter and potentially scale.
        Finally, apply the converted value to an instance of MotorStatusMsg and return it."""
        # convert to a memory view so they can be consumed
        # also get a list of FieldMasks which have converters and scaling
        view = memoryview(payload)
        masks = [cls.from_field(field) for field in fields]
        status_msg = MotorStatusMsg()
        # iterate over each member and pass the raw bytes (view) to the converter
        for member in masks:
            try:
                converted, consumed = member.converter(view)
            except Exception as e:
                raise StatusMsgConversionError(f"Conversion for field {member.name.lower()} failed!") from e
            if member.scaling:  # scale if necessary
                converted = converted / member.scaling
            view = view[consumed:]  # update view to consume converted bytes

            # set the field attribute of the MotorStatusMsg
            setattr(status_msg, member.name.lower(), converted)
        
        # ensure we converted everything properly
        if len(view) != 0:
            raise StatusMsgConversionError(f"Some payload left over after conversion! -> {view.hex()}")

        return status_msg


def connection_guard(method):
    """decorator for ensuring we're connected to the serial device"""
    @functools.wraps(method)
    def wrapper(self: VescCommandInterface, *args, **kwargs):
        if self._serial is None or not self._serial.is_open:
            raise ConnectionError(
                "No connection to serial connection to VESC!"
            )
        return method(self, *args, **kwargs)
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
            timeout=0.5
        )

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        return

    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return
    
    @connection_guard
    def stop_single(self, to_can: bool=False):
        """Stop a single motor. Optionally send this command via CAN forwarding to a separate motor."""
        payload = struct.pack(BYTE_FORMAT, CommandByte.SET_CURRENT, 0)
        if to_can:
            assert self.has_can, "Can forwarding requested but no can id exists!"
            payload = self._with_can_forward(payload=payload)
        self._build_and_send_packet(payload=payload)

    @connection_guard
    def stop_all(self):
        """Stop both motors immediately. Includes forwarding over can bus"""
        payload = struct.pack(BYTE_FORMAT, CommandByte.SET_CURRENT, 0)
        if self.has_can:
            can_payload = self._with_can_forward(payload=payload)
            self._build_and_send_packet(payload=can_payload)
        self._build_and_send_packet(payload=payload)


    @connection_guard
    def set_rpm(self, rpm: int, to_can: bool=False):
        """Set the RPM of a motor. Optionally send this command via CAN forwarding to a separate motor."""
        # TODO: figure out if we can set negative rpms
        payload = struct.pack(BYTE_FORMAT, CommandByte.SET_RPM, rpm)
        if to_can:
            assert self.has_can, "Can forwarding requested but no can id exists!"
            payload = self._with_can_forward(payload=payload)
        self._build_and_send_packet(payload=payload)
    
    @connection_guard
    def get_status(self, *fields, to_can: bool=False) -> MotorStatusMsg:
        """Get requested fields from a motor"""
        # get the FieldMask from the set of fields and construct the payload for the command
        fields = set(fields)
        field_mask = FieldMask.from_fields(fields)
        payload = struct.pack(BYTE_FORMAT, CommandByte.GET_VALUES_SELECTIVE, field_mask)
        # send with can forwarding if the secondary motor
        if to_can:
            assert self.has_can, "Can forwarding requested but no can id exists!"
            payload = self._with_can_forward(payload=payload)
        # send the packet and then get the response payload
        self._build_and_send_packet(payload=payload)
        response_bytes = self._get_response_payload(from_command=CommandByte.GET_VALUES_SELECTIVE)
        # parse the payload and get the MotorStatusMsg
        return FieldMask.parse_response(response_bytes, fields)

    # TODO: implement the above, also think about keep alive packet sending. Connection guard doesn't help
    # against vesc side connection, only os/process side. Think about if get status acts as keep alive,
    # claude says it doesn't count but something worth checking

    # ── Packet helpers ─────────────────────────────────────────────────────────────
    def _crc16(self, data):
        """build the CRC-CCITT error correcting code"""
        crc = 0x0000
        poly = 0x1021
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = (crc << 1) ^ poly if crc & 0x8000 else crc << 1
                crc &= 0xFFFF
        return crc

    def _build_and_send_packet(self, payload: bytes) -> bytes:
        """build the complete packet with CRC-CCITT error correcting code.
        Then send over the serial connection. (Assumes connected)"""
        crc = self._crc16(payload)
        packet = bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])
        self._serial.write(packet)

    def _with_can_forward(self, payload: bytes) -> bytes:
        """Append can forwarding with can id"""
        return bytes([CommandByte.FORWARD_CAN, self.secondary_can_id]) + payload
    
    def _get_response_payload(self, from_command: CommandByte, timeout_s=1) -> bytes:
        """Get the response payload only verify the following:
            - frame validation (start and terminal bytes match)
            - command byte matches the passed in command byte
            - crc validation check (recompute)
        """
        raise_timeout = True
        reason = ""
        deadtime = time.monotonic() + timeout_s
        while time.monotonic() < deadtime:
            # get the start byte
            start = self._serial.read(1)
            if not start:
                reason = "Serial port timed out reading start byte"
                continue
            elif start[0] != 0x02:
                reason = "Malformed start byte"
                continue
            
            # get the length byte
            length = self._serial.read(1)
            if not length:
                reason = "Serial port timed out reading length byte"
                continue
            
            # get the payload, crc, and end byte
            n = length[0]
            payload = self._serial.read(n)
            crc = self._serial.read(2)
            end = self._serial.read(1)
            
            if not end or end[0] != 0x03:
                raise ValueError("Malformed packet: bad end byte")
            # don't raise if we get here, then break out of loop
            raise_timeout = False
            break
            
        if raise_timeout:
            raise ValueError(f"Failed to get response: {reason}")
        
        # verify crc
        crc_recv = (crc[0] << 8) | crc[1]
        crc_comp = self._crc16(payload)
        assert crc_comp == crc_recv, f"CRC Mismatch! Computed - {crc_comp} != Received {crc_recv}"
        
        # verify command match
        assert payload[0] == from_command, f"Payload is not from command {from_command.name}! Rather {CommandByte(payload[0])}"
        return payload[1:] # we only want to return the values, not the command name


    




    


    