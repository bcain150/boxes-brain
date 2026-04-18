"""Interface layer for UART communication with flipsky VESC controller.
Implementation is based off of commands accepted by VESC firmware by 
Benjamin Vedder https://github.com/vedderb/bldc"""

def crc16(data: bytes) -> int:
    crc = 0x0000          # starting value
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021   # the CCITT polynomial
            else:
                crc <<= 1
            crc &= 0xFFFF   # keep it 16-bit
    return crc

def build_packet(payload: bytes) -> bytes:
    crc = crc16(payload)
    return bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])
