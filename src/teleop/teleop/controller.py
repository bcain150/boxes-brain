"""Interprets messages from the xone driver"""
from evdev import InputDevice, list_devices, ecodes
import time

VENDOR_ID = 0x045e      # microsoft vendor id
PRODUCT_ID = 0x0b12     # xbox controller product id

def find_controller() -> InputDevice:
    for path in list_devices():
        device = InputDevice(path)
        print(f"Device:\n\t{device}\nInfo:\n\t{device.info}")
        if device.info.vendor == VENDOR_ID and device.info.product == PRODUCT_ID:
            print(f"Device {device.name} was discovered!")
            return device
    
    print("No device was found!")
    return None
        
def read_controller(device: InputDevice):
    print(f"Controller ({device.name}) Connected!")
    try:
        for event in device.read_loop():
            print(event) # raw event
    except OSError as oe:
        print(f"Controller ({device.name}) Disconnected - {oe}")

def main():
    try:
        while True:
            print("Scanning for controller...")
            controller = find_controller()
            if controller:
                read_controller(controller)
            else:
                time.sleep(5)
    except KeyboardInterrupt:
        print("Quitting ...")

if __name__ == "__main__":
    main()