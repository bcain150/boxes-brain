"""Interprets messages from the xone driver"""
from evdev import InputDevice, InputEvent, list_devices, ecodes
import time
import threading
from enum import Enum

VENDOR_ID = 0x045e      # microsoft vendor id
PRODUCT_IDS = [0x0b12, 0x02ea]     # xbox controller product ids

class Button(Enum):
    """Enum mapping of ecodes for ease of use in Controller state"""

    # sticks
    LEFT_STICK_Y = (ecodes.EV_ABS, ecodes.ABS_Y)
    LEFT_STICK_X = (ecodes.EV_ABS, ecodes.ABS_X)
    LEFT_STICK_PRESS = (ecodes.EV_KEY, ecodes.BTN_THUMBL)
    RIGHT_STICK_Y = (ecodes.EV_ABS, ecodes.ABS_RY)
    RIGHT_STICK_X = (ecodes.EV_ABS, ecodes.ABS_RX)
    RIGHT_STICK_PRESS = (ecodes.EV_KEY, ecodes.BTN_THUMBR)

    # X, Y, A, B buttons
    X = (ecodes.EV_KEY, ecodes.BTN_X)
    Y = (ecodes.EV_KEY, ecodes.BTN_Y)
    A = (ecodes.EV_KEY, ecodes.BTN_A)
    B = (ecodes.EV_KEY, ecodes.BTN_B)

    # D-pad
    LEFT_RIGHT = (ecodes.EV_ABS, ecodes.ABS_HAT0X)
    UP_DOWN = (ecodes.EV_ABS, ecodes.ABS_HAT0Y)

    # Triggers
    LEFT_TRIGGER = (ecodes.EV_ABS, ecodes.ABS_Z)
    RIGHT_TRIGGER = (ecodes.EV_ABS, ecodes.ABS_RZ)

    # Bumpers
    LEFT_BUMPER = (ecodes.EV_KEY, ecodes.BTN_TL)
    RIGHT_BUMPER = (ecodes.EV_KEY, ecodes.BTN_TR)

    # Other
    SELECT = (ecodes.EV_KEY, ecodes.BTN_SELECT)
    START = (ecodes.EV_KEY, ecodes.BTN_START)
    XBOX = (ecodes.EV_KEY, ecodes.BTN_MODE)

    @classmethod
    def get_keys(cls):
        """Get a Set of Key based control Buttons"""
        return {
            cls.LEFT_STICK_PRESS,
            cls.RIGHT_STICK_PRESS,
            cls.X,
            cls.Y,
            cls.A,
            cls.B,
            cls.LEFT_BUMPER,
            cls.RIGHT_BUMPER,
            cls.SELECT,
            cls.START,
            cls.XBOX,
        }

    def get_abs(cls):
        """Get a Set of ABS based control Buttons"""
        return {
            cls.LEFT_STICK_Y,
            cls.LEFT_STICK_X,
            cls.RIGHT_STICK_Y,
            cls.RIGHT_STICK_X,
            cls.LEFT_RIGHT,
            cls.UP_DOWN,
        }


class ControllerState:
    """Threadsafe mapping used to update and grab the internal state of the controller
    Only for use within the xbox interface node."""
    def __init__(self, device: InputDevice, node_logger):
        self._lock = threading.RLock()
        self._state = {}
        self._build_from_capabilities(device)
        self.logger = node_logger

    def _build_from_capabilities(self, device: InputDevice):
        now = time.time()
        caps = device.capabilities()

        if ecodes.EV_KEY in caps:
            for code in caps[ecodes.EV_KEY]:
                self._state[(ecodes.EV_KEY, code)] = (False, now)

        if ecodes.EV_ABS in caps:
            for code, absinfo in caps[ecodes.EV_ABS]:
                self._state[(ecodes.EV_ABS, code)] = (absinfo.value, now)

    def update(self, event: InputEvent):
        if event.type not in (ecodes.EV_KEY, ecodes.EV_ABS):
            # we want to avoid a key error here
            self.logger.warning(f"The event type {event.type.name} does not exist in the Controller State!")
            return
        with self._lock:
            # update the internal button state
            self._state[(event.type, event.code)] = (event.value, event.timestamp)

    def _get(self, event_type, code):
        with self._lock:
            return self.state.get((event_type, code))

    def get_button(self, button: Button):
        return self._get(**button.value)

    def snapshot(self):
        """Get a full snapshot of the current state"""
        with self._lock:
            return dict(self._state)


# Dummy demo code below
def find_controller() -> InputDevice:
    for path in list_devices():
        device = InputDevice(path)
        print(f"Device:\n\t{device}\nInfo:\n\t{device.info}")
        if device.info.vendor == VENDOR_ID and device.info.product in PRODUCT_IDS:
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