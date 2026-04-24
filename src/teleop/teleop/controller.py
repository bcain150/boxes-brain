"""Interprets messages from the xone driver"""

from evdev import InputDevice, InputEvent, list_devices, ecodes
import time
import threading
from enum import Enum
from typing import Dict, Optional, Tuple
from dataclasses import dataclass

VENDOR_ID = 0x045E  # microsoft vendor id
PRODUCT_IDS = [0x0B12, 0x02EA]  # xbox controller product ids


class Button(Enum):
    """Enum mapping of ecodes for ease of use in Controller state"""

    event_type: int
    code: int

    def __new__(cls, event_type, code):
        obj = object.__new__(cls)
        obj._value_ = (event_type, code)
        obj.event_type = event_type
        obj.code = code

    # sticks
    LEFT_STICK_Y = ecodes.EV_ABS, ecodes.ABS_Y
    LEFT_STICK_X = ecodes.EV_ABS, ecodes.ABS_X
    LEFT_STICK_PRESS = ecodes.EV_KEY, ecodes.BTN_THUMBL
    RIGHT_STICK_Y = ecodes.EV_ABS, ecodes.ABS_RY
    RIGHT_STICK_X = ecodes.EV_ABS, ecodes.ABS_RX
    RIGHT_STICK_PRESS = ecodes.EV_KEY, ecodes.BTN_THUMBR

    # X, Y, A, B buttons
    X = ecodes.EV_KEY, ecodes.BTN_X
    Y = ecodes.EV_KEY, ecodes.BTN_Y
    A = ecodes.EV_KEY, ecodes.BTN_A
    B = ecodes.EV_KEY, ecodes.BTN_B

    # D-pad
    LEFT_RIGHT = ecodes.EV_ABS, ecodes.ABS_HAT0X
    UP_DOWN = ecodes.EV_ABS, ecodes.ABS_HAT0Y

    # Triggers
    LEFT_TRIGGER = ecodes.EV_ABS, ecodes.ABS_Z
    RIGHT_TRIGGER = ecodes.EV_ABS, ecodes.ABS_RZ

    # Bumpers
    LEFT_BUMPER = ecodes.EV_KEY, ecodes.BTN_TL
    RIGHT_BUMPER = ecodes.EV_KEY, ecodes.BTN_TR

    # Other
    SELECT = ecodes.EV_KEY, ecodes.BTN_SELECT
    START = ecodes.EV_KEY, ecodes.BTN_START
    XBOX = ecodes.EV_KEY, ecodes.BTN_MODE

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
    

@dataclass
class AxisMeta:
    """Stores some Abs info from device.capabilities"""
    min: int
    max: int
    flat: int
    fuzz: int

    @property
    def center(self) -> float:
        return (self.min + self.max) / 2.0

    @property
    def half_range(self) -> float:
        return max(1.0, (self.max - self.min) / 2.0)

    @property
    def full_range(self) -> float:
        return max(1.0, (self.max - self.min))
    
    @classmethod
    def from_abs_info(cls, abs_info):
        return cls(
            min = abs_info.min,
            max = abs_info.max,
            flat = abs_info.flat or 0,
            fuzz = abs_info.fuzz or 0,
        )

@dataclass
class ButtonState:
    raw_value: int | bool
    stamp: float
    meta_info: Optional[AxisMeta] = None
    _last: int = None       # used to calculate flat and fuzz
    _value: int = None      # use this to check how flat and fuzz are computed

    @property
    def is_axis(self):
        return self.meta_info is not None
    
    @property
    def normalized(self):
        """get a normalized value of the button state. If not an axis just return raw value"""
        if not self.is_axis:
            return self.raw_value
        if self.meta_info.min >= 0 :
            return self.normalize_unsigned()
        return self.normalize_signed()
    
    def update(self, value: int|bool, stamp: float):
        self.raw_value = value
        self.stamp = stamp
    
    def _apply_fuzz(self):
        """Sets the value to last value if the difference is less than fuzz"""
        if self._last is None:
            self._last = self._value = self.raw_value
            return
        if abs(self.raw_value - self._last) <= self.meta_info.fuzz:
            self._value = self._last
            return
        self._last = self._value = self.raw_value

    def _apply_flat(self):
        """applies deadzone normalization if difference from center is less than fuzz"""
        if abs(self.raw_value - self.meta_info.center) <= self.meta_info.flat:
            self._value = self.meta_info.center
        self._value = self.raw_value

    def _normalize_signed(self) -> float:
        """Normalize a signed value from it's original range to -1.0 to 1.0"""
        self._apply_flat()
        self._apply_fuzz()
        n = (self._value - self.meta_info.center) / self.meta_info.half_range
        return max(-1.0, min(1.0, n))    # incase there is some noise
    
    def _normalize_unsigned(self) -> float:
        """Normalize an unsigned value from it's original range to 0.0 to 1.0"""
        self._apply_fuzz()
        v = max(self.meta_info.min, min(self.meta_info.max, self._value))
        n = (v - self.meta_info.min) / self.meta_info.full_range
        return max(0.0, min(1.0, n))


class ControllerState:
    """Threadsafe mapping used to update and grab the internal state of the controller
    Only for use within the xbox interface node."""

    def __init__(self, device: InputDevice, node_logger):
        self._lock = threading.RLock()
        self._state: Dict[Button, ButtonState] = {}
        self.logger = node_logger
        self._build_from_capabilities(device)

    def _build_from_capabilities(self, device: InputDevice):
        now = time.monotonic()
        caps = device.capabilities()

        if ecodes.EV_KEY in caps:
            # for every EV_KEY get it's code and initialize it
            # it's a boolean value
            for code in caps[ecodes.EV_KEY]:
                self._state[Button(ecodes.EV_KEY, code)] = ButtonState(
                    raw_value=False,
                    stamp=now
                )

        if ecodes.EV_ABS in caps:
            # initialize with value from abs
            for code, absinfo in caps[ecodes.EV_ABS]:
                self._state[Button(ecodes.EV_ABS, code)] = ButtonState(
                    raw_value=absinfo.value,
                    stamp=now,
                    meta_info=AxisMeta.from_abs_info(abs_info=absinfo)
                )

    def update(self, event: InputEvent):
        if event.type not in (ecodes.EV_KEY, ecodes.EV_ABS):
            # we want to avoid a key error here
            self.logger.warning(
                f"The event type {event.type} does not exist in the Controller State!"
            )
            return
        with self._lock:
            # update the internal button state
            self._state[Button(event.type, event.code)].update(event.value, event.timestamp())

    def _get(self, button: Button):
        with self._lock:
            return self._state[button]

    def get_raw(self, button: Button) -> bool | int:
        with self._lock:
            return self._state[button].raw_value
    
    def get_normalized(self, button: Button) -> bool | float:
        with self._lock:
            return self._state[button].normalized

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
            print(event)  # raw event
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
