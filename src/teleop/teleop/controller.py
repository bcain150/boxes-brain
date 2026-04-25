"""Interprets messages from the xone driver"""

from evdev import InputDevice, InputEvent, list_devices, ecodes
import time
import threading
from enum import Enum
from typing import Dict, Optional
from dataclasses import dataclass

from boxes_utils import format_error_message

VENDOR_ID = 0x045E  # microsoft vendor id
PRODUCT_IDS = [0x0B12, 0x02EA]  # xbox controller product ids

# override value for flat abs_info, kernel doesn't provide good enough values
STICK_DEADZONE = 2000

class Button(Enum):
    """Enum mapping of ecodes for ease of use in Controller state"""

    event_type: int
    code: int

    def __new__(cls, event_type, code):
        obj = object.__new__(cls)
        obj._value_ = (event_type, code)
        obj.event_type = event_type
        obj.code = code
        return obj

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

    @classmethod
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

    @classmethod
    def stick_inputs(cls):
        return {
            cls.LEFT_STICK_Y,
            cls.LEFT_STICK_X,
            cls.RIGHT_STICK_Y,
            cls.RIGHT_STICK_X,
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
    from_button: Button
    meta_info: Optional[AxisMeta] = None
    _last: Optional[int] = None       # used to calculate flat and fuzz
    _value: Optional[int] = None      # use this to check how flat and fuzz are computed

    def __post_init__(self):
        if self.from_button in Button.stick_inputs():
            # override flat:
            self.meta_info.flat = STICK_DEADZONE

    @property
    def is_axis(self):
        return self.meta_info is not None
    
    @property
    def normalized(self):
        """get a normalized value of the button state. If not an axis just return raw value"""
        if not self.is_axis:
            return self.raw_value
        if self.meta_info.min >= 0 :
            return self._normalize_unsigned()
        return self._normalize_signed()
    
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
        else:
            self._value = self.raw_value

    def _normalize_signed(self) -> float:
        """Normalize a signed value from it's original range to -1.0 to 1.0"""
        if self.from_button == Button.UP_DOWN:
            return float(self.raw_value*-1) # flip this because it's reversed some reason
        elif self.from_button == Button.LEFT_RIGHT:
            return float(self.raw_value)
        
        self._apply_fuzz()
        self._apply_flat()
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
        verbose_caps = device.capabilities(verbose=True)

        if ecodes.EV_KEY in caps:
            # for every EV_KEY get it's code and initialize it
            # it's a boolean value
            for i, code in enumerate(caps[ecodes.EV_KEY]):
                try:
                    button = Button(ecodes.EV_KEY, code)
                    self._state[button] = ButtonState(
                        raw_value=False,
                        stamp=now,
                        from_button=button
                    )
                except ValueError:
                    names = verbose_caps[('EV_KEY', ecodes.EV_KEY)][i][0]
                    self.logger.warning(f"Skipping unmapped EV_KEY - {names}")

        if ecodes.EV_ABS in caps:
            # initialize with value from abs
            for i, (code, absinfo) in enumerate(caps[ecodes.EV_ABS]):
                try:
                    button = Button(ecodes.EV_ABS, code)
                    self._state[button] = ButtonState(
                        raw_value=absinfo.value,
                        stamp=now,
                        meta_info=AxisMeta.from_abs_info(abs_info=absinfo),
                        from_button=button
                    )
                except ValueError:
                    names = verbose_caps[('EV_ABS', ecodes.EV_ABS)][i][0]
                    self.logger.warning(f"Skipping unmapped EV_ABS - {names}")

    def update(self, event: InputEvent):
        with self._lock:
            # update the internal button state
            try:
                button = Button(event.type, event.code)
            except ValueError as e:
                self.logger.warning(f"Unrecognized Button Press. Ignoring -> {format_error_message(e)}")
                return

            self._state[button].update(event.value, event.timestamp())
            norm = self._state[button].normalized
            self.logger.info(f"Button Event for {button.name}: {norm}")

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
        

# pythonic test code generated by claude below


class PrintLogger:
    """Stand-in for a ROS node logger when running this file standalone."""
    def warning(self, msg): print(f"[WARN] {msg}")
    def info(self, msg): print(f"[INFO] {msg}")


def find_controller() -> Optional[InputDevice]:
    for path in list_devices():
        device = InputDevice(path)
        print(f"Device:\n\t{device}\nInfo:\n\t{device.info}")
        if device.info.vendor == VENDOR_ID and device.info.product in PRODUCT_IDS:
            print(f"Device {device.name} was discovered!")
            return device

    print("No device was found!")
    return None


def read_controller(device: InputDevice, logger):
    print(f"Controller ({device.name}) Connected!")
    state = ControllerState(device, logger)

    # print a snapshot every N events so the terminal isn't a firehose
    # print_every = 20
    # counter = 0

    try:
        for event in device.read_loop():
            if event.type == ecodes.EV_SYN:
                print("------SYN RECEIVED------")
                continue  # SYN_REPORT etc — not meaningful state

            state.update(event)
            _dump_interesting(state)

    except OSError as oe:
        print(f"Controller ({device.name}) Disconnected - {oe}")


def _dump_interesting(state: ControllerState):
    """Print just the buttons you'd actually look at while driving."""
    watch = [
        Button.LEFT_STICK_X, Button.LEFT_STICK_Y,
        Button.RIGHT_STICK_X, Button.RIGHT_STICK_Y,
        Button.LEFT_TRIGGER, Button.RIGHT_TRIGGER,
        Button.A, Button.B, Button.X, Button.Y,
        Button.LEFT_RIGHT, Button.UP_DOWN,
    ]
    parts = []
    for btn in watch:
        val = state.get_normalized(btn)
        if isinstance(val, float):
            parts.append(f"{btn.name}={val:.4f}")
        else:
            parts.append(f"{btn.name}={int(bool(val))}")
    print(" | ".join(parts))


def main():
    logger = PrintLogger()
    try:
        while True:
            print("Scanning for controller...")
            controller = find_controller()
            if controller:
                read_controller(controller, logger)
            else:
                time.sleep(5)
    except KeyboardInterrupt:
        print("Quitting ...")


if __name__ == "__main__":
    main()