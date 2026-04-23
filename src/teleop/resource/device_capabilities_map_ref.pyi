# below is for reference only,
# its a capabilities map outputed by device.capabilities(verbose=True)
# when a controller is connected
# ruff: noqa
{
    ("EV_SYN", 0): [
        ("SYN_REPORT", 0),
        ("SYN_CONFIG", 1),
        ("SYN_DROPPED", 3),
        ("?", 21),
    ],
    ("EV_KEY", 1): [
        (["BTN_A", "BTN_GAMEPAD", "BTN_SOUTH"], 304),
        (["BTN_B", "BTN_EAST"], 305),
        (["BTN_NORTH", "BTN_X"], 307),
        (["BTN_WEST", "BTN_Y"], 308),
        ("BTN_TL", 310),
        ("BTN_TR", 311),
        ("BTN_SELECT", 314),
        ("BTN_START", 315),
        ("BTN_MODE", 316),
        ("BTN_THUMBL", 317),
        ("BTN_THUMBR", 318),
    ],
    ("EV_ABS", 3): [
        (
            ("ABS_X", 0),
            AbsInfo(value=3165, min=-32768, max=32767, fuzz=16, flat=128, resolution=0),
        ),
        (
            ("ABS_Y", 1),
            AbsInfo(value=2169, min=-32768, max=32767, fuzz=16, flat=128, resolution=0),
        ),
        (("ABS_Z", 2), AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0)),
        (
            ("ABS_RX", 3),
            AbsInfo(value=1929, min=-32768, max=32767, fuzz=16, flat=128, resolution=0),
        ),
        (
            ("ABS_RY", 4),
            AbsInfo(value=1446, min=-32768, max=32767, fuzz=16, flat=128, resolution=0),
        ),
        (
            ("ABS_RZ", 5),
            AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0),
        ),
        (
            ("ABS_HAT0X", 16),
            AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0),
        ),
        (
            ("ABS_HAT0Y", 17),
            AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0),
        ),
    ],
    ("EV_FF", 21): [
        (["FF_EFFECT_MIN", "FF_RUMBLE"], 80),
        ("FF_PERIODIC", 81),
        (["FF_SQUARE", "FF_WAVEFORM_MIN"], 88),
        ("FF_TRIANGLE", 89),
        ("FF_SINE", 90),
        (["FF_GAIN", "FF_MAX_EFFECTS"], 96),
    ],
}
