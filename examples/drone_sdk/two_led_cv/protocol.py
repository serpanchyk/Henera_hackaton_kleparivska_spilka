# Masks stay 4 chars for Gazebo/plugin compatibility. The model has only two
# lenses: position 1 -> led_lens_01 (green), position 2 -> led_lens_04 (red).
# Positions 3/4 are unused padding.
LED_COUNT = 4
GREEN_LED_INDEX = 1
RED_LED_INDEX = 2
ANCHOR_LED_INDEX = GREEN_LED_INDEX
SIGNAL_LED_INDEX = RED_LED_INDEX

FOLLOW_MASK = '1100'
HOLD_MASK = '1000'
SAFE_MASK = '0100'
FINISH_ON_MASK = FOLLOW_MASK
FINISH_OFF_MASK = '0000'

FOLLOW = 'FOLLOW'
HOLD = 'HOLD'
FINISH = 'FINISH'
SAFE = 'SAFE'
UNKNOWN = 'UNKNOWN'


def make_led_mask(green_on: bool, red_on: bool) -> str:
    mask = ['0'] * LED_COUNT
    mask[GREEN_LED_INDEX - 1] = '1' if green_on else '0'
    mask[RED_LED_INDEX - 1] = '1' if red_on else '0'
    return ''.join(mask)


def led_states_for_state(state: str, t: float) -> tuple[bool, bool]:
    normalized = state.upper() if isinstance(state, str) else UNKNOWN
    if normalized == FOLLOW:
        return True, True
    if normalized == HOLD:
        return True, False
    if normalized == SAFE:
        return False, True
    if normalized == FINISH:
        both_on = _toggle(t, interval_s=0.5)
        return both_on, both_on
    return False, False


def mask_for_state(state: str, t: float) -> str:
    green_on, red_on = led_states_for_state(state, t)
    return make_led_mask(green_on, red_on)


def signal_on_for_state(state: str, t: float) -> bool:
    return led_states_for_state(state, t)[1]


def _toggle(t: float, interval_s: float) -> bool:
    return int(max(0.0, t) / interval_s) % 2 == 0
