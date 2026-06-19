# Masks stay 4 chars (4 positions), but the model has only 2 lenses and the
# plugin reads the first 2 mask bits: position 1 -> led_lens_01 (green anchor),
# position 2 -> led_lens_04 (red signal). Positions 3/4 are unused.
LED_COUNT = 4
ANCHOR_LED_INDEX = 1
SIGNAL_LED_INDEX = 2

FOLLOW_MASK = '1100'
SAFE_MASK = '1000'

FOLLOW = 'FOLLOW'
HOLD = 'HOLD'
FINISH = 'FINISH'
SAFE = 'SAFE'
UNKNOWN = 'UNKNOWN'


def make_led_mask(anchor_on: bool, signal_on: bool) -> str:
    mask = ['0'] * LED_COUNT
    mask[ANCHOR_LED_INDEX - 1] = '1' if anchor_on else '0'
    mask[SIGNAL_LED_INDEX - 1] = '1' if signal_on else '0'
    return ''.join(mask)


def signal_on_for_state(state: str, t: float) -> bool:
    normalized = state.upper() if isinstance(state, str) else UNKNOWN
    if normalized == FOLLOW:
        return True
    if normalized == HOLD:
        return _toggle(t, interval_s=0.5)
    if normalized == FINISH:
        return _toggle(t, interval_s=0.2)
    return False


def _toggle(t: float, interval_s: float) -> bool:
    return int(max(0.0, t) / interval_s) % 2 == 0
