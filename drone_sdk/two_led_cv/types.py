from dataclasses import dataclass


@dataclass
class LedBlob:
    cx: float
    cy: float
    area: float
    confidence: float


@dataclass
class TwoLedObservation:
    visible: bool
    x_error: float | None
    y_error: float | None
    led_distance_px: float | None
    pair_angle_rad: float | None
    anchor_visible: bool
    signal_visible: bool
    state: str
    confidence: float
    last_seen_age_s: float
