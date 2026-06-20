"""Adapter: TwoLedObservation (CV pipeline) -> VisualObservation (follower controller)."""

import math

from ..follower_controller import VisualObservation


def two_led_to_visual(
    obs,
    *,
    timestamp: float,
    horizontal_fov_rad: float = 1.6,
    frame_aspect: float = 0.75,
    size_gain: float = 240.0,
    desired_target_size: float = 80.0,
    min_range_m: float = 0.3,
) -> VisualObservation:
    """Convert one decoded camera sample using its original frame timestamp."""
    if not math.isfinite(timestamp):
        raise ValueError('timestamp must be finite')

    if not obs.visible or obs.x_error is None or obs.y_error is None:
        return VisualObservation(
            target_visible=False,
            horizontal_angle_deg=0.0,
            vertical_angle_deg=0.0,
            target_size=0.0,
            mission_state=obs.state,
            timestamp=timestamp,
        )

    half = math.tan(horizontal_fov_rad / 2.0)
    horizontal_angle_deg = math.degrees(math.atan(obs.x_error * half))
    vertical_angle_deg = math.degrees(math.atan(obs.y_error * half * frame_aspect))

    range_m = obs.estimated_range_m
    if range_m is not None and range_m > 0.0:
        target_size = size_gain / max(range_m, min_range_m)
    else:
        target_size = desired_target_size

    return VisualObservation(
        target_visible=True,
        horizontal_angle_deg=horizontal_angle_deg,
        vertical_angle_deg=vertical_angle_deg,
        target_size=target_size,
        mission_state=obs.state,
        timestamp=timestamp,
    )
