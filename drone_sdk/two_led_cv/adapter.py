"""Adapter: TwoLedObservation (CV pipeline) -> VisualObservation (follower controller).

The CV pipeline reports normalized image errors (x_error/y_error in -1..1) and a
metric range, while FollowerController consumes bearing angles in degrees and a
`target_size` where bigger == closer. This module converts between the two so the
real optical channel (camera -> LED) can drive the already-tested controller.

Pinhole conversion (exact): x_error is normalized by width/2, so the bearing is
    angle = atan(x_error * tan(hfov/2))
and the width term cancels. Vertical uses the same focal length, scaled by the
frame aspect (height/width).

target_size reuses the debug provider's convention so the SAME
FollowerControllerConfig works for both the debug and CV paths:
    target_size = SIZE_GAIN / range,  SIZE_GAIN = desired_target_size * desired_distance
so at the desired distance target_size == desired_target_size (default 80 at 3 m).
"""
import math
import time

from ..follower_controller import VisualObservation


def two_led_to_visual(
    obs,
    *,
    horizontal_fov_rad: float = 1.6,
    frame_aspect: float = 0.75,          # height / width (480/640)
    size_gain: float = 240.0,            # desired_target_size(80) * desired_distance(3.0)
    desired_target_size: float = 80.0,
    min_range_m: float = 0.3,
) -> VisualObservation:
    """Convert a TwoLedObservation into a VisualObservation for FollowerController."""
    if (
        not obs.visible
        or obs.x_error is None
        or obs.y_error is None
    ):
        return VisualObservation(
            target_visible=False,
            horizontal_angle_deg=0.0,
            vertical_angle_deg=0.0,
            target_size=0.0,
            mission_state=obs.state,
            timestamp=time.monotonic(),
        )

    half = math.tan(horizontal_fov_rad / 2.0)
    horizontal_angle_deg = math.degrees(math.atan(obs.x_error * half))
    vertical_angle_deg = math.degrees(math.atan(obs.y_error * half * frame_aspect))

    range_m = obs.estimated_range_m
    if range_m is not None and range_m > 0.0:
        target_size = size_gain / max(range_m, min_range_m)
    else:
        # No metric range (e.g. anchor-only with no prior pair): keep forward neutral.
        target_size = desired_target_size

    return VisualObservation(
        target_visible=True,
        horizontal_angle_deg=horizontal_angle_deg,
        vertical_angle_deg=vertical_angle_deg,
        target_size=target_size,
        mission_state=obs.state,
        timestamp=time.monotonic(),
    )
