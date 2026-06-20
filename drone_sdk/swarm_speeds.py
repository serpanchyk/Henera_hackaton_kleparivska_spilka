"""Single source of truth for swarm motion speeds.

Historically the leader cruise speed and the follower chase speed were each
duplicated in several places: ``cv_swarm_test.py`` (``STEP_SLEEP_S``), both
``mission_launch.py`` scripts (``LEADER_CRUISE_SPEED_M_S``), and the
``FollowerControllerConfig`` default. They drifted apart — the leader crawled
at ~0.4 m/s while the followers chased at 1.5 m/s (almost 4x faster), so the
followers overran the leader, crowded him, then reversed hard. That mismatch
is the root cause of the dangerous approach/retreat oscillation.

Define the speeds *once*, here, and derive everything else from them:

- the leader flies its route at ``LEADER_CRUISE_SPEED_M_S``;
- a follower's forward/reverse caps are derived from the leader's cruise via
  fixed ratios, so they stay matched no matter how the leader speed is tuned.

To make the whole swarm faster, change ``LEADER_CRUISE_SPEED_M_S`` here and
nowhere else. CV tracking in SITL degrades at high speed, so raise it
gradually (the original route ran at ~0.4 m/s on purpose).
"""

# Leader horizontal cruise speed (m/s). Single knob for the whole swarm's pace.
LEADER_CRUISE_SPEED_M_S: float = 2.0

# Followers chase at a modest multiple of the leader so they can close a gap
# without dramatically outrunning him. Reverse is capped much lower: backing
# up fast inside a chain risks the trailing follower, so when a follower gets
# too close it eases back gently rather than slamming into reverse.
FOLLOWER_FORWARD_RATIO: float = 1.5
FOLLOWER_REVERSE_RATIO: float = 0.4

# Convenience constants matched to the default leader cruise above.
FOLLOWER_MAX_FORWARD_SPEED_M_S: float = LEADER_CRUISE_SPEED_M_S * FOLLOWER_FORWARD_RATIO
FOLLOWER_MAX_REVERSE_SPEED_M_S: float = LEADER_CRUISE_SPEED_M_S * FOLLOWER_REVERSE_RATIO


def follower_forward_for(leader_cruise_m_s: float = LEADER_CRUISE_SPEED_M_S) -> float:
    """Follower max forward speed matched to a given leader cruise speed."""
    return abs(leader_cruise_m_s) * FOLLOWER_FORWARD_RATIO


def follower_reverse_for(leader_cruise_m_s: float = LEADER_CRUISE_SPEED_M_S) -> float:
    """Follower max reverse (back-off) speed matched to a given leader cruise speed."""
    return abs(leader_cruise_m_s) * FOLLOWER_REVERSE_RATIO


def step_sleep_s(step_m: float, cruise_m_s: float = LEADER_CRUISE_SPEED_M_S) -> float:
    """Seconds to sleep per route step so a stepped ``go_to`` cruises at ``cruise_m_s``.

    The leader route advances in ``step_m`` hops with a sleep between each; the
    effective speed is ``step_m / sleep``. This inverts that so the route speed
    is expressed in m/s instead of an opaque sleep constant.
    """
    if cruise_m_s <= 0.0:
        raise ValueError('cruise_m_s must be positive')
    return abs(step_m) / cruise_m_s
