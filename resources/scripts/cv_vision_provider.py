#!/usr/bin/env python3
"""Compatibility alias for the moved camera observation provider."""

import sys

from henera_swarm.perception import cv_vision_provider as _provider

CVVisionProvider = _provider.CVVisionProvider
HFOV_RAD = _provider.HFOV_RAD
LED_BASELINE_M = _provider.LED_BASELINE_M
GREEN_RANGES = _provider.GREEN_RANGES
RED_RANGES = _provider.RED_RANGES
_detect_color = _provider._detect_color

sys.modules[__name__] = _provider
