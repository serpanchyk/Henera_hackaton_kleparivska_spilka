#!/usr/bin/env python3
"""Thin entrypoint for the production follower runtime."""

import os
import sys

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from henera_swarm.orchestration.follower_runtime import cli


if __name__ == '__main__':
    cli()
