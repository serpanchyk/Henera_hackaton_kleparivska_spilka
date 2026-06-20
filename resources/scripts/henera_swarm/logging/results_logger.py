#!/usr/bin/env python3
"""
Thread-safe metric logger. Writes results.json at repo root on save().
Required for hackathon submission.
"""
import json
import os
import threading
import time

RESULTS_PATH = os.path.join(
    os.path.dirname(__file__),
    '..',
    '..',
    '..',
    '..',
    'results.json',
)


class ResultsLogger:
    def __init__(self):
        self._lock = threading.Lock()
        self._entries = []

    def log(self, drone_id: int, detection: dict):
        entry = {"ts": time.time(), "drone_id": drone_id, **detection}
        with self._lock:
            self._entries.append(entry)

    def save(self):
        path = os.path.abspath(RESULTS_PATH)
        with self._lock:
            data = list(self._entries)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"Results saved → {path} ({len(data)} entries)")
