"""Deterministic presentation intervals; never modifies frozen analyses.

The same key denotes the same statistic in tables and figures. Confirmatory
unsafe/deadlock intervals are read from the archived frozen analysis output.
Other intervals are descriptive paired percentile bootstraps (4,000 draws).
"""
import hashlib
import json
import re
from pathlib import Path
import numpy as np


def stable_seed(key):
    payload = json.dumps(key, sort_keys=True, separators=(",", ":")).encode()
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")


def reporting_ci(values, key):
    v = np.asarray(values, float)
    if key in ("x1", "x2"):
        label = "unsafe arrival, hyb->hyb-ttc" if key == "x1" else "deadlock, hyb->hyb-ttc"
        src = Path(__file__).resolve().parents[1] / "v2/runs/v2/confirm_analysis_output.txt"
        line = next(line for line in src.read_text().splitlines() if label in line)
        m = re.search(r"([+-]\d+\.\d+)\s+\[([+-]\d+\.\d+),([+-]\d+\.\d+)\]", line)
        mean, lo, hi = map(float, m.groups())
        if not np.isclose(v.mean(), mean):
            raise ValueError("Archived interval does not match plotted statistic")
        return mean, lo, hi
    rng = np.random.default_rng(stable_seed(key))
    boot = v[rng.integers(0, len(v), size=(4000, len(v)))].mean(axis=1)
    return float(v.mean()), *map(float, np.percentile(boot, [2.5, 97.5]))
