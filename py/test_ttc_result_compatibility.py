"""The development TTC head must be behaviour-identical to the confirmatory one.

Run: python3 -m pytest py/test_ttc_result_compatibility.py -q

`py/ttc.py` gained `ttc_brake_ex`, which exposes the mandatory-intervention
flags the Amendment F recovery controller needs. The 3-tuple `ttc_brake` is now
a thin view over it. That is claimed to be behaviour-preserving; this test is
what makes the claim checkable, and it compares against the ARCHIVED source
that actually produced the confirmatory results, not against a description of
it.

The randomised state generator and its seed live here, so "N states passed" is
reproducible rather than an assertion the reader has to trust.
"""
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from sfm_explore import Params

N_STATES = 4000
SEED = 20260803
ARCHIVED = HERE.parent / "archive" / "confirm_code" / "py" / "ttc.py"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def states(n=N_STATES, seed=SEED):
    """Deterministic random states: varying crowd size, geometry and history."""
    rng = np.random.default_rng(seed)
    P = Params()
    for _ in range(n):
        k = int(rng.integers(0, 6))            # includes the empty-crowd path
        yield (rng.uniform(0, 50, 2), rng.uniform(-2, 2, 2),
               rng.uniform(0, 50, (2, k)), rng.uniform(-2, 2, (2, k)),
               np.full(k, P.rPed), P, float(rng.uniform(0, 1)))


@pytest.mark.skipif(not ARCHIVED.exists(), reason="archived source absent")
def test_ttc_brake_identical_to_archived_confirmatory_source():
    old = _load("ttc_archived", ARCHIVED)
    import ttc as new
    bad = []
    for i, s in enumerate(states()):
        a = old.ttc_brake(*s)
        b = new.ttc_brake(*s)
        same = (a[0] == b[0] and a[1] == b[1]
                and (a[2] == b[2] or (np.isinf(a[2]) and np.isinf(b[2]))))
        if not same:
            bad.append((i, a, b))
    assert not bad, f"{len(bad)} of {N_STATES} states differ; first: {bad[:1]}"


def test_archived_source_hash_matches_frozen_protocol():
    import hashlib
    h = hashlib.sha256(ARCHIVED.read_bytes()).hexdigest()[:16]
    assert h == "b89dcad9104b6db9", f"archived confirmatory ttc.py is {h}"


def test_empty_crowd_path_agrees():
    old = _load("ttc_archived2", ARCHIVED)
    import ttc as new
    P = Params()
    args = (np.zeros(2), np.zeros(2), np.zeros((2, 0)), np.zeros((2, 0)),
            np.zeros(0), P, 1.0)
    assert old.ttc_brake(*args) == new.ttc_brake(*args)
