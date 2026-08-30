"""Tests for PathProgress. Run: python3 -m pytest py/test_path_progress.py -q

The backward-across-a-waypoint case exists because an external review found the
original implementation wrong there. The first version used a strictly monotone
segment index: it blocked self-intersection jumps but silently SATURATED
`remaining` when the robot moved back past a waypoint, discarding exactly the
signal the recovery trigger needs. The earlier test suite missed it by only
testing backward motion WITHIN one segment.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
from path_progress import PathProgress

STRAIGHT = np.array([[0.0, 1.0, 2.0], [0.0, 0.0, 0.0]])
LSHAPE = np.array([[0.0, 10.0, 10.0], [0.0, 0.0, 10.0]])


def walk(pp, xs):
    return [pp.remaining(np.array([x, 0.0])) for x in xs]


def test_total_arc_length():
    assert PathProgress(LSHAPE).total == pytest.approx(20.0)


def test_arc_length_at_start_corner_end():
    pp = PathProgress(LSHAPE)
    assert pp.remaining(np.array([0.0, 0.0])) == pytest.approx(20.0)
    assert pp.remaining(np.array([10.0, 0.0])) == pytest.approx(10.0)
    assert pp.remaining(np.array([10.0, 10.0])) == pytest.approx(0.0, abs=1e-9)


def test_continuous_across_waypoint():
    pp = PathProgress(LSHAPE)
    a = pp.remaining(np.array([9.99, 0.0]))
    b = pp.remaining(np.array([10.0, 0.0]))
    c = pp.remaining(np.array([10.0, 0.01]))
    assert abs(a - b) < 0.02 and abs(b - c) < 0.02


def test_backward_within_a_segment_increases_remaining():
    pp = PathProgress(LSHAPE)
    r1 = pp.remaining(np.array([6.0, 0.0]))
    r2 = pp.remaining(np.array([5.0, 0.0]))
    assert r2 > r1
    assert (r1 - r2) < 0                      # progress negative, not clamped


def test_backward_ACROSS_a_passed_waypoint():
    """The regression the review caught.

    Advance past waypoint 1, then walk back to 0.2. Remaining must rise by the
    full backward displacement, not saturate at the waypoint.
    """
    pp = PathProgress(STRAIGHT)
    got = walk(pp, [0.9, 1.1, 0.8, 0.2])
    assert got == pytest.approx([1.1, 0.9, 1.2, 1.8], abs=1e-6)


def test_backward_across_waypoint_full_displacement():
    pp = PathProgress(STRAIGHT)
    pp.remaining(np.array([0.9, 0.0]))
    fwd = pp.remaining(np.array([1.1, 0.0]))
    back = pp.remaining(np.array([0.6, 0.0]))
    # Moved back 0.5 m; remaining must rise by 0.5 m.
    assert (back - fwd) == pytest.approx(0.5, abs=1e-6)


def test_no_capture_by_self_near_segment():
    """A route doubling back beside its own start must not jump."""
    W = np.array([[0.0, 10.0, 10.0, 0.2], [0.0, 0.0, 0.4, 0.4]])
    pp = PathProgress(W)
    pp.remaining(np.array([5.0, 0.0]))
    before = pp.last_travelled
    # Spatially close to BOTH segment 0 and the returning segment 2, but only a
    # tiny step: the plausibility bound must reject the far candidate.
    pp.remaining(np.array([5.0, 0.02]))
    assert abs(pp.last_travelled - before) < 0.2


def test_large_arc_jump_rejected_for_small_displacement():
    W = np.array([[0.0, 10.0, 10.0, 0.0], [0.0, 0.0, 0.5, 0.5]])
    pp = PathProgress(W)
    pp.remaining(np.array([2.0, 0.0]))
    before = pp.last_travelled
    pp.remaining(np.array([2.0, 0.01]))       # 1 cm move
    assert abs(pp.last_travelled - before) <= 0.01 + pp.jump_tol + 1e-9


def test_monotone_forward_walk_is_strictly_decreasing():
    pp = PathProgress(LSHAPE)
    rs = walk(pp, list(np.arange(0.0, 10.0, 0.25)))
    assert all(b < a for a, b in zip(rs, rs[1:]))


def test_arc_length_beats_straight_line_around_a_corner():
    """The reason arc length is used at all.

    Walked at simulation-like increments, which is how the class is used.
    """
    pp = PathProgress(LSHAPE)
    s_a = pp.remaining(np.array([9.0, 0.0]))
    for y in np.arange(0.0, 10.01, 0.05):     # along x=10 after the corner
        x = 10.0 if y > 0 else 9.5
        s_b = pp.remaining(np.array([x, y]))
        if y >= 1.0:
            break
    assert (s_a - s_b) == pytest.approx(2.0, abs=0.1)
    # Straight-line distance to the END moves far less over the same motion,
    # which is why a straight-line detector misreads cornering as a stall.
    d_a = float(np.hypot(10 - 9.0, 10 - 0.0))
    d_b = float(np.hypot(10 - 10.0, 10 - 1.0))
    assert (s_a - s_b) > (d_a - d_b)


def test_assumes_simulation_rate_sampling():
    """DOCUMENTED LIMITATION, not a bug.

    The plausibility bound is displacement-based, so it assumes `remaining` is
    called every simulation step (~0.013 m at dt=0.01). Given one enormous
    jump it may prefer a nearer-in-arc projection over the geometrically
    closest, because it cannot tell a real leap from a projection jump. The
    integration calls it per step, where the bound is tight.
    """
    pp = PathProgress(LSHAPE)
    pp.remaining(np.array([9.0, 0.0]))
    coarse = pp.remaining(np.array([10.0, 1.0]))      # 1.41 m in one call
    pp2 = PathProgress(LSHAPE)
    pp2.remaining(np.array([9.0, 0.0]))
    for y in np.arange(0.05, 1.001, 0.05):
        fine = pp2.remaining(np.array([10.0, y]))
    assert fine < coarse                               # fine-grained is correct
    assert fine == pytest.approx(9.0, abs=0.1)


def test_rejects_degenerate_waypoints():
    with pytest.raises(ValueError):
        PathProgress(np.array([[0.0], [0.0]]))
