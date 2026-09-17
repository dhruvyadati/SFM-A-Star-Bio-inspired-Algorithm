"""Shared-state integration, scoring and waypoint timing checks."""
import numpy as np
import pytest
pytest.importorskip('rvo2')
import orca
from sfm import Params, Scenario, compute_forces, run_trial


def scenario(ped=True):
    p = Params(maxSteps=1, goal=(10., 0.), dt=.1)
    return Scenario('fixture', p, [], np.zeros(2), np.zeros(2),
                    np.array([[.5], [0.]]) if ped else np.zeros((2, 0)),
                    np.array([[1.], [0.]]) if ped else np.zeros((2, 0)),
                    np.array([[20.], [0.]]) if ped else np.zeros((2, 0)),
                    np.zeros((2, 0)), np.array([[0., 10.], [0., 0.]]))


def test_same_pre_update_contact_as_sfm():
    s = scenario()
    a = orca.run_trial_orca(s, aligned=True)
    b = run_trial(s)
    assert a.ped_collisions == b.ped_collisions == 1
    assert a.min_ped_gap == pytest.approx(b.min_ped_gap)
    assert a.min_ped_gap == pytest.approx(-.15)


def test_pre_update_obstacle_gap_without_pedestrians():
    s = scenario(False)
    s.obstacles = [np.array([[.2,-1.], [1.,-1.], [1.,1.], [.2,1.]])]
    a = orca.run_trial_orca(s, aligned=True)
    b = run_trial(s)
    assert a.obs_collisions == b.obs_collisions == 1
    assert a.min_obs_gap == pytest.approx(b.min_obs_gap)


def test_shared_snapshot_and_pedestrian_integration(monkeypatch):
    s = scenario()
    s.params.maxSteps = 2
    snapshots = []
    original = orca.rvo2.PyRVOSimulator
    class Trace:
        def __init__(self, *args):
            self.sim = original(*args)
        def __getattr__(self, name):
            return getattr(self.sim, name)
        def doStep(self):
            snapshots.append((np.array(self.sim.getAgentPosition(1)),
                              np.array(self.sim.getAgentVelocity(1))))
            self.sim.doStep()
    monkeypatch.setattr(orca.rvo2, 'PyRVOSimulator', Trace)
    x = np.column_stack((s.robot_start, s.ped_pos))
    v = np.column_stack((s.robot_vel, s.ped_vel))
    goals = np.column_stack((np.zeros(2), s.ped_goal))
    f, _, _ = compute_forces(x,v,goals,np.array([.35,.30]),np.array([1.3,1.34]),[],s.params)
    expected_v = v[:,1] + f[:,1]*s.params.dt
    expected_v *= min(1., s.params.vMax/np.linalg.norm(expected_v))
    orca.run_trial_orca(s, aligned=True)
    np.testing.assert_allclose(snapshots[0][0], s.ped_pos[:,0])
    np.testing.assert_allclose(snapshots[0][1], s.ped_vel[:,0])
    np.testing.assert_allclose(snapshots[1][0], s.ped_pos[:,0]+expected_v*s.params.dt)
    np.testing.assert_allclose(snapshots[1][1], expected_v)


def test_waypoint_capture_after_first_update():
    s = scenario(False)
    r = orca.run_trial_orca(s, aligned=True, planner='hybrid')
    assert r.path_length == 0.
    s.params.maxSteps = 2
    r = orca.run_trial_orca(s, aligned=True, planner='hybrid')
    assert r.path_length > 0.


def test_aligned_rejects_other_crowd_model():
    with pytest.raises(ValueError, match='orca_robot'):
        orca.run_trial_orca(scenario(), mode='orca_crowd', aligned=True)
