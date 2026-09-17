"""Selection boundaries and checkpoint integrity checks."""
import pytest
from run_orca_experiment import select, load_rows, write_json, key


def test_selection_preserves_smallest_nonzero_rule():
    rows = [dict(planner='hybrid', margin=0., safe=.8, contacts=90),
            dict(planner='hybrid', margin=.1, safe=.85, contacts=1)]
    assert select(rows) == 0.
    rows[1]['contacts'] = 0
    assert select(rows) == .1


def test_selection_excludes_ineligible_zero_contacts():
    rows = [dict(planner='hybrid', margin=0., safe=.79, contacts=0),
            dict(planner='hybrid', margin=.1, safe=.85, contacts=1),
            dict(planner='direct', margin=.2, safe=1., contacts=0)]
    assert select(rows) == .1


def test_checkpoint_rejects_wrong_hash_and_endpoint(tmp_path):
    (tmp_path/'trials').mkdir()
    manifest = dict(scenarios={'dev_a': 'abc'}, margins=[0.])
    row = dict(scenario='dev_a', planner='hybrid', margin=0.,
               scenario_sha256='abc', outcome='success', ped_collisions=1,
               obs_collisions=0, safe=False)
    path = tmp_path/'trials'/f'{key("dev_a", "hybrid", 0.)}.json'
    write_json(path, row)
    rows, expected = load_rows(tmp_path, manifest)
    assert len(rows) == 1 and len(expected) == 2
    row['safe'] = True
    write_json(path, row)
    with pytest.raises(ValueError, match='safe endpoint'):
        load_rows(tmp_path, manifest)
    row['safe'] = False
    row['scenario_sha256'] = 'changed'
    write_json(path, row)
    with pytest.raises(ValueError, match='hash mismatch'):
        load_rows(tmp_path, manifest)


def test_json_rejects_nonfinite(tmp_path):
    with pytest.raises(ValueError):
        write_json(tmp_path/'x.json', {'gap': float('nan')})
    assert not (tmp_path/'x.json').exists()
