"""Validate paired ORCA results and produce descriptive comparisons."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[1]
sys.path.insert(0, str(ROOT/'py'))
from run_orca_experiment import digest, load_rows, sources, summarize, select, PRIOR_INSPECTION
OUT = BASE/'aligned'


def interval(values, groups, tag):
    values = np.asarray(values, float)
    seed = int.from_bytes(hashlib.sha256(tag.encode()).digest()[:8], 'big')
    rng = np.random.default_rng(seed)
    boot = np.zeros(10000)
    for condition in sorted(set(groups)):
        v = values[np.asarray(groups) == condition]
        boot += v[rng.integers(0,len(v),(10000,len(v)))].mean(axis=1) * len(v)/len(values)
    return [float(values.mean()), *map(float,np.percentile(boot,[2.5,97.5]))]


def main():
    manifests = {}
    datasets = {}
    for stage,count in [('development',3000),('evaluation',1000)]:
        folder = OUT/stage
        manifest = json.loads((folder/'manifest.json').read_text())
        for name, checksum in manifest['runtime']['files'].items():
            assert digest(ROOT/'py'/name) == checksum, 'Source mismatch: '+name
        if '--strict-runtime' in sys.argv:
            assert manifest['runtime'] == sources(), 'Runtime mismatch'
        rows,expected = load_rows(folder,manifest)
        assert set(rows) == set(expected) and len(rows) == count
        pool = 'export_dev' if stage=='development' else 'export_confirm'
        for name,checksum in manifest['scenarios'].items():
            assert digest(ROOT/'v2/runs/v2'/pool/(name+'.json')) == checksum
        manifests[stage]=manifest
        datasets[stage]=list(rows.values())
    selected=json.loads((OUT/'development/selection.json').read_text())
    assert digest(OUT/'development/manifest.json')==selected['manifest_sha256']
    assert digest(OUT/'development/selection.json')==manifests['evaluation']['selection_sha256']
    for name,checksum in selected['results_sha256'].items():
        assert digest(OUT/'development/trials'/(name+'.json'))==checksum
    assert selected['margin']==select(summarize(datasets['development']))==.02
    src=ROOT/'v2/runs/v2/confirm_factorial.json'
    assert digest(src)=='3892404078d40782ad876fa516ef8d0f83b15cae4ed61950aeeeed8341f5bb3a'
    original=json.loads(src.read_text())
    pairs={}
    for row in original:
        assert not row['failed']
        name=row['scenario'];arm=row['arm']
        target=pairs.setdefault(name,{})
        assert arm not in target
        target[arm]=dict(safe=int(row['safe']),unsafe=int(row['unsafe_goal']),
                         deadlock=int(row['deadlock']),timeout=int(row['timeout']),
                         contact=int(row['contacts']>0))
    for row in datasets['evaluation']:
        name=row['scenario'];arm='orca-guided' if row['planner']=='hybrid' else 'orca-direct'
        assert arm not in pairs[name]
        pairs[name][arm]=dict(safe=int(row['safe']),
                              unsafe=int(row['outcome']=='success' and not row['safe']),
                              deadlock=int(row['outcome']=='deadlock'),
                              timeout=int(row['outcome']=='timeout'),
                              contact=int(row['ped_collisions']+row['obs_collisions']>0))
    arms=['sfm','sfm-ttc','hybrid','hybrid-ttc','orca-direct','orca-guided']
    names=sorted(pairs)
    assert len(names)==500
    for name in names:
        assert set(pairs[name])==set(arms)
        for r in pairs[name].values():assert sum(r[k] for k in ['safe','unsafe','deadlock','timeout'])==1
    counts={arm:{k:sum(pairs[n][arm][k] for n in names) for k in ['safe','unsafe','deadlock','timeout','contact']} for arm in arms}
    comparisons=[]
    for a,b in [('hybrid','orca-guided'),('hybrid-ttc','orca-guided'),('sfm','orca-direct'),('sfm-ttc','orca-direct')]:
        result=dict(reference=a,comparator=b,metrics={})
        for metric in ['safe','unsafe','deadlock','contact']:
            v=[pairs[n][b][metric]-pairs[n][a][metric] for n in names]
            result['metrics'][metric]=interval(v,[n.split('_')[1] for n in names],a+b+metric)
        result['safe_gains']=sum(not pairs[n][a]['safe'] and pairs[n][b]['safe'] for n in names)
        result['safe_losses']=sum(pairs[n][a]['safe'] and not pairs[n][b]['safe'] for n in names)
        remaining=[n for n in names if n not in PRIOR_INSPECTION]
        result['excluding_inspected_safe_delta']=float(np.mean([pairs[n][b]['safe']-pairs[n][a]['safe'] for n in remaining]))
        comparisons.append(result)
    conditions={c:{a:sum(pairs[n][a]['safe'] for n in names if n.split('_')[1]==c) for a in arms} for c in ['baseline','bigObs','cluttered','dense']}
    audit=dict(complete=True,development_trials=3000,evaluation_trials=1000,
               selected_margin=selected['margin'],counts=counts,comparisons=comparisons,
               condition_safe_counts=conditions,prior_inspection=PRIOR_INSPECTION,
               analysis='Descriptive post-evaluation paired percentile intervals, 10000 stratified resamples; no multiplicity-adjusted or confirmatory inference',
               source_sha256=digest(__file__),
               evaluation_hashes={p.name:digest(p) for p in sorted((OUT/'evaluation/trials').glob('*.json'))})
    (OUT/'analysis.json').write_text(json.dumps(audit,indent=2,sort_keys=True)+'\n')
    lines=['# ORCA robot results','',audit['analysis'],'', '| Controller | Safe | Unsafe arrival | Deadlock | Timeout | Any contact |', '|---|---:|---:|---:|---:|---:|']
    for arm,c in counts.items():lines.append('| '+arm+' | '+' | '.join(f'{c[k]}/500 ({c[k]/5:.1f}%)' for k in ['safe','unsafe','deadlock','timeout','contact'])+' |')
    for comp in comparisons:
        lines+=['',f"## {comp['comparator']} minus {comp['reference']}"]
        for k,v in comp['metrics'].items():lines.append(f'- {k}: {v[0]*100:+.1f} percentage points [{v[1]*100:+.1f}, {v[2]*100:+.1f}]')
        lines.append(f"- Safe gains/losses: {comp['safe_gains']}/{comp['safe_losses']}")
        lines.append(f"- Safe delta excluding three inspected scenarios: {100*comp['excluding_inspected_safe_delta']:+.2f} percentage points (497 pairs)")
    (OUT/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))
    print('Condition safe counts:',conditions)

if __name__=='__main__':main()
