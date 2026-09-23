"""Capture deterministic navigation states without changing the force update."""
import sys,json,hashlib,time
from pathlib import Path
from collections import defaultdict
import numpy as np
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[1]/'SFM-A-Star-Bio-inspired-Algorithm'
sys.path.insert(0,str(ROOT/'py'))
import sfm

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 out=BASE/'traces';out.mkdir(exist_ok=True)
 files=defaultdict(list)
 for p in sorted((ROOT/'v2/runs/v2/export_confirm').glob('*.json')):files[sfm.Scenario.load(p).condition].append(p)
 selected=[files[c][0] for c in sorted(files)]
 spec=dict(selection='First lexicographic scenario per condition, chosen without checking its outcome for this diagnostic.',sources={p.name:sha(p) for p in sorted((ROOT/'py').glob('*.py'))},inputs={str(p.relative_to(ROOT)):sha(p) for p in selected},runner_sha256=sha(Path(__file__)),position_stride=10)
 m=BASE/'trace_manifest.json'
 if m.exists():assert json.loads(m.read_text())==spec
 else:m.write_text(json.dumps(spec,indent=2)+'\n')
 frozen={(r['scenario'],r['arm']):r for r in json.loads((ROOT/'v2/runs/v2/confirm_factorial.json').read_text())}
 original=sfm.compute_forces
 for f in selected:
  for enabled in [False,True]:
   a='hybrid-ttc' if enabled else 'hybrid';name=f.stem+'__'+a
   if (out/(name+'.json')).exists():continue
   positions=[];times=[];pg=[];og=[];n=0
   def capture(x,v,goals,radii,v0,obstacles,P):
    nonlocal n
    result=original(x,v,goals,radii,v0,obstacles,P)
    if n%10==0:positions.append(x.copy());times.append(n*P.dt)
    pg.append(float(result[1][0]));og.append(float(result[2][0]));n+=1
    return result
   sfm.compute_forces=capture
   try:
    s=sfm.Scenario.load(f);r=sfm.run_trial(s,'hybrid',ttc=enabled,log=True)
   finally:sfm.compute_forces=original
   expected=frozen[f.stem,a];assert r.outcome==expected['outcome'] and r.steps==expected['steps']
   assert r.ped_collisions+r.obs_collisions==expected['contacts']
   assert np.isclose(r.min_ped_gap,expected['min_ped_gap'],rtol=1e-9,atol=1e-10)
   assert np.isclose(r.min_obs_gap,expected['min_obs_gap'],rtol=1e-9,atol=1e-10)
   assert np.isclose(r.path_length,expected['path_len'],rtol=1e-9,atol=1e-10)
   np.savez_compressed(out/(name+'.npz'),positions=np.array(positions),position_times=np.array(times),gap_times=np.arange(n)*s.params.dt,ped_gap=np.array(pg),obs_gap=np.array(og),robot_poststep=r.traj)
   (out/(name+'.json')).write_text(json.dumps(dict(scenario=f.stem,condition=s.condition,arm=a,outcome=r.outcome,steps=r.steps,contacts=r.ped_collisions+r.obs_collisions,matches_saved_endpoints=True),indent=2)+'\n')
   print(name,r.outcome,'matches frozen record',flush=True)
if __name__=='__main__':main()
