"""Run a fixed one-at-a-time navigation parameter screen."""
import hashlib,json,sys,time,os
from pathlib import Path
from datetime import datetime,timezone
from dataclasses import replace
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor,as_completed
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[1]/'SFM-A-Star-Bio-inspired-Algorithm'
sys.path.insert(0,str(ROOT/'py'))
import numpy as np
from sfm import Scenario,run_trial
SETTINGS={'nominal':{},'ped_speed_low':{'v0Ped':1.0},'ped_speed_high':{'v0Ped':1.7},'obs_radius_low':{'rInfObs':2.0},'obs_radius_high':{'rInfObs':4.0},'speed_cap_low':{'vMax':1.5},'speed_cap_high':{'vMax':2.5}}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):
 t=p.with_suffix('.tmp');t.write_text(json.dumps(v,indent=2)+'\n');t.replace(p)
def one(job):
 f,label,enabled=job;s=Scenario.load(f);s=replace(s,params=replace(s.params,**SETTINGS[label]))
 row=dict(scenario=f.stem,condition=s.condition,setting=label,arm='hybrid-ttc' if enabled else 'hybrid')
 try:
  r=run_trial(s,'hybrid',ttc=enabled);n=int(r.ped_collisions+r.obs_collisions)
  row.update(failed=False,outcome=r.outcome,safe=int(r.outcome=='success' and n==0),unsafe_goal=int(r.outcome=='success' and n>0),deadlock=int(r.outcome=='deadlock'),timeout=int(r.outcome=='timeout'),contacts=n,steps=int(r.steps),path_len=float(r.path_length),min_ped_gap=float(r.min_ped_gap),min_obs_gap=float(r.min_obs_gap))
 except Exception as e:row.update(failed=True,error=repr(e))
 return row
def main():
 import fcntl
 lock=(BASE/'.run.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 per=defaultdict(list)
 for f in sorted((ROOT/'v2/runs/v2/export_dev').glob('*.json')):per[Scenario.load(f).condition].append(f)
 assert len(per)==4 and sum(map(len,per.values()))==300
 files=[f for c in sorted(per) for f in per[c][:25]];assert len(files)==100
 spec=dict(settings=SETTINGS,sources={p.name:sha(p) for p in sorted((ROOT/'py').glob('*.py'))},inputs={str(p.relative_to(ROOT)):sha(p) for p in files},runner_sha256=sha(Path(__file__)),protocol_sha256=sha(BASE/'PROTOCOL.md'),python=sys.version,numpy=np.__version__)
 manifest=BASE/'manifest.json'
 if manifest.exists():assert json.loads(manifest.read_text())['spec']==spec,'Snapshot changed'
 else:write(manifest,dict(created_at=datetime.now(timezone.utc).isoformat(),spec=spec))
 out=BASE/'trials';out.mkdir(exist_ok=True)
 jobs=[(f,l,b) for f in files for l in SETTINGS for b in (False,True)]
 def dest(j):return out/f'{j[0].stem}__{j[1]}__{int(j[2])}.json'
 pending=[j for j in jobs if not dest(j).exists()];done=len(jobs)-len(pending);start=time.monotonic()
 write(BASE/'status.json',dict(state='running',pid=os.getpid(),completed=done,total=len(jobs)))
 with ProcessPoolExecutor(max_workers=4) as pool:
  futures={pool.submit(one,j):j for j in pending}
  for future in as_completed(futures):
   row=future.result();write(dest(futures[future]),row);done+=1
   write(BASE/'status.json',dict(state='running',pid=os.getpid(),completed=done,total=len(jobs),elapsed_s=time.monotonic()-start))
   if done%20==0:print(f'{done}/{len(jobs)}; elapsed {(time.monotonic()-start)/60:.1f} min',flush=True)
 failures=sum(json.loads(p.read_text()).get('failed',False) for p in out.glob('*.json'))
 write(BASE/'status.json',dict(state='complete',completed=done,total=len(jobs),failures=failures,elapsed_s=time.monotonic()-start))
 print('complete',done,'failures',failures,flush=True)
if __name__=='__main__':main()
