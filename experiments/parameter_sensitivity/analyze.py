"""Summarize a completed fixed parameter screen without selecting settings."""
from pathlib import Path
from collections import defaultdict
import json,hashlib
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[1]
def main():
 m=json.loads((BASE/'manifest.json').read_text())['spec']
 for name,digest in m['sources'].items():assert hashlib.sha256((ROOT/'py'/name).read_bytes()).hexdigest()==digest
 for name,digest in m['inputs'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest
 for name,key in [('run.py','runner_sha256'),('PROTOCOL.md','protocol_sha256')]:assert hashlib.sha256((BASE/name).read_bytes()).hexdigest()==m[key]
 rows=[json.loads(p.read_text()) for p in sorted((BASE/'trials').glob('*.json'))]
 assert len(rows)==1400,'Screen incomplete'
 assert not any(r['failed'] for r in rows),'Inspect failed trials before analysis'
 names={Path(p).stem for p in m['inputs']};settings=m['settings']
 expected={(s,k,a) for s in names for k in settings for a in ['hybrid','hybrid-ttc']}
 assert len({(r['scenario'],r['setting'],r['arm']) for r in rows})==len(rows)
 assert {(r['scenario'],r['setting'],r['arm']) for r in rows}==expected
 per=defaultdict(dict)
 for r in rows:
  assert sum(r[k] for k in ['safe','unsafe_goal','deadlock','timeout'])==1
  per[(r['setting'],r['arm'])][r['scenario']]=r
 stats={};lines=['# Parameter screen results','','Exploratory development-set screen; 100 shared scenarios per setting and arm.','Counts are out of 100. Effects are descriptive, without confirmatory inference.','','| Setting | Arm | Safe | Unsafe arrival | Deadlock | Timeout | Any contact |','|---|---|---:|---:|---:|---:|---:|']
 for k in settings:
  stats[k]={}
  for a in ['hybrid','hybrid-ttc']:
   rr=list(per[k,a].values());q={f:sum(r[f] for r in rr) for f in ['safe','unsafe_goal','deadlock','timeout']};q['any_contact']=sum(r['contacts']>0 for r in rr);stats[k][a]=q
   lines.append('| '+k+' | '+a+' | '+' | '.join(str(v) for v in q.values())+' |')
  stats[k]['paired_ttc_safe_difference_pp']=stats[k]['hybrid-ttc']['safe']-stats[k]['hybrid']['safe']
 for k in settings:stats[k]['change_from_nominal_pp']=stats[k]['paired_ttc_safe_difference_pp']-stats['nominal']['paired_ttc_safe_difference_pp']
 lines+=['','| Setting | TTC safe-completion difference (pp) | Change from nominal (pp) |','|---|---:|---:|']
 for k in settings:lines.append(f"| {k} | {stats[k]['paired_ttc_safe_difference_pp']:+d} | {stats[k]['change_from_nominal_pp']:+d} |")
 (BASE/'analysis.json').write_text(json.dumps(stats,indent=2)+'\n')
 conditions={}
 lines+=['','## Condition summaries','','Each condition has 25 scenarios per setting and arm. These are descriptive subsets.','','| Setting | Condition | SFM safe | SFM + TTC safe | Paired difference (pp) |','|---|---|---:|---:|---:|']
 for k in settings:
  conditions[k]={}
  for c in sorted({r['condition'] for r in rows}):
   a=[r for r in rows if r['setting']==k and r['condition']==c and r['arm']=='hybrid']
   b=[r for r in rows if r['setting']==k and r['condition']==c and r['arm']=='hybrid-ttc']
   assert len(a)==len(b)==25
   aa=sum(r['safe'] for r in a);bb=sum(r['safe'] for r in b)
   conditions[k][c]=dict(n=25,hybrid_safe=aa,hybrid_ttc_safe=bb,difference_pp=4*(bb-aa))
   lines.append(f'| {k} | {c} | {aa} | {bb} | {4*(bb-aa):+d} |')
 (BASE/'condition_analysis.json').write_text(json.dumps(conditions,indent=2)+'\n')
 (BASE/'RESULTS.md').write_text('\n'.join(lines)+'\n')
if __name__=='__main__':main()
