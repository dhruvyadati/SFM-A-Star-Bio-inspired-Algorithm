"""Describe sampled surface gaps in fixed navigation outputs."""
from pathlib import Path
from collections import defaultdict
import hashlib,json
import numpy as np
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[1]
SOURCE=ROOT/'v2/runs/v2/confirm_factorial.json'
def main():
 rows=json.loads(SOURCE.read_text());assert len(rows)==2000 and not any(r['failed'] for r in rows)
 sources={str(SOURCE.relative_to(ROOT)):hashlib.sha256(SOURCE.read_bytes()).hexdigest()}
 for p in sorted((ROOT/'experiments/orca_robot/aligned/evaluation/trials').glob('*.json')):
  r=json.loads(p.read_text());sources[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
  rows.append(dict(scenario=r['scenario'],arm='orca-'+('guided' if r['planner']=='hybrid' else r['planner']),min_ped_gap=r['min_ped_gap'],min_obs_gap=r['min_obs_gap'],contacts=r['ped_collisions']+r['obs_collisions']))
 assert len(rows)==3000
 groups=defaultdict(list)
 for r in rows:groups[r['arm']].append(r)
 assert len(groups)==6
 results={};lines=['# Sampled clearance diagnostics','','Post hoc descriptive analysis of minimum recorded robot surface gaps.','Each arm uses the same 500 scenarios. No uncertainty or confirmatory test is added.','A contact-free close pass means no recorded contact and at least one sampled','surface gap at or below 0.15 m (an operational threshold, not validated comfort).','A margin encounter can include contact. Trials have different durations.','These summaries do not detect collisions between samples.','','| Arm | Any contact | Contact-free close pass | Any gap <= 0.15 m | Any gap <= 0.40 m | Median minimum pedestrian gap (m) | Median minimum obstacle gap (m) |','|---|---:|---:|---:|---:|---:|---:|']
 for a in ['sfm','sfm-ttc','hybrid','hybrid-ttc','orca-direct','orca-guided']:
  rr=groups[a];assert len(rr)==500 and len({r['scenario'] for r in rr})==500
  pg=np.array([r['min_ped_gap'] for r in rr]);og=np.array([r['min_obs_gap'] for r in rr]);g=np.minimum(pg,og);contact=np.array([r['contacts']>0 for r in rr]);assert np.isfinite(g).all()
  assert np.array_equal(g<0,contact),'Mismatch with sampled contact scoring'
  q=dict(n=500,any_contact=int(contact.sum()),contact_free_close_pass_015=int(((~contact)&(g<=.15)).sum()),gap_le_015=int((g<=.15).sum()),gap_le_040=int((g<=.4).sum()),min_ped_gap_median=float(np.median(pg)),min_obs_gap_median=float(np.median(og)),min_ped_gap_q10=float(np.quantile(pg,.1)),min_obs_gap_q10=float(np.quantile(og,.1)))
  results[a]=q
  lines.append(f"| {a} | {q['any_contact']} | {q['contact_free_close_pass_015']} | {q['gap_le_015']} | {q['gap_le_040']} | {q['min_ped_gap_median']:.3f} | {q['min_obs_gap_median']:.3g} |")
 (BASE/'analysis.json').write_text(json.dumps(dict(post_hoc=True,thresholds_m=[.15,.4],results=results,sources=sources),indent=2)+'\n')
 (BASE/'RESULTS.md').write_text('\n'.join(lines)+'\n')
if __name__=='__main__':main()
