"""Result tables (LaTeX), from FROZEN archived results only.

Run: python3 py/make_tables.py   ->  tables/*.tex

Tables for the recovery and route-recovery studies are absent by design
until those studies exist.
"""
import json, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))

R = Path("v2/runs/v2"); T = Path("../paper/tables"); T.mkdir(exist_ok=True)
def cat(r):
    if r["outcome"]=="success": return "safe" if r["contacts"]==0 else "unsafe"
    return "deadlock" if r["outcome"]=="deadlock" else "other"
def w(name, s):
    (T/f"{name}.tex").write_text(s); print(f"  {name}.tex")

# T1 — confirmatory outcomes by arm
rows=json.load(open(R/"confirm_factorial.json"))
by=defaultdict(list)
for r in rows:
    if not r.get("failed"): by[r["arm"]].append(cat(r))
DISP={"sfm":"SFM","sfm-ttc":"SFM--TTC","hybrid":"A*--SFM","hybrid-ttc":"A*--SFM--TTC"}
L=[r"\begin{tabular}{lrrrr}", r"\toprule",
   r"Arm & Safe (\%) & Unsafe arrival (\%) & Deadlock (\%) & Timeout (\%) \\", r"\midrule"]
for a in ("sfm","sfm-ttc","hybrid","hybrid-ttc"):
    v=by[a]; f=lambda c: 100*np.mean([x==c for x in v])
    L.append(f"{DISP[a]} & {f('safe'):.1f} & {f('unsafe'):.1f} & {f('deadlock'):.1f} & {f('other'):.1f} \\\\")
L+=[r"\bottomrule", r"\end{tabular}"]
w("t1_outcomes", "\n".join(L))

# T2 — transitions
b2=defaultdict(dict)
for r in rows:
    if not r.get("failed"): b2[r["scenario"]][r["arm"]]=r
comp=[s for s in b2 if len(b2[s])==4]
M=defaultdict(int)
for s in comp: M[(cat(b2[s]["hybrid"]),cat(b2[s]["hybrid-ttc"]))]+=1
L=[r"\begin{tabular}{lrl}", r"\toprule",
   r"Transition & $n$ & Interpretation \\", r"\midrule"]
# DESCRIPTIVE only. Interpretive phrasing such as "safety bought by
# abandoning progress" belongs in the discussion, not in the table.
for a,b,t in (("unsafe","safe","unsafe arrival converted to safe arrival"),
              ("unsafe","deadlock","unsafe arrival converted to deadlock"),
              ("safe","unsafe","safe arrival converted to unsafe arrival"),
              ("safe","deadlock","safe arrival converted to deadlock"),
              ("deadlock","safe","deadlock converted to safe arrival")):
    L.append(f"{a} $\\rightarrow$ {b} & {M[(a,b)]} & {t} \\\\")
L+=[r"\bottomrule", r"\end{tabular}"]
w("t2_transitions", "\n".join(L))

# T3 — TTC effect across pedestrian-RESPONSE settings
conf=b2
rec=defaultdict(dict)
for r in json.load(open(R/"reciprocal.json")):
    if r["set"]=="confirm" and r["responsibility"]=="reciprocal": rec[r["scenario"]][r["arm"]]=r
cn=json.load(open(R/"crowdnav_transfer.json"))
from reporting_stats import reporting_ci as ci
def dpair(pairs,a,b,key,tag):
    c=lambda r: r["category"] if "category" in r else cat(r)
    pr=[p for p in pairs.values() if a in p and b in p]
    v=[(c(p[b])==key)-(c(p[a])==key) for p in pr]
    return ci(v,tag)+(float(np.mean([c(p[a])==key for p in pr])),)
def dcn(vis,key,tag):
    p=defaultdict(dict)
    for r in cn:
        if r["dt"]==0.25 and r["visible"]==vis: p[(r["rule"],r["n_human"],r["seed"])][r["arm"]]=r
    pr=[v for v in p.values() if len(v)==2]
    v=[float(x["ttc"][key])-float(x["reference"][key]) for x in pr]
    return ci(v,tag)+(float(np.mean([float(x["reference"][key]) for x in pr])),)
def fmt(t_): return f"{t_[0]:+.3f} [{t_[1]:+.3f}, {t_[2]:+.3f}]"
ROWS=[("Custom", "SFM (weak repulsion)", "unsafe arrival",
       dpair(conf,'hybrid','hybrid-ttc','unsafe','x1'), dpair(conf,'hybrid','hybrid-ttc','deadlock','x2')),
      ("Custom", "ORCA-responsive, asymmetric", "unsafe arrival",
       dpair(rec,'disabled','ttc0.40','unsafe','x3'), dpair(rec,'disabled','ttc0.40','deadlock','x4')),
      ("CrowdNav", "robot invisible, $0.25$ s", "collision",
       dcn(False,'collision','x5'), dcn(False,'deadlock','x6')),
      ("CrowdNav", "robot visible, $0.25$ s", "collision",
       dcn(True,'collision','x7'), dcn(True,'deadlock','x8'))]
L=[r"\begin{tabular}{lllrrr}", r"\toprule",
   r"Environment & Pedestrian response & Safety endpoint & Baseline (\%) & $\Delta$ safety & $\Delta$ deadlock \\", r"\midrule"]
for env,resp,ep,s,d_ in ROWS:
    L.append(f"{env} & {resp} & {ep} & {100*s[3]:.1f} & {fmt(s)} & {fmt(d_)} \\\\")
L+=[r"\bottomrule", r"\end{tabular}"]
w("t3_across_settings", "\n".join(L))

# T4 — INTERNAL ONLY. Not for the paper: a project-management artefact that
# goes stale, and readers do not need a table of completion percentages.
# Written to tables/internal/ so it cannot be \input by accident.
# T4 — reviewer status
import re
src=Path("../resubmission/REVIEWER_COMMENTS.md")
if not src.exists():
    print("  (comment tracker absent; skipping status table)"); raise SystemExit(0)
t=src.read_text()
c=defaultdict(int)
for l in t.split("\n"):
    if l.startswith("| ") and l.count("|")>=5:
        p=[x.strip() for x in l.split("|")]
        if len(p)<6 or p[1] in ("#","ID"): continue
        st=re.sub(r"\*\*","",p[4]).split("—")[0].strip().upper()
        for k in ("DONE","PARTIAL","TODO","WRITE","DECISION","IN PROGRESS"):
            if st.startswith(k): c[k]+=1; break
tot=sum(c.values())
L=[r"\begin{tabular}{lrr}", r"\toprule", r"Status & $n$ & \% \\", r"\midrule"]
for k,v in sorted(c.items(), key=lambda kv:-kv[1]):
    L.append(f"{k.title()} & {v} & {100*v/tot:.0f} \\\\")
L+=[r"\midrule", f"Total & {tot} & 100 \\\\", r"\bottomrule", r"\end{tabular}"]
(T/"internal").mkdir(exist_ok=True)
(T/"internal"/"t4_reviewer_status_INTERNAL.tex").write_text("\n".join(L))
print("  internal/t4_reviewer_status_INTERNAL.tex  (NOT for the paper)")
print(f"\n  tables -> {T}")

# T6 — complete timestep disclosure, including overlapping matched subsets.
L = [r"\begin{tabular}{llrrrrr}", r"\toprule",
     r"Visibility & Step and subset & $n$ & Collision ref. & Collision TTC & Deadlock ref. & Deadlock TTC \\",
     r"\midrule"]
for vis in (False, True):
    for dt, limit, label in ((.25, 50, "$0.25$ s, full"),
                             (.25, 10, "$0.25$ s, matched subset"),
                             (.01, 10, "$0.01$ s, matched subset")):
        arm_rows = {arm: [r for r in cn if r['visible'] == vis
                         and r['dt'] == dt and r['seed'] < limit
                         and r['arm'] == arm] for arm in ('reference', 'ttc')}
        a, b = arm_rows['reference'], arm_rows['ttc']
        assert len(a) == len(b) == (200 if limit == 50 else 40)
        cells = ['Visible' if vis else 'Invisible', label, str(len(a))]
        cells += [str(sum(r[key] for r in group))
                  for key in ('collision', 'deadlock') for group in (a, b)]
        L.append(' & '.join(cells) + r' \\')
L += [r"\bottomrule", r"\end{tabular}"]
w('t6_crowdnav_timestep', '\n'.join(L) + '\n')
