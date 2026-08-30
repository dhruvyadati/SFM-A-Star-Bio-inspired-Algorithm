"""Pre-analysis integrity gate. Runs BEFORE the frozen primary analysis.

Separate from analyze_confirm.py so the hashed primary script is not modified
to accommodate a check. Exits non-zero on any failure, so an incomplete or
altered run cannot silently enter analysis.

Verifies:
  1. executable files still match the hashes recorded before the run
  2. the raw result file is hashed and preserved before it is opened for analysis
  3. expected scenario and arm counts are present
  4. no duplicate or missing scenario-arm pairs
"""
import hashlib
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

EXPECTED_SCENARIOS = 500
EXPECTED_ARMS = {"sfm", "sfm-ttc", "hybrid", "hybrid-ttc"}
EXPECTED_PER_COND = 125
CONDS = {"baseline", "dense", "cluttered", "bigObs"}
RESULT = Path("v2/runs/v2/confirm_factorial.json")

fails = []
print("=" * 68)
print("PRE-ANALYSIS INTEGRITY GATE")
print("=" * 68)

# --- 1. executables unchanged since the pre-run record --------------
env = json.loads(Path("v2/runs/v2/confirm_environment.json").read_text())
print("\n1. Executable hashes vs the record made before the run")
for f, expected in env["hashes"].items():
    p = Path(f)
    if not p.exists():
        fails.append(f"missing file {f}")
        print(f"   MISSING  {f}")
        continue
    actual = hashlib.sha256(p.read_bytes()).hexdigest()[:16]
    ok = actual == expected
    print(f"   {'ok  ' if ok else 'DIFF'} {f:<44} {actual}")
    if not ok:
        fails.append(f"{f}: {expected} -> {actual}")

# --- 2. hash the raw result before analysis ------------------------
print("\n2. Raw result file")
if not RESULT.exists():
    print("   result file does not exist yet — run has not finished")
    sys.exit(2)
raw = RESULT.read_bytes()
digest = hashlib.sha256(raw).hexdigest()
print(f"   sha256 {digest}")
print(f"   bytes  {len(raw):,}")
Path("v2/runs/v2/confirm_result_hash.txt").write_text(
    f"{digest}  confirm_factorial.json\n")

# --- 3 & 4. completeness -------------------------------------------
res = json.loads(raw)
print(f"\n3. Run counts")
print(f"   records: {len(res)} (expected {EXPECTED_SCENARIOS * 4})")
if len(res) != EXPECTED_SCENARIOS * 4:
    fails.append(f"record count {len(res)} != {EXPECTED_SCENARIOS * 4}")

failed = [r for r in res if r.get("failed")]
print(f"   failed runs: {len(failed)}")

pairs = Counter((r["scenario"], r["arm"]) for r in res)
dups = {k: v for k, v in pairs.items() if v > 1}
print(f"\n4. Scenario-arm pair integrity")
print(f"   duplicate pairs: {len(dups)}")
if dups:
    fails.append(f"{len(dups)} duplicate scenario-arm pairs")
    for k in list(dups)[:5]:
        print(f"      {k} x{dups[k]}")

by = defaultdict(set)
cond = {}
for r in res:
    by[r["scenario"]].add(r["arm"])
    if not r.get("failed"):
        cond[r["scenario"]] = r["condition"]
incomplete = {s: EXPECTED_ARMS - a for s, a in by.items() if a != EXPECTED_ARMS}
print(f"   scenarios: {len(by)} (expected {EXPECTED_SCENARIOS})")
print(f"   scenarios missing an arm: {len(incomplete)}")
if len(by) != EXPECTED_SCENARIOS:
    fails.append(f"scenario count {len(by)} != {EXPECTED_SCENARIOS}")
for s, miss in list(incomplete.items())[:5]:
    print(f"      {s} missing {sorted(miss)}")

cc = Counter(cond.values())
print(f"\n   per condition: {dict(cc)}")
for c in CONDS:
    if cc.get(c, 0) != EXPECTED_PER_COND:
        fails.append(f"condition {c}: {cc.get(c,0)} != {EXPECTED_PER_COND}")

print("\n" + "=" * 68)
if fails:
    print(f"GATE FAILED — {len(fails)} problem(s). Do NOT run the analysis.")
    for f in fails:
        print("  -", f)
    sys.exit(1)
print("GATE PASSED — run is complete and executables are unchanged.")
print("Proceed to the frozen primary analysis.")
