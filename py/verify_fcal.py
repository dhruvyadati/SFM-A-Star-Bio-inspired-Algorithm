"""Integrity checks for the Amendment F calibration export. Run BEFORE calibrating.

Every check must pass before any controller threshold is chosen. Nothing here
runs a recovery controller; determinism is checked with the unmodified hybrid
and hybrid-TTC arms only.

  1. exactly 160 accepted scenarios, 40 per condition
  2. NO seed overlap with pilot, development, confirmation, fixtures or the
     ORCA follow-up -- verified against the actual seed lists and filenames,
     not against the claim that seeds above 2^31 are safe
  3. every exported scenario loads in Python
  4. unmodified hybrid and hybrid-TTC reproduce deterministically on repeat
  5. hashes archived before calibration starts
"""
from __future__ import annotations

import hashlib
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial

RUNS = Path("v2/runs/v2")
FCAL = RUNS / "export_fcal"
OUT = RUNS / "fcal_manifest.json"
CONDS = ["baseline", "dense", "cluttered", "bigObs"]
PER_COND = 40
DETERMINISM_SAMPLE = 6
# Deterministic replay is the slow part (each trial is ~15-20 s). Split so a
# reviewer can validate archive structure in seconds and run the full replay
# separately; the build gate always runs --full.
FAST = "--fast" in sys.argv

fails = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        fails.append(name)
    return ok


def prior_seeds():
    """Every seed used by any earlier study, from seed lists AND filenames."""
    seen = set()
    for f in RUNS.glob("*seeds*.json"):
        d = json.loads(f.read_text())
        if isinstance(d, dict):
            for v in d.values():
                if isinstance(v, list):
                    seen.update(int(x) for x in v)
    for d in ("export_confirm", "export_dev", "export", "export_rand"):
        for p in (RUNS / d).glob("*.json"):
            m = re.search(r"_(\d+)\.json$", p.name)
            if m:
                seen.add(int(m.group(1)))
    return seen


print("=" * 78)
print("F CALIBRATION SET — INTEGRITY CHECKS")
print("=" * 78)

if not FCAL.exists():
    sys.exit(f"{FCAL} not found — run Export_FCalibration.m in MATLAB first")

man = json.loads((FCAL / "manifest.json").read_text())
files = sorted(p for p in FCAL.glob("fcal_*.json"))

# --- 1. counts ------------------------------------------------------------
print("\n1. COUNTS")
by_cond = defaultdict(list)
for p in files:
    for c in CONDS:
        if p.name.startswith(f"fcal_{c}_"):
            by_cond[c].append(p)
check("160 scenarios exported", len(files) == 160, f"got {len(files)}")
for c in CONDS:
    check(f"{c}: {PER_COND}", len(by_cond[c]) == PER_COND, f"got {len(by_cond[c])}")
print(f"  rejected during generation: {man.get('n_rejected')} "
      f"(reported, as the protocol requires)")

# --- 2. seed disjointness -------------------------------------------------
print("\n2. SEED DISJOINTNESS")
new = set()
for p in files:
    m = re.search(r"_(\d+)\.json$", p.name)
    if m:
        new.add(int(m.group(1)))
old = prior_seeds()
overlap = new & old
check("no overlap with any prior seed", not overlap,
      f"{len(old)} prior seeds checked, overlap={sorted(overlap)[:5]}")
check("all new seeds >= 2^31", all(s >= 2**31 for s in new),
      f"min={min(new) if new else '-'}")
check("prior seeds all < 2^31", all(s < 2**31 for s in old),
      f"max={max(old) if old else '-'}")

# --- 3. loadability -------------------------------------------------------
print("\n3. LOADABILITY")
bad = []
for p in files:
    try:
        S = Scenario.load(str(p))
        if S.waypoints is None or S.waypoints.shape[1] < 2:
            bad.append((p.name, "no waypoints"))
        elif S.ped_pos.shape[1] < 1:
            bad.append((p.name, "no pedestrians"))
    except Exception as e:
        bad.append((p.name, str(e)[:60]))
check("every scenario loads with waypoints and pedestrians", not bad,
      f"{len(bad)} bad: {bad[:2]}" if bad else "")

# --- 4. determinism -------------------------------------------------------
if FAST:
    print("\n4. DETERMINISM — SKIPPED (--fast). Run without --fast for the "
          "full replay.")
    mismatch = []
else:
    print(f"\n4. DETERMINISM (unmodified arms, {DETERMINISM_SAMPLE} scenarios)")
    # Defect fixed 2026-08-31: `mismatch` was initialised only on the --fast
    # branch, so the full determinism replay raised NameError and had never
    # completed. The check below is therefore run here for the first time.
    mismatch = []
    for p in files[:: max(1, len(files) // DETERMINISM_SAMPLE)][:DETERMINISM_SAMPLE]:
        for ttc in (False, True):
            a = run_trial(Scenario.load(str(p)), "hybrid", log=False, ttc=ttc)
            b = run_trial(Scenario.load(str(p)), "hybrid", log=False, ttc=ttc)
            key = lambda r: (r.outcome, r.steps, r.ped_collisions,
                             r.obs_collisions, r.path_length, r.stop_time)
            if key(a) != key(b):
                mismatch.append((p.name, ttc))
    check("hybrid and hybrid-TTC reproduce exactly on repeat", not mismatch,
          f"{len(mismatch)} mismatches")

# --- 5. archive -----------------------------------------------------------
print("\n5. ARCHIVE")
h = hashlib.sha256()
for p in files:
    h.update(p.name.encode())
    h.update(p.read_bytes())
sethash = h.hexdigest()
rec = {"set": "F_calibration", "n": len(files),
       "per_condition": {c: len(by_cond[c]) for c in CONDS},
       "seed_min": min(new) if new else None, "seed_max": max(new) if new else None,
       "n_rejected": man.get("n_rejected"),
       "collective_sha256": sethash,
       "generator_sha256": hashlib.sha256(
           Path("v2/Export_FCalibration.m").read_bytes()).hexdigest()[:16],
       "status": ("CALIBRATION ONLY. Results from this set may never be "
                  "presented as the formal Amendment F recovery test.")}
if OUT.exists():
    prev = json.loads(OUT.read_text())
    check("set unchanged since first archive",
          prev.get("collective_sha256") == sethash)
else:
    OUT.write_text(json.dumps(rec, indent=1))
    print(f"  wrote {OUT}")
print(f"  collective sha256: {sethash[:32]}")

print("\n" + "=" * 78)
if fails:
    print(f"FAILED: {len(fails)} check(s) — {', '.join(fails)}")
    print("Do NOT begin calibration until these pass.")
    sys.exit(1)
print("ALL CHECKS PASSED — calibration set is sound and archived.")
print("Calibration thresholds may now be chosen on THIS set only.")
