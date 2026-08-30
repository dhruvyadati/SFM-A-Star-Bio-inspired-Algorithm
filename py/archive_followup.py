"""Archive raw follow-up outputs BEFORE any analysis reads them.

Ordering matters. Hashing the raw outputs first means the analysis cannot be
re-run against quietly altered inputs, and any later change to a raw file is
detectable. The manifest records what produced each file, not just its digest.

Run immediately after the sweeps finish and BEFORE analyze_dose.py,
diversion_verdict.py or diversion_diagnostics.py.

Re-running is safe: an existing manifest is verified rather than overwritten,
and a changed digest is reported as a discrepancy instead of being silently
replaced.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

RUNS = Path("v2/runs/v2")
PY = Path("py")
MANIFEST = RUNS / "followup_manifest.json"

RAW_OUTPUTS = [
    RUNS / "dose_response.json",
    RUNS / "trajectory_diversion.json",
    RUNS / "trajectory_diversion_paths.npz",
    RUNS / "reciprocal.json",
]
# Executables whose exact content determines what the raw outputs mean.
SCRIPTS = [
    PY / "sfm_explore.py", PY / "ttc.py",
    PY / "dose_response.py", PY / "trajectory_diversion.py",
    PY / "analyze_dose.py", PY / "diversion_verdict.py",
    PY / "diversion_diagnostics.py", PY / "Test_AnalyzeDose.py",
    PY / "reciprocal.py", PY / "run_reciprocal.py",
    PY / "analyze_reciprocal.py", PY / "orca_cap_diagnostic.py",
]
# Inputs the follow-ups depend on but must not modify.
INPUTS = [RUNS / "confirm_factorial.json"]


def digest(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def collect():
    out = {}
    for group, files in (("raw_outputs", RAW_OUTPUTS), ("scripts", SCRIPTS),
                         ("inputs", INPUTS)):
        out[group] = {}
        for p in files:
            if p.exists():
                out[group][str(p)] = {"sha256": digest(p), "bytes": p.stat().st_size}
            else:
                out[group][str(p)] = {"sha256": None, "bytes": None,
                                      "note": "ABSENT at archive time"}
    return out


def describe():
    """Provenance the digests alone do not carry."""
    d = {}
    f = RUNS / "dose_response.json"
    if f.exists():
        rows = json.load(open(f))
        planners = sorted({r["planner"] for r in rows})
        cfgs = sorted({r["config"] for r in rows})
        scens = sorted({r["scenario"] for r in rows})
        d["dose_response"] = {
            "runs": len(rows), "planners": planners, "configs": cfgs,
            "scenarios": len(scens),
            "expected_runs": len(planners) * len(cfgs) * len(scens),
            "complete": len(rows) == len(planners) * len(cfgs) * len(scens),
        }
    f = RUNS / "trajectory_diversion.json"
    if f.exists():
        raw = json.load(open(f))
        d["trajectory_diversion"] = {
            g: {"n": len(rows),
                "scenarios": sorted(r["scenario"] for r in rows)}
            for g, rows in raw.items()}
    return d


def main():
    cur = {"files": collect(), "provenance": describe(),
           "status": ("Follow-up analyses. Mechanistic and exploratory. "
                      "The dose-response sweep is a PAIRED FOLLOW-UP on the "
                      "confirmed scenario set (Amendment D.1), not an "
                      "independent replication. The trajectory analysis is "
                      "conditional on outcome transition class and cannot "
                      "estimate prevalence across all 500 scenarios.")}

    missing = [n for g in cur["files"].values() for n, v in g.items()
               if v["sha256"] is None]
    for n in missing:
        print(f"  ABSENT: {n}")

    if MANIFEST.exists():
        old = json.load(open(MANIFEST))
        diffs = []
        for group, files in old.get("files", {}).items():
            for name, rec in files.items():
                new = cur["files"].get(group, {}).get(name, {})
                if rec["sha256"] and new.get("sha256") and rec["sha256"] != new["sha256"]:
                    diffs.append(f"{name}: {rec['sha256'][:12]} -> {new['sha256'][:12]}")
        if diffs:
            print("\n  MANIFEST DISCREPANCY — a previously archived file changed:")
            for d in diffs:
                print(f"    {d}")
            reason = None
            if len(sys.argv) > 2 and sys.argv[1] == "--resolve":
                reason = sys.argv[2]
            if reason is None:
                print("  Not overwriting. Resolve before interpreting any analysis.")
                print('  To record a legitimate change: '
                      '--resolve "why the file changed"')
                return 1
            # A recorded amendment, never a silent overwrite. The superseded
            # digests stay in the history so the change is auditable.
            hist = old.get("history", [])
            hist.append({"superseded": diffs, "reason": reason,
                         "raw_outputs_unchanged": not any(
                             d.split(":")[0].strip().startswith("v2/runs")
                             for d in diffs)})
            cur["history"] = hist
            MANIFEST.write_text(json.dumps(cur, indent=1, sort_keys=True))
            print(f"\n  AMENDED and recorded in history: {reason}")
            print(f"  raw outputs unchanged: {cur['history'][-1]['raw_outputs_unchanged']}")
            return 0
        print(f"\n  manifest unchanged, verified against {MANIFEST}")
        return 0

    MANIFEST.write_text(json.dumps(cur, indent=1, sort_keys=True))
    print(f"\n  wrote {MANIFEST}")
    for group, files in cur["files"].items():
        print(f"\n  {group}")
        for name, rec in files.items():
            h = rec["sha256"][:16] if rec["sha256"] else "ABSENT"
            print(f"    {h}  {name}")
    p = cur["provenance"].get("dose_response")
    if p:
        print(f"\n  dose_response: {p['runs']}/{p['expected_runs']} runs, "
              f"complete={p['complete']}")
    t = cur["provenance"].get("trajectory_diversion")
    if t:
        for g, v in t.items():
            print(f"  trajectory {g}: n={v['n']}")
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
