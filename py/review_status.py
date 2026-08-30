"""Emit review_status.json — machine-readable audit state for each review finding.

Run: python3 py/review_status.py

Hashes and file existence are read from the REPOSITORY, not typed, so the
status file cannot drift from reality the way prose does. Each finding names
the evidence file that settles it; if that file is missing the finding is
reported as unsupported regardless of what its status says.
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "v2" / "runs" / "v2" / "review_status.json"

# (id, finding, status, evidence paths, note)
FINDINGS = [
    ("R1", "PathProgress lost backward motion across a passed waypoint",
     "fixed", ["py/path_progress.py", "py/test_path_progress.py"],
     "Monotone index replaced by a displacement-plausibility bound. Reproduced "
     "the reported case before fixing; regression test is the reviewer's exact "
     "walk 0.9/1.1/0.8/0.2."),
    ("R2", "Recovery/path/TTC tests absent from the bundle",
     "fixed", ["py/test_path_progress.py", "py/test_ttc_result_compatibility.py",
               "py/test_recovery_integration.py", "py/test_recovery_state_machine.py"],
     "39 tests across four suites, one command: python3 -m pytest py/test_*.py"),
    ("R3", "Test_AnalyzeDose.py failed",
     "fixed", ["py/Test_AnalyzeDose.py"],
     "Synthetic generator now mirrors the real shared-centre labelling. It "
     "passed when written, then analyze_dose.py changed and it was not re-run."),
    ("R4", "Modified py/ttc.py broke historical integrity verification",
     "fixed", ["archive/confirm_code/py/ttc.py",
               "py/test_ttc_result_compatibility.py"],
     "Confirmatory sources archived separately; manifest NOT rewritten."),
    ("R5", "verify_fcal.py cannot run from the compact bundle",
     "fixed", ["py/verify_fcal.py", "README.md"],
     "README documents regeneration (matlab -batch Export_FCalibration; "
     "verify_fcal.py) with the expected hash, and classifies the geometry and "
     "replay checks as recorded historical checks."),
    ("R6", "Recovery code integrated nowhere",
     "fixed", ["py/sfm_explore.py", "py/test_recovery_integration.py"],
     "Wired into run_trial with fixed execution order; six end-to-end tests."),
    ("R7", "Progress/activation window one step short of documented",
     "fixed", ["py/recovery.py"],
     "int(round(window/dt))+1; constructor rejects non-multiples of dt."),
    ("R8", "Emergency flags need an exact semantic invariant",
     "fixed", ["py/ttc.py", "py/test_recovery_integration.py",
               "v2/runs/v2/integration_fixtures.json"],
     "Flags derive from the same weights as the command. Precedence tested "
     "mid-episode against a PINNED fixture (1013 co-occurrences, 0 violations) "
     "so it can no longer skip."),
    ("R9", "Cooldown semantics not reflected in metrics",
     "fixed", ["py/recovery.py"],
     "Termination reason, suppressed triggers, rearmed state and release "
     "integral all recorded."),
    ("R10", "Amendment F stale in several material places",
     "fixed", ["../resubmission/AMENDMENT_F_DRAFT.md"],
     "Progress definition, prospective pool, Stage A grid, selection rule, "
     "cooldown semantics, episode cap and live hashes all updated."),
    ("R11", "Environment record mixes superseded and current claims",
     "fixed", ["../resubmission/ENVIRONMENT.md"],
     "Every line of the superseded section is quoted, so no sentence in it "
     "reads or greps as a current statement."),
    ("R12", "No executable environment specification",
     "fixed", ["requirements.txt", "py/check_environment.py"],
     "Tiers report which analyses each unlocks, and flag MATLAB installed but "
     "off PATH -- the check that originally failed."),
    ("R13", "README reproduction order should lead with integrity checks",
     "fixed", ["README.md"],
     "check_environment -> verify_confirm -> review_status -> tests -> analyses."),
]

KEY_HASHES = ["py/ttc.py", "py/recovery.py", "py/path_progress.py",
              "py/sfm_explore.py", "archive/confirm_code/py/ttc.py",
              "archive/confirm_code/py/verify_confirm.py"]


def locate(rel):
    """Resolve an evidence path in BOTH the repository and the review bundle.

    Documents live beside the repo (`../resubmission/`) in the working tree but
    inside `docs/` once packaged. Without this the status generator reports
    every document as missing when run from an extracted archive -- which is
    exactly where a reviewer runs it.
    """
    cands = [ROOT / rel]
    name = Path(rel).name
    for d in (ROOT.parent / "docs", ROOT.parent.parent / "docs",
              ROOT / "docs", ROOT.parent):
        cands.append(d / name)
    for c in cands:
        try:
            if c.resolve().exists():
                return c.resolve()
        except OSError:
            pass
    return None


def sha(rel):
    p = locate(rel)
    if p is None:
        return None
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def main():
    findings = []
    for fid, desc, status, ev, note in FINDINGS:
        present = {e: locate(e) is not None for e in ev}
        # A finding cannot be better than its evidence.
        effective = status
        if status in ("fixed", "partial") and not all(present.values()):
            effective = "UNSUPPORTED — evidence file missing"
        findings.append(dict(id=fid, finding=desc, claimed_status=status,
                             effective_status=effective, note=note,
                             evidence={e: {"present": present[e], "sha256": sha(e)}
                                       for e in ev}))

    rec = dict(
        generated_from="repository state, not transcribed",
        gate="Stage A calibration sweep is BLOCKED until every finding is fixed",
        key_hashes={k: sha(k) for k in KEY_HASHES},
        counts={s: sum(1 for f in findings if f["claimed_status"] == s)
                for s in ("fixed", "partial", "open")},
        findings=findings,
    )
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(rec, indent=1))

    print(f"wrote {OUT}")
    print(f"\n  {'id':<5}{'status':<10}{'effective':<34}finding")
    print("  " + "-" * 96)
    for f in findings:
        print(f"  {f['id']:<5}{f['claimed_status']:<10}"
              f"{f['effective_status'][:32]:<34}{f['finding'][:44]}")
    c = rec["counts"]
    print(f"\n  fixed {c['fixed']}   partial {c['partial']}   open {c['open']}")
    bad = [f["id"] for f in findings if "UNSUPPORTED" in f["effective_status"]]
    if bad:
        print(f"  UNSUPPORTED CLAIMS: {bad}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
