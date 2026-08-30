"""Seal the confirmatory record: hashes every input, script and output.

Run after all confirmatory and exploratory analyses complete. Produces a
single manifest so the record can be verified later without trusting any
narrative description of it.
"""
import hashlib, json, subprocess
from pathlib import Path

def h(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

ARCH = {
  "sealed": "2026-07-31",
  "study": "Confirmatory: does TTC filtering improve safe-goal success under A* guidance?",
  "protocol": "resubmission/CONFIRMATORY_PROTOCOL.md",
  "chronology": [
    "1. 500 scenarios generated from the frozen seed list (sha256 fc590516...)",
    "2. 2000 runs executed once; 0 failed",
    "3. Integrity gate FAILED on a stale expected hash for analyze_confirm.py",
    "4. On-disk file verified against the already-dated Amendment B; matched",
    "5. NO result file was opened or analysed before that verification",
    "6. Stale environment record corrected, reason logged",
    "7. Gate re-run and PASSED",
    "8. Frozen primary analysis executed",
    "9. Secondary, transition and mediator analyses executed in the fixed order",
    "10. Post-outcome exploratory analyses run and labelled as such",
  ],
  "gate_note": ("The gate did not pass on first execution. It detected a stale "
                "record, which is the behaviour intended. Described this way "
                "rather than as an unqualified pass."),
}

groups = {
  "inputs": ["v2/runs/v2/confirm_seeds.json", "v2/runs/v2/confirm_manifest.json",
             "v2/runs/v2/confirm_environment.json"],
  "confirmatory_executables": ["py/sfm.py", "py/ttc.py", "py/run_confirm.py",
                               "v2/SfmParams.m", "v2/RunTrial.m",
                               "v2/ComputeForces.m", "v2/MakeScenario.m",
                               "v2/Export_ConfirmScenarios.m"],
  "frozen_analysis": ["py/verify_confirm.py", "py/analyze_confirm.py"],
  "pre_outcome_exploratory": ["py/transition_table.py"],
  "post_outcome_exploratory": ["py/transition_temporal.py", "py/temporal_ordering.py",
                               "py/group_uncertainty.py", "py/sfm_explore.py"],
  "raw_results": ["v2/runs/v2/confirm_factorial.json"],
  "outputs": ["v2/runs/v2/confirm_analysis_output.txt",
              "v2/runs/v2/confirm_transition_output.txt",
              "v2/runs/v2/confirm_temporal_output.txt"],
}
ARCH["hashes"] = {}
for g, fs in groups.items():
    ARCH["hashes"][g] = {f: h(f) for f in fs if Path(f).exists()}

ARCH["headline"] = {
  "primary_delta": 0.0780,
  "primary_ci": [0.0340, 0.1220],
  "primary_p": 7.559e-4,
  "discordant": "84/45",
  "threshold": 0.075,
  "branch": ("positive; interval spans the prespecified meaningful threshold; "
             "insufficient evidence the benefit reaches that magnitude"),
  "unsafe_arrival_delta": -0.198,
  "deadlock_delta": +0.120,
  "goal_arrival_delta": -0.120,
  "ttc_unguided_delta": -0.004,
  "interaction_delta": 0.082,
  "interaction_ci": [0.034, 0.130],
  "interaction_development": 0.087,
}
Path("v2/runs/v2/CONFIRMATORY_ARCHIVE.json").write_text(json.dumps(ARCH, indent=1))
n = sum(len(v) for v in ARCH["hashes"].values())
print(f"sealed {n} files across {len(ARCH['hashes'])} groups")
for g, fs in ARCH["hashes"].items():
    print(f"  {g:<28} {len(fs)} files")
print("\nwrote v2/runs/v2/CONFIRMATORY_ARCHIVE.json")
