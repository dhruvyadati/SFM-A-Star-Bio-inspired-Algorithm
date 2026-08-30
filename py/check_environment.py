"""Environment smoke test. Run: python3 py/check_environment.py

Reports what is present and, crucially, WHICH ANALYSES EACH TIER UNLOCKS, so a
reviewer knows what they can and cannot run before hitting an ImportError.
"""
import importlib
import shutil
import sys
from pathlib import Path

TIERS = [
    ("required", "reproduce all archived SFM analyses",
     [("numpy", "numpy"), ("pytest", "pytest")]),
    ("optional", "ORCA-responsive pedestrian study (Amendment E) only",
     [("rvo2", "pyrvo2")]),
]


def main():
    print(f"python {sys.version.split()[0]}  ({sys.executable})")
    fail = 0
    for tier, unlocks, mods in TIERS:
        print(f"\n{tier.upper()} — {unlocks}")
        for mod, pkg in mods:
            try:
                m = importlib.import_module(mod)
                v = getattr(m, "__version__", "?")
                print(f"  ok       {mod:<10} {v}")
            except ImportError:
                print(f"  MISSING  {mod:<10} (pip install {pkg})")
                if tier == "required":
                    fail += 1

    print("\nMATLAB-ONLY — scenario generation, no toolboxes")
    mat = shutil.which("matlab")
    if not mat:
        for c in Path("/Applications").glob("MATLAB*/bin/matlab"):
            mat = str(c)
            break
    print(f"  {'ok      ' if mat else 'MISSING '} matlab     {mat or 'not found'}")
    if mat and not shutil.which("matlab"):
        print("           (installed but NOT on PATH — `which matlab` will miss it)")

    print("\nDATA")
    for p, what in (("v2/runs/v2/confirm_factorial.json", "confirmatory results"),
                    ("v2/runs/v2/dose_response.json", "dose-response"),
                    ("v2/runs/v2/reciprocal.json", "ORCA-responsive"),
                    ("v2/runs/v2/export_fcal", "F calibration scenarios")):
        print(f"  {'ok      ' if Path(p).exists() else 'absent  '} {what}")

    print("\n" + ("ENVIRONMENT OK for archived analyses" if not fail
                  else f"{fail} REQUIRED dependency missing"))
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
