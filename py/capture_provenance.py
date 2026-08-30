"""Record the execution environment and run sizes for the methods section.

Run: python3 py/capture_provenance.py  ->  v2/runs/v2/provenance.json

Wall-clock times are recorded ONLY where they were actually measured. Runs that
completed before timing was instrumented are marked `null` rather than
estimated -- a plausible-looking number in a methods section is worse than an
honest gap, because a reader cannot tell the two apart.
"""
from __future__ import annotations
import json, platform, subprocess, sys
from pathlib import Path

R = Path("v2/runs/v2")
OUT = R / "provenance.json"

def sh(cmd, default=None):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=60).stdout.strip() or default
    except Exception:
        return default

def versions():
    v = {}
    for m in ("numpy", "scipy", "matplotlib", "rvo2", "gym", "pytest"):
        try:
            v[m] = getattr(__import__(m), "__version__", "unversioned")
        except ImportError:
            v[m] = None
    return v

# Wall-clock timings OBSERVED during execution. Absent entries stay null.
TIMINGS = {
    "dose_response":       {"wall_clock_s": None, "workers": 12,
                            "note": "ran to completion; wall clock not instrumented"},
    "reciprocal":          {"wall_clock_s": 21600, "workers": 12,
                            "note": "approx 6 h observed; ORCA solve dominates"},
    "crowdnav_transfer":   {"wall_clock_s": None, "workers": 10,
                            "note": "wall clock not instrumented"},
    "confirm_factorial":   {"wall_clock_s": None, "workers": None,
                            "note": "predates timing instrumentation"},
    "stage_a":             {"wall_clock_s": None, "workers": 10, "note": ""},
}

STUDIES = [
    ("confirm_factorial.json", "confirmatory factorial", "MATLAB + Python"),
    ("dose_response.json", "dose-response sweep", "Python"),
    ("reciprocal.json", "ORCA-responsive pedestrians", "Python + RVO2"),
    ("crowdnav_transfer.json", "CrowdNav external transfer", "Python + CrowdNav"),
    ("stage_a.json", "recovery Stage A calibration", "Python"),
]

def main():
    rec = {
        "hardware": {
            "cpu": sh("sysctl -n machdep.cpu.brand_string"),
            "logical_cores": int(sh("sysctl -n hw.ncpu", "0")),
            "physical_cores": int(sh("sysctl -n hw.physicalcpu", "0")),
            "ram_gb": round(int(sh("sysctl -n hw.memsize", "0")) / 1073741824),
            "arch": platform.machine(),
        },
        "os": f"{sh('sw_vers -productName')} {sh('sw_vers -productVersion')}",
        "python": platform.python_version(),
        "packages": versions(),
        "matlab": sh('/Applications/MATLAB_R2025b.app/bin/matlab -batch "disp(version)" 2>/dev/null | tail -1'),
        "external": {
            "CrowdNav": {
                "commit": sh("git -C ../external/CrowdNav rev-parse HEAD"),
                "remote": sh("git -C ../external/CrowdNav config --get remote.origin.url"),
            }
        },
        "studies": [],
        "caveat": ("Wall-clock times are given only where measured. Null means "
                   "the run predates timing instrumentation and no estimate is "
                   "substituted."),
    }
    for fn, label, stack in STUDIES:
        p = R / fn
        key = fn.replace(".json", "")
        e = {"study": label, "stack": stack, "file": fn,
             "runs": None, "size_mb": None, "present": p.exists()}
        if p.exists():
            e["runs"] = len(json.load(open(p)))
            e["size_mb"] = round(p.stat().st_size / 1e6, 2)
        e.update(TIMINGS.get(key, {}))
        rec["studies"].append(e)

    tot = sum(s["runs"] or 0 for s in rec["studies"])
    rec["total_simulation_runs"] = tot
    OUT.write_text(json.dumps(rec, indent=1))

    h = rec["hardware"]
    print(f"wrote {OUT}\n")
    print(f"  {h['cpu']}")
    print(f"  {h['physical_cores']} physical / {h['logical_cores']} logical cores, "
          f"{h['ram_gb']} GB, {rec['os']}")
    print(f"  Python {rec['python']}, MATLAB {rec['matlab']}\n")
    print(f"  {'study':<32}{'runs':>7}{'workers':>9}{'wall clock':>12}")
    for s in rec["studies"]:
        w = s.get("wall_clock_s")
        wc = f"{w/3600:.1f} h" if w else "not measured"
        print(f"  {s['study']:<32}{str(s['runs'] or '--'):>7}"
              f"{str(s.get('workers') or '--'):>9}{wc:>12}")
    print(f"\n  total archived simulation runs: {tot}")

if __name__ == "__main__":
    main()
