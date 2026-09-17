# Archived source as executed for the confirmatory study

These are the EXACT sources that produced the frozen confirmatory results. They
are historical artefacts and must never be edited.

`py/ttc.py` — sha256 prefix `b89dcad9104b6db9`, the hash recorded in the frozen
protocol.

The development head `py/ttc.py` has since gained `ttc_brake_ex`, which exposes
the TTC layer's mandatory-intervention flags for the Amendment F recovery
controller. That change is behaviour-preserving for `ttc_brake` — proven over
4000 randomised states by `py/test_ttc_result_compatibility.py` — but a
compatibility view preserves BEHAVIOUR, not FILE PROVENANCE.

`verify_confirm.py` therefore verifies THIS archived source, not the current
head. The manifest is correct about what ran and is not rewritten.

`v2/SfmParams.m` — sha256 prefix `aa7d0f2329cedfa4`, the hash recorded in the
frozen protocol.

Commit `d3a25a6` (30 August 2026, reviewer-item closure) commented out the
declaration `P.rGoalSweep = [0.25 0.5 1.0]`, which was labelled a "reported
sensitivity band" but was referenced by no script — reviewer 1.13 is answered
instead by re-scoring the frozen Phase 0 outputs (`py/analyze_rgoal_sensitivity.py`).
Removing an unreferenced declaration cannot change behaviour, but it changes the
file, and a hash gate is a provenance test, not a behaviour test.

That commit archived no frozen copy, so the confirmatory gate reported a
mismatch from 30 August until this copy was restored from `d3a25a6^`. The
restored file is byte-identical to the manifest hash. As with `py/ttc.py`, the
manifest is correct about what ran and is NOT rewritten; the gate is pointed at
this archived copy instead.
