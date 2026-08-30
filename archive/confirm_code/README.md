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
