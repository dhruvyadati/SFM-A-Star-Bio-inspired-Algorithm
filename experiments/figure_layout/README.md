# Saved-result figure layout

Run `python experiments/figure_layout/plot.py` from the repository root with NumPy
and Matplotlib installed. It loads the existing plotting definitions, relocates
input/output directories, and changes only labels, legends and layout for the
force-law and parameter-response plots. Simulation sources and saved results
are unchanged. Numeric curves and bootstrap calculations use the existing
functions. Outputs are under `figures/` next to this script.
