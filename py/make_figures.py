"""Result figures, built only from FROZEN archived results.

Run: python3 py/make_figures.py

Every figure here draws on a completed, archived study whose numbers cannot
change: the confirmatory run, the transition decomposition, the dose-response
sweep, Amendment E, and Amendment H. Figures for Amendments F and G are
deliberately absent until those studies exist -- a placeholder figure invites
someone to fill it in from a draft result.

Covers the corrected force laws, outcome composition, transitions,
dose-response, cross-model comparison and matched-seed trajectories.
"""
from __future__ import annotations

import json
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Params, Scenario, run_trial
from reporting_stats import stable_seed, reporting_ci

R = Path("v2/runs/v2")
FIG = Path("../paper/figures")
FIG.mkdir(exist_ok=True)
# MATLAB default look: boxed axes, inward ticks, light grid, Helvetica-like
# sans, and MATLAB's default colour order. NO in-figure titles -- publication figures
# carry their text in the caption, and a title duplicated there wastes column
# height and reads as a slide.
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"],
    "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8,
    "legend.fontsize": 7, "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
    "axes.grid": True, "grid.alpha": 0.15, "grid.linestyle": "-",
    "grid.color": "#26282a", "grid.linewidth": 0.5,
    "axes.axisbelow": True, "axes.edgecolor": "#26282a", "axes.linewidth": 0.5,
    "xtick.direction": "in", "ytick.direction": "in",
    "xtick.top": True, "ytick.right": True,
    "xtick.major.width": 0.5, "ytick.major.width": 0.5,
    "xtick.minor.width": 0.5, "ytick.minor.width": 0.5,
    "legend.frameon": True, "legend.edgecolor": "#26282a",
    "legend.framealpha": 1.0, "legend.fancybox": False,
    "figure.dpi": 200, "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})
# MATLAB R2014b+ default colour order.
M_BLUE, M_ORANGE, M_YELLOW = "#0072BD", "#D95319", "#EDB120"
M_PURPLE, M_GREEN, M_CYAN, M_RED = "#7E2F8E", "#77AC30", "#4DBEEE", "#A2142F"
BLUE, RED, GREY, GREEN = M_BLUE, M_RED, "#4d4d4d", M_GREEN


def save(fig, name):
    fig.savefig(FIG / f"{name}.pdf")
    fig.savefig(FIG / f"{name}.png")
    plt.close(fig)
    print(f"  {name}")


def cat(r):
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    return "deadlock" if r["outcome"] == "deadlock" else "other"


# --- 1. corrected force laws (reviewer 1.5) ----------------------------------
def fig_forces():
    """Corrected force laws. NO title: the claim belongs in the caption.

    The earlier title said "monotonically decreasing, maximal at contact",
    which is not literally true over the plotted range -- the exponential keeps
    rising left of r_i+r_j, inside physical overlap. It is maximal at contact
    only over the NON-OVERLAP admissible domain, so the overlap region is now
    shaded and excluded rather than silently included.
    """
    P = Params()
    r_sum = P.rRobot + P.rPed
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.5))

    d = np.linspace(0.001, 3.0, 800)
    ped = P.aPed * np.exp((r_sum - d) / P.bPed)
    total = ped + P.kBody * np.maximum(0.0, r_sum - d)
    adm = d >= r_sum
    ax[0].axvspan(0, r_sum, color="#d9d9d9", alpha=0.55, lw=0, zorder=0)
    ax[0].plot(d[adm], ped[adm], color=M_BLUE, lw=1.2, label="social repulsion")
    ax[0].plot(d[~adm], total[~adm], color=M_ORANGE, lw=1.2,
               label="total force in overlap")
    # Same lw and the SAME explicit dash pattern for both reference lines.
    # With ls="--" matplotlib scales the dash period by linewidth, so 0.8 and
    # 0.9 rendered differently in the legend. DASH is in points, unscaled.
    DASH = (3.0, 2.0)
    ax[0].axvline(r_sum, color="#26282a", lw=0.9, ls=(0, DASH),
                  label=r"contact threshold, $d=r_i+r_j$")
    ax[0].axhline(P.v0Robot / P.tau, color=M_GREEN, lw=0.9, ls=(0, DASH),
                  label=r"goal acceleration $v_0/\tau$")
    ax[0].plot([r_sum], [P.aPed], "o", color=M_BLUE, ms=3.5, zorder=5)
    ax[0].set(xlabel=r"centre distance $d$ (m)",
              ylabel=r"magnitude (m s$^{-2}$)", ylim=(0, 12), xlim=(0, 3))
    _lg = ax[0].legend(loc="upper right", fontsize=5.5, borderpad=0.4,
                       labelspacing=0.35, handlelength=8.0 / 5.5,
                       handletextpad=0.5, borderaxespad=0.4)
    _lg.get_frame().set_linewidth(0.4)
    _lg.get_frame().set_edgecolor("#9a9a9a")

    do = np.linspace(0.001, 1.5, 600)
    ax[1].plot(do, P.aObs * np.exp(-do / P.bObs), color=M_BLUE, lw=1.2,
               label="obstacle repulsion")
    ax[1].set(xlabel=r"surface distance $d$ (m)",
              ylabel=r"magnitude (m s$^{-2}$)", xlim=(0, 1.5))
    _lg1 = ax[1].legend(fontsize=5.5, borderpad=0.4, labelspacing=0.35,
                        handlelength=1.6, handletextpad=0.5, borderaxespad=0.4)
    _lg1.get_frame().set_linewidth(0.4)
    _lg1.get_frame().set_edgecolor("#9a9a9a")

    # Figure 1 only: grey panel frames instead of near-black. Tick marks are
    # matched to the frame so they do not read as heavier than the box they
    # sit on. The dashed contact-threshold line stays dark: it is data, not
    # chrome.
    for a in ax:
        for sp in a.spines.values():
            sp.set_edgecolor("#8c8c8c")
        a.tick_params(color="#8c8c8c", which="both")

    save(fig, "fig1_force_laws")


# --- 2. outcomes by arm ------------------------------------------------------
def fig_outcomes():
    """Outcome composition. `other` is renamed `timeout`, which is what the
    taxonomy actually contains. Every segment is labelled, including small
    ones, so no category is silently invisible.

    Categories are a single-hue blue ramp, dark to light in stack order. A
    sequential ramp is used rather than four unrelated hues because the
    categories are ordered by severity, and it survives greyscale printing.
    """
    rows = json.load(open(R / "confirm_factorial.json"))
    by = defaultdict(list)
    for r in rows:
        if not r.get("failed"):
            by[r["arm"]].append(cat(r))
    arms = ["sfm", "sfm-ttc", "hybrid", "hybrid-ttc"]
    disp = ["SFM", "SFM-TTC", "A*-SFM", "A*-SFM-TTC"]
    cats = ["safe", "unsafe", "deadlock", "other"]
    show = {"safe": "safe", "unsafe": "unsafe arrival",
            "deadlock": "deadlock", "other": "timeout"}
    # Dark -> light, bottom -> top. The ramp is deliberately kept in the
    # mid-to-light range so that ONE ink colour is legible on every shade and
    # on the white page: a full dark-to-pale ramp would force white text on the
    # dark end and dark text on the pale end, which is what we are avoiding.
    cols = {"safe": "#5da5d1", "unsafe": "#8ec1e0",
            "deadlock": "#b8d8ec", "other": "#dcebf7"}
    INK = "#26282a"          # one label colour everywhere

    fig, ax = plt.subplots(figsize=(5.0, 2.6))
    bottom = np.zeros(len(arms))
    BAR_W = 0.6
    SMALL = 6.0          # below this a segment cannot hold its own label
    for c in cats:
        v = np.array([100 * np.mean([x == c for x in by[a]]) for a in arms])
        # no white separator between segments: the ramp already separates them
        ax.bar(disp, v, bottom=bottom, color=cols[c], label=show[c],
               width=BAR_W, linewidth=0)
        for i_, (b, h) in enumerate(zip(bottom, v)):
            if h <= 0:
                continue
            if h > SMALL:
                # fits inside: centre it in the segment
                ax.annotate(f"{h:.1f}%", (i_, b + h / 2), ha="center",
                            va="center", fontsize=6, color=INK)
            # Segments below SMALL carry no label: at this axes height they are
            # 2-6 pt tall, so any label has to sit outside the bar, and five of
            # those read as clutter. The exact values for every segment are in
            # Table I directly below, so nothing is lost.
        bottom += v
    ax.set(ylabel="% of 500 scenarios", ylim=(0, 100))
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(top=False, right=False, which="both")
    _lg = ax.legend(ncol=4, loc="lower center", bbox_to_anchor=(0.5, -0.30),
                    borderpad=0.4, handlelength=1.4, handletextpad=0.5,
                    columnspacing=1.4)
    _lg.get_frame().set_linewidth(0.4)
    _lg.get_frame().set_edgecolor("#9a9a9a")
    save(fig, "fig2_outcomes_by_arm")


# --- 3. transition decomposition --------------------------------------------
def fig_transitions():
    """Paired transition matrix, guided arm with and without the filter.

    The `other` (timeout) row and column are dropped: neither guided arm timed
    out in any of the 500 scenarios, so that category is identically zero and
    would occupy a quarter of the panel as an empty band. Its absence is stated
    in the caption so the reader knows the category exists and was empty, not
    that it was omitted from the taxonomy.
    """
    rows = json.load(open(R / "confirm_factorial.json"))
    by = defaultdict(dict)
    for r in rows:
        if not r.get("failed"):
            by[r["scenario"]][r["arm"]] = r
    comp = [s for s in by if len(by[s]) == 4]
    cats = ["safe", "unsafe", "deadlock", "other"]
    M = np.zeros((4, 4))
    for s in comp:
        M[cats.index(cat(by[s]["hybrid"])), cats.index(cat(by[s]["hybrid-ttc"]))] += 1

    # keep only categories that actually occur on either axis
    keep = [i for i in range(4) if M[i, :].sum() or M[:, i].sum()]
    M = M[np.ix_(keep, keep)]
    labels = [cats[i] for i in keep]
    n = len(labels)

    fig, ax = plt.subplots(figsize=(3.9, 3.4))
    im = ax.imshow(M, cmap="Blues")
    for i in range(n):
        for j in range(n):
            if M[i, j]:
                ax.text(j, i, int(M[i, j]), ha="center", va="center", fontsize=8,
                        color="white" if M[i, j] > M.max() / 2 else "black")
    ax.set(xticks=range(n), yticks=range(n), xticklabels=labels,
           yticklabels=labels, xlabel="with TTC", ylabel="without TTC")
    ax.grid(False)

    # lighter chrome: thinner frame on both the panel and the colour bar
    for sp in ax.spines.values():
        sp.set_linewidth(0.4)
    ax.tick_params(width=0.4, which="both")
    cb = fig.colorbar(im, ax=ax, shrink=0.8, label="scenarios")
    cb.outline.set_linewidth(0.4)
    cb.ax.tick_params(width=0.4)
    save(fig, "fig3_transition_matrix")


# --- 4. dose-response --------------------------------------------------------
def fig_dose():
    """One-factor-at-a-time sweep. Units on axes, tested levels as the only
    ticks, and the shared frozen default marked -- the centre point is common
    to all three panels, which is invisible otherwise."""
    rows = json.load(open(R / "dose_response.json"))
    by = defaultdict(lambda: defaultdict(dict))
    for r in rows:
        by[r["planner"]][r["config"]][r["scenario"]] = r
    scens = sorted(by["hybrid"]["no-filter"])
    LEV = {"ttcHorizon": ([1.0, 1.5, 2.0, 3.0], "s"),
           "ttcStop": ([0.4, 0.8, 1.2], "s"),
           "ttcMinGap": ([0.20, 0.40, 0.60], "m")}
    DEF = {"ttcHorizon": 2.0, "ttcStop": 0.8, "ttcMinGap": 0.40}
    fig, axes = plt.subplots(1, 3, figsize=(7.0, 2.3), sharey=True)
    for ax, (f, (lv, unit)) in zip(axes, LEV.items()):
        for key, col, lab in (("safe", M_GREEN, "safe success (95% CI)"),
                              ("deadlock", M_ORANGE, "deadlock (95% CI)")):
            m, lo, hi = [], [], []
            for x in lv:
                c = "ttcHorizon=2.0" if x == DEF[f] else f"{f}={x}"
                v = np.array([by["hybrid"][c][s]["category"] == key for s in scens],
                             float)
                g = np.random.default_rng(stable_seed((f, x, key)))
                bs = v[g.integers(0, len(v), size=(2000, len(v)))].mean(axis=1)
                m.append(v.mean()); lo.append(np.percentile(bs, 2.5))
                hi.append(np.percentile(bs, 97.5))
            ax.plot(lv, m, "o-", color=col, ms=3, lw=1.1, label=lab)
            ax.fill_between(lv, lo, hi, color=col, alpha=0.16, lw=0)
        ax.axvline(DEF[f], color="#26282a", lw=0.7, ls=(0, (3.0, 2.0)))
        ax.set(xlabel=f"{f} ({unit})", xticks=lv)
        ax.set_xticklabels([f"{x:g}" for x in lv])
        # panel frame kept lighter than the legend frame
        for sp in ax.spines.values():
            sp.set_linewidth(0.3)
        ax.tick_params(width=0.3, which="both")
    axes[0].set_ylabel("rate")
    # The dashed rule and the shaded band are explained in the legend rather
    # than by a rotated in-plot label, which collided with the data.
    from matplotlib.lines import Line2D
    h, l = axes[0].get_legend_handles_labels()
    # Matplotlib SCALES dash patterns by linewidth (rcParams lines.scale_dashes),
    # so a (3,2) pattern at lw=0.7 is really (2.1,1.4) on the page. The legend
    # handle must be a whole number of those SCALED periods or the final dash is
    # sliced off mid-stroke. Two dashes plus the gap between them:
    DLW, DASH, GAP, FS = 0.7, 3.0, 2.0, 5.5
    handle_pt = 2 * DASH * DLW + GAP * DLW          # dash + gap + dash
    h += [Line2D([], [], color="#26282a", lw=DLW, ls=(0, (DASH, GAP)))]
    l += ["frozen default"]
    _lg = axes[0].legend(h, l, loc="center left", fontsize=FS, borderpad=0.35,
                         labelspacing=0.3, handlelength=handle_pt / FS,
                         handletextpad=0.5, borderaxespad=0.4)
    _lg.get_frame().set_linewidth(0.4)
    _lg.get_frame().set_edgecolor("#9a9a9a")
    save(fig, "fig4_dose_response")


# --- 5. pedestrian model comparison (E + H) ---------------------------------
def fig_models():
    """TTC effect across pedestrian-RESPONSE settings, not "models".

    Endpoints are NOT interchangeable: the custom simulator scores unsafe
    ARRIVAL, CrowdNav scores collision INCIDENCE. Directionally analogous,
    numerically distinct -- the caption must say so.

    The baseline unsafe/collision rate is annotated under each setting because
    it carries the ceiling mechanism: where little is left to rescue, only the
    mobility cost remains. Without it the reversal looks unexplained.
    """
    conf = defaultdict(dict)
    for r in json.load(open(R / "confirm_factorial.json")):
        if not r.get("failed"):
            conf[r["scenario"]][r["arm"]] = r
    rec = defaultdict(dict)
    for r in json.load(open(R / "reciprocal.json")):
        if r["set"] == "confirm" and r["responsibility"] == "reciprocal":
            rec[r["scenario"]][r["arm"]] = r
    cn = json.load(open(R / "crowdnav_transfer.json"))

    ci = reporting_ci

    def dpair(pairs, a, b, key, tag):
        c = lambda r: r["category"] if "category" in r else cat(r)
        v = [(c(p[b]) == key) - (c(p[a]) == key)
             for p in pairs.values() if a in p and b in p]
        base = np.mean([c(p[a]) == key for p in pairs.values() if a in p and b in p])
        return ci(v, tag) + (base,)

    def dcn(vis, key, tag):
        p = defaultdict(dict)
        for r in cn:
            if r["dt"] == 0.25 and r["visible"] == vis:
                p[(r["rule"], r["n_human"], r["seed"])][r["arm"]] = r
        pr = [v for v in p.values() if len(v) == 2]
        v = [float(x["ttc"][key]) - float(x["reference"][key]) for x in pr]
        base = np.mean([float(x["reference"][key]) for x in pr])
        return ci(v, tag) + (base,)

    settings = [
        ("Custom sim\nSFM pedestrians", dpair(conf, "hybrid", "hybrid-ttc", "unsafe", "x1"),
         dpair(conf, "hybrid", "hybrid-ttc", "deadlock", "x2")),
        ("Custom sim\nORCA-responsive", dpair(rec, "disabled", "ttc0.40", "unsafe", "x3"),
         dpair(rec, "disabled", "ttc0.40", "deadlock", "x4")),
        ("CrowdNav (0.25 s)\nrobot invisible", dcn(False, "collision", "x5"),
         dcn(False, "deadlock", "x6")),
        ("CrowdNav (0.25 s)\nrobot visible", dcn(True, "collision", "x7"),
         dcn(True, "deadlock", "x8")),
    ]
    x = np.arange(4); w = 0.34
    fig, ax = plt.subplots(figsize=(6.2, 2.9))
    for k, (off, col, lab) in enumerate(((-w / 2, M_RED, "unsafe arrival / collision"),
                                         (+w / 2, M_ORANGE, "deadlock"))):
        m = [s[1 + k][0] for s in settings]
        lo = [s[1 + k][0] - s[1 + k][1] for s in settings]
        hi = [s[1 + k][2] - s[1 + k][0] for s in settings]
        ax.bar(x + off, m, w, color=col, label=lab, edgecolor="#26282a", lw=0.4)
        ax.errorbar(x + off, m, yerr=[lo, hi], fmt="none", ecolor="#26282a",
                    elinewidth=0.8, capsize=2.2)
    ax.axhline(0, color="#26282a", lw=0.8)
    # The baseline is appended as a third line of the tick label so it shares
    # the tick labels' font, size and colour rather than being a separate
    # annotation in a different size.
    xlabels = [f"{s[0]}\nbaseline {100 * s[1][3]:.0f}%" for s in settings]
    ax.set(xticks=x, xticklabels=xlabels,
           ylabel=r"$\Delta$ rate (TTC - no TTC)", ylim=(-0.34, 0.50))
    _lg = ax.legend(loc="upper left", ncol=2, fontsize=6, borderpad=0.35,
                    labelspacing=0.3, handlelength=1.4, handletextpad=0.5,
                    columnspacing=1.2)
    _lg.get_frame().set_linewidth(0.4)
    _lg.get_frame().set_edgecolor("#9a9a9a")
    save(fig, "fig5_across_settings")


# --- 6. trajectories (reviewer 1.32 / 1.33) ---------------------------------
def fig_trajectories():
    """Two panels, answering two different questions.

    (a) reviewer 1.32: SFM trapped vs hybrid escaping the SAME region.
    (b) the exposure finding: hybrid and hybrid-TTC follow essentially the same
        route, so a plain overlay hides one line under the other and reads as a
        plotting error. The separation-vs-time inset is what actually shows it:
        the paths coincide in space while diverging in time.
    """
    conf = defaultdict(dict)
    for r in json.load(open(R / "confirm_factorial.json")):
        if not r.get("failed"):
            conf[r["scenario"]][r["arm"]] = r
    trapped = [s for s, v in conf.items() if len(v) == 4
               and cat(v["sfm"]) == "deadlock" and cat(v["hybrid"]) in ("safe", "unsafe")]
    adverse = [s for s, v in conf.items() if len(v) == 4
               and cat(v["hybrid"]) == "safe" and cat(v["hybrid-ttc"]) == "unsafe"]
    if not (trapped and adverse):
        print("  (required scenario types absent; skipping)")
        return

    fig, axes = plt.subplots(1, 2, figsize=(8.4, 4.2))
    for ax, scen, arms, cols, labs, title in (
            # "trap" is deliberately avoided: the trap-analysis thread was
            # retracted and the word now carries methodological baggage.
            (axes[0], sorted(trapped)[0], ("sfm", "hybrid"), (M_RED, M_BLUE),
             ("SFM only (deadlock)", "A*-SFM (reaches goal)"),
             "(a) A*-SFM reaches the goal where direct-goal SFM deadlocks"),
            (axes[1], sorted(adverse)[0], ("hybrid", "hybrid-ttc"), (M_BLUE, M_RED),
             ("A*-SFM (no TTC)", "A*-SFM-TTC"),
             "(b) Same geometric route, different temporal position")):
        p = R / "export_confirm" / f"{scen}.json"
        if not p.exists():
            continue
        S = Scenario.load(str(p))
        for o in S.obstacles:
            ax.fill(o[:, 0], o[:, 1], color="#B08628", ec="#7d5f1c", lw=0.6,
                    zorder=1)
        if S.waypoints is not None:
            ax.plot(S.waypoints[0], S.waypoints[1], "s-", color=M_GREEN,
                    ms=2.0, lw=0.7, label="A* waypoints", zorder=2)
        runs = []
        for arm, col, lab in zip(arms, cols, labs):
            Rn = run_trial(Scenario.load(str(p)), "sfm" if arm == "sfm" else "hybrid",
                           log=True, ttc=arm.endswith("ttc"))
            runs.append(Rn)
            # Blue is drawn slightly wider than red, and red on top, so where
            # the paths coincide exactly (panel b) blue still reads as an
            # outline instead of vanishing under red.
            ax.plot(Rn.traj[0], Rn.traj[1], color=col,
                    lw=1.0 if col == BLUE else 0.6, ls="-", alpha=0.95,
                    label=lab, zorder=3 if col == BLUE else 4)
        ax.plot(*S.robot_start, "o", color="black", ms=4.5, zorder=6)
        ax.plot(S.params.goal[0], S.params.goal[1], "o", color="black", ms=4.5,
                zorder=6)
        # Mark where the deadlocking run stops -- otherwise the reader must
        # infer it from where a line happens to end.
        if arms[0] == "sfm":
            ax.plot(runs[0].traj[0, -1], runs[0].traj[1, -1], "o", color=M_RED,
                    ms=5.5, mew=0, zorder=7, label="deadlock")
        ax.set(xlabel="x (m)", ylabel="y (m)", aspect="equal")
        # Panel LETTER sits OUTSIDE the frame. The descriptive text belongs in
        # the caption.
        ax.text(-0.02, 1.03, title[:3], transform=ax.transAxes, fontsize=8,
                fontweight="bold", va="bottom", ha="left")
        _lg = ax.legend(fontsize=5.5, loc="lower right", borderpad=0.35,
                        labelspacing=0.3, handlelength=1.5, handletextpad=0.5)
        _lg.get_frame().set_linewidth(0.4)
        _lg.get_frame().set_edgecolor("#9a9a9a")
        # panel frame kept lighter than the legend frame
        for sp in ax.spines.values():
            sp.set_linewidth(0.3)
        ax.tick_params(width=0.3, which="both")

        if title.startswith("(b)"):
            a, b = runs
            n = min(a.traj.shape[1], b.traj.shape[1])
            sep = np.hypot(a.traj[0, :n] - b.traj[0, :n], a.traj[1, :n] - b.traj[1, :n])
            # x0 nudged right so the inset's y-axis label clears the parent
            # panel's own y-axis rather than sitting on top of it.
            ins = ax.inset_axes([0.105, 0.70, 0.30, 0.24])
            ins.plot(np.arange(n) * 0.01, sep, color="#8e44ad", lw=0.9)
            ins.set(xlabel="t (s)", ylabel="separation (m)")
            ins.tick_params(labelsize=5, width=0.4)
            ins.xaxis.label.set_size(5.5); ins.yaxis.label.set_size(5.5)
            for sp in ins.spines.values():
                sp.set_linewidth(0.4)
                sp.set_edgecolor("#9a9a9a")
            ins.grid(False)
            # The two summary numbers previously printed here overlapped the
            # trajectories; they are stated in the caption instead.

    save(fig, "fig6_trajectories")


if __name__ == "__main__":
    print("figures ->", FIG)
    for f in (fig_forces, fig_outcomes, fig_transitions, fig_dose,
              fig_models, fig_trajectories):
        try:
            f()
        except Exception as e:
            print(f"  FAILED {f.__name__}: {type(e).__name__}: {e}")
