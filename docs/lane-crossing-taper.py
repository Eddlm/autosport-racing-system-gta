"""Lane ask surviving the crossing taper, by speed, mirroring Racer.cs.

Reproduces LaneCrossingThresholdDegrees and LaneCrossingTaperShare with no corner gate (the
taper is skipped while the corner line owns the lane; every curve here is the non-corner case):

    threshold(v)  = degrees(asin(min(LaneCrossingLateralSpeedMps / v, 1)))
    share(c, v)   = 0.5 * (1 - tanh((c - threshold - span/2) / (span/2)))

The threshold is the heading that carries the car sideways at the lateral allowance, so it is
`1/speed` by construction; the span is fixed, which is what holds the wheel up once speed has
collapsed the threshold. Below the threshold the same expression is ~1, so there is no branch.

Top panel: how much of the lane ask survives at a given crossing angle (the car's heading
against the track direction). Bottom panel: the same angles read as sideways motion, where
every threshold lands on the allowance line - that is why the threshold follows 1/speed.

Run: python docs/lane-crossing-taper.py
"""
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

LATERAL_MPS = 2.0  # LaneCrossingLateralSpeedMps
SPAN_DEG = 2.0     # LaneCrossingTaperSpanDegrees
MPH = 0.44704
SPEEDS = [10, 20, 40, 60, 80, 120]
COLOURS = ["#1f4e9c", "#2e8b57", "#c0392b", "#8e44ad", "#d68910", "#34495e"]

TOP_LIMIT = 1.5


def threshold_deg(speed_mph):
    v = speed_mph * MPH
    return math.degrees(math.asin(min(LATERAL_MPS / v, 1.0)))


def share(crossing_deg, speed_mph):
    t = threshold_deg(speed_mph)
    half = SPAN_DEG / 2.0
    return 0.5 * (1.0 - np.tanh((crossing_deg - t - half) / half))


crossings = np.linspace(0.0, 14.0, 400)

fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(10.5, 8.6), dpi=150)

for mph, colour in zip(SPEEDS, COLOURS):
    t = threshold_deg(mph)
    top.plot(crossings, share(crossings, mph), "-", color=colour, linewidth=2.2, label=f"{mph} mph  (starts at {t:.1f} deg)")
    top.plot([t], [1.0], "o", color=colour, markersize=5)
    bottom.plot(crossings, (mph * MPH) * np.sin(np.radians(crossings)), "-", color=colour, linewidth=2.2, label=f"{mph} mph")
    bottom.plot([t], [LATERAL_MPS], "o", color=colour, markersize=5)

t40 = threshold_deg(40)
top.annotate("", xy=(t40, 0.55), xytext=(t40 + SPAN_DEG, 0.55), arrowprops=dict(arrowstyle="<->", color="#333333", linewidth=1.2))
top.text(t40 + SPAN_DEG / 2.0, 0.58, f"span {SPAN_DEG:.0f} deg", ha="center", va="bottom", fontsize=9, color="#333333")
top.axhline(0.5, color="#999999", linewidth=0.8, linestyle=":", zorder=0)
top.text(0.25, 0.53, "half authority", ha="left", va="bottom", fontsize=8.5, color="#777777")
top.text(0.25, 0.30, "dots: the threshold, where the reduction starts\nbelow it the ask is untouched", fontsize=9, color="#555555", va="top")
top.set_ylim(0.0, TOP_LIMIT)
top.set_ylabel("share of the lane ask that survives")
top.set_title(
    "Lane ask surviving the crossing taper\n"
    f"allowance {LATERAL_MPS:.0f} m/s sideways, span {SPAN_DEG:.0f} deg fixed, corner gate open (no corner owns the lane)",
    fontsize=11,
)
top.grid(alpha=0.25, linewidth=0.6)
top.legend(loc="upper center", ncol=3, fontsize=8.5, framealpha=0.95)

bottom.axhline(LATERAL_MPS, color="#333333", linewidth=1.0, linestyle="--", zorder=0)
bottom.text(13.8, LATERAL_MPS + 0.35, f"the allowance: {LATERAL_MPS:.0f} m/s sideways", ha="right", va="bottom", fontsize=9, color="#333333")
bottom.text(13.8, 1.35, "every threshold sits on that line\nby construction, so it follows 1/speed", ha="right", va="top", fontsize=9, color="#555555")
bottom.set_ylim(0.0, 13.0)
bottom.set_xlim(0.0, 14.0)
bottom.set_xlabel("crossing angle: the car's heading against the track direction (deg)")
bottom.set_ylabel("sideways motion (m/s)")
bottom.grid(alpha=0.25, linewidth=0.6)

fig.tight_layout()
fig.savefig("docs/lane-crossing-taper.png", facecolor="white")

print(f"{'mph':>5} {'threshold':>10} {'half':>7} {'spent':>7}   (deg of crossing)")
for mph in SPEEDS:
    t = threshold_deg(mph)
    print(f"{mph:>5} {t:>10.2f} {t + SPAN_DEG / 2.0:>7.2f} {t + SPAN_DEG:>7.2f}")
