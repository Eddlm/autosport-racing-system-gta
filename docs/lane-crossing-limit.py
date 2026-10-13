"""Lane ask surviving the crossing limit, by speed, mirroring Racer.cs.

Reproduces LaneCrossingThresholdDegrees and the step at the call site, with no corner gate (the
limit stands down while the corner line owns the lane, so every curve here is the non-corner case):

    threshold(v) = degrees(asin(min(LaneCrossingLateralSpeedMps / v, 1)))
    the ask that would cross the car further is dropped outright past that threshold

The threshold is the heading that carries the car sideways at the lateral allowance, so it is
`1/speed` by construction. Past it the ask is dropped as a step rather than tapered, so there is
no width to tune; the ask that unwinds a crossing is never dropped, which is the asymmetry the
step keeps, and corner entry is exempt in the code (not drawn here).

Top panel: how much of the lane ask survives at a given crossing angle (the car's heading
against the track direction). Bottom panel: the same angles read as sideways motion, where every
threshold lands on the allowance line - that is why the threshold follows 1/speed.

Run: python docs/lane-crossing-limit.py
"""
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

LATERAL_MPS = 1.0  # LaneCrossingLateralSpeedMps
MPH = 0.44704
SPEEDS = [10, 20, 30, 40, 50, 60, 80, 120]
COLOURS = ["#1f4e9c", "#2e8b57", "#c0392b", "#8e44ad", "#d68910", "#16a085", "#34495e", "#7f8c8d"]
X_MAX = 14.0
TOP_LIMIT = 1.55
BOTTOM_LIMIT = 14.0


def threshold_deg(speed_mph):
    v = speed_mph * MPH
    return math.degrees(math.asin(min(LATERAL_MPS / v, 1.0)))


def share(crossing_deg, speed_mph):
    return np.where(np.abs(crossing_deg) <= threshold_deg(speed_mph), 1.0, 0.0)


crossings = np.linspace(0.0, X_MAX, 1401)

fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(10.5, 8.6), dpi=150)

for mph, colour in zip(SPEEDS, COLOURS):
    t = threshold_deg(mph)
    top.plot(crossings, share(crossings, mph), "-", color=colour, linewidth=2.0, label=f"{mph} mph  (stops at {t:.2f} deg)")
    top.plot([t], [1.0], "o", color=colour, markersize=5)
    bottom.plot(crossings, (mph * MPH) * np.sin(np.radians(crossings)), "-", color=colour, linewidth=2.0, label=f"{mph} mph")
    bottom.plot([t], [LATERAL_MPS], "o", color=colour, markersize=5)

t50 = threshold_deg(50)
top.annotate("the ask stops here — a step, no taper",
             xy=(t50, 0.5), xytext=(t50 + 1.1, 0.62),
             arrowprops=dict(arrowstyle="->", color="#333333", linewidth=1.1), fontsize=9, color="#333333")
top.text(0.25, 0.32, "dots: the threshold, where the ask stops\nbelow it the ask is untouched", fontsize=9, color="#555555", va="top")
top.set_ylim(0.0, TOP_LIMIT)
top.set_ylabel("share of the lane ask that survives")
top.set_title(
    "Lane ask surviving the crossing limit\n"
    f"allowance {LATERAL_MPS:.2f} m/s sideways, no taper, corner gate open (no corner owns the lane)",
    fontsize=11,
)
top.grid(alpha=0.25, linewidth=0.6)
top.legend(loc="upper center", ncol=2, fontsize=8.5, framealpha=0.95)

bottom.axhline(LATERAL_MPS, color="#333333", linewidth=1.0, linestyle="--", zorder=0)
bottom.text(X_MAX - 0.2, LATERAL_MPS - 0.35, f"the allowance: {LATERAL_MPS:.2f} m/s sideways", ha="right", va="top", fontsize=9, color="#333333")
bottom.text(0.2, BOTTOM_LIMIT - 0.4, "every threshold sits on that allowance line,\nwhich is why it follows 1/speed:\nthe same sideways rate at every speed", ha="left", va="top", fontsize=9, color="#555555")
bottom.set_ylim(0.0, BOTTOM_LIMIT)
bottom.set_xlim(0.0, X_MAX)
bottom.set_xlabel("crossing angle: the car's heading against the track direction (deg)")
bottom.set_ylabel("sideways motion (m/s)")
bottom.grid(alpha=0.25, linewidth=0.6)

fig.tight_layout()
fig.savefig("docs/lane-crossing-limit.png", facecolor="white")

print(f"{'mph':>5} {'threshold':>10}   (deg of crossing, the ask stops there)")
for mph in SPEEDS:
    print(f"{mph:>5} {threshold_deg(mph):>10.2f}")
