"""Lane ask surviving the crossing share, by speed, mirroring Racer.cs.

Reproduces LaneCrossingThresholdDegrees and LaneCrossingShare with no corner gate (the share stands
down while the corner line owns the lane, so every curve here is the non-corner case):

    threshold(v) = degrees(asin(min(LaneCrossingLateralSpeedMps / v, 1)))
    share(c, v)  = threshold / (threshold + |c|)

The allowance is a softness scale rather than a boundary: the ask keeps half its authority at the
allowance, a third at twice it, and never reaches nothing — a car well off its lane at speed is
slowed down rather than forbidden. Only the ask that would cross the car further is scaled; the ask
that unwinds a crossing is left alone, and corner entry is exempt in the code (not drawn here).

Top panel: how much of the lane ask survives at a given crossing angle (the car's heading against
the track direction), with every dot on the half-authority line by construction. Bottom panel: the
same angles read as sideways motion, where every allowance lands on the same sideways rate — which
is why the allowance follows 1/speed.

Run: python docs/lane-crossing-share.py
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
TOP_LIMIT = 1.45
BOTTOM_LIMIT = 14.0


def threshold_deg(speed_mph):
    v = speed_mph * MPH
    return math.degrees(math.asin(min(LATERAL_MPS / v, 1.0)))


def share(crossing_deg, speed_mph):
    t = threshold_deg(speed_mph)
    return t / (t + np.abs(crossing_deg))


crossings = np.linspace(0.0, X_MAX, 700)

fig, (top, bottom) = plt.subplots(2, 1, sharex=True, figsize=(10.5, 8.6), dpi=150)

for mph, colour in zip(SPEEDS, COLOURS):
    t = threshold_deg(mph)
    top.plot(crossings, share(crossings, mph), "-", color=colour, linewidth=2.0, label=f"{mph} mph  (half at {t:.2f} deg)")
    top.plot([t], [0.5], "o", color=colour, markersize=5)
    bottom.plot(crossings, (mph * MPH) * np.sin(np.radians(crossings)), "-", color=colour, linewidth=2.0, label=f"{mph} mph")
    bottom.plot([t], [LATERAL_MPS], "o", color=colour, markersize=5)

top.axhline(0.5, color="#999999", linewidth=0.8, linestyle=":", zorder=0)
top.text(0.2, 0.53, "half authority — every dot sits here, which is what the allowance scales", ha="left", va="bottom", fontsize=9, color="#555555")
top.set_ylim(0.0, TOP_LIMIT)
top.set_ylabel("share of the lane ask that survives")
top.set_title(
    "Lane ask surviving the crossing share\n"
    f"allowance {LATERAL_MPS:.2f} m/s sideways as a softness scale, corner gate open (no corner owns the lane)",
    fontsize=11,
)
top.grid(alpha=0.25, linewidth=0.6)
top.legend(loc="upper center", ncol=2, fontsize=8.5, framealpha=0.95)

bottom.axhline(LATERAL_MPS, color="#333333", linewidth=1.0, linestyle="--", zorder=0)
bottom.text(X_MAX - 0.2, LATERAL_MPS - 0.35, f"the allowance: {LATERAL_MPS:.2f} m/s sideways", ha="right", va="top", fontsize=9, color="#333333")
bottom.text(0.2, BOTTOM_LIMIT - 0.4, "every dot sits on that sideways rate,\nwhich is why the allowance follows 1/speed:\nthe same sideways motion at every speed", ha="left", va="top", fontsize=9, color="#555555")
bottom.set_ylim(0.0, BOTTOM_LIMIT)
bottom.set_xlim(0.0, X_MAX)
bottom.set_xlabel("crossing angle: the car's heading against the track direction (deg)")
bottom.set_ylabel("sideways motion (m/s)")
bottom.grid(alpha=0.25, linewidth=0.6)

fig.tight_layout()
fig.savefig("docs/lane-crossing-share.png", facecolor="white")

print(f"{'mph':>5} {'half at':>8} {'2 deg':>7} {'5 deg':>7} {'10 deg':>7}   (share of the ask)")
for mph in SPEEDS:
    print(f"{mph:>5} {threshold_deg(mph):>8.2f} {share(2.0, mph):>7.2f} {share(5.0, mph):>7.2f} {share(10.0, mph):>7.2f}")
