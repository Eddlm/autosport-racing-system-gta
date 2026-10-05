"""Steer-in ceiling by speed, one governor, mirroring Racer.cs.

Reproduces ApplySteerLimits with no damper term on the commanded side:
    ceiling = max(ResolveSteerCeiling, min(|slide| x SlideLimitSlideShare + SlideLimitFreeplayDegrees, lock))
    ResolveSteerCeiling = max(peakSlipCeiling, ManeuverRamp(endSpeed, peakSlipCeilingAt(endSpeed)))

Assumes the fleet-typical car: lock 40 deg, authored LateralTractionCurve 22 deg, no damper bypass.
The yaw-usage governor and its Turn-In dials were removed: the slide is the only quantity that opens authority.

Run: python docs/steer-ceiling.py
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOCK = 40.0
TRLAT = 22.0
PEAK_SHARE = 1.0 / 0.75
RAMP_START, RAMP_END = 5.0, 30.0
SLIDE_SHARE = 0.5
FREE_PLAY = 0.5
MPH = 0.44704
SPEEDS = [i * 0.5 for i in range(0, 101)]


def trlat_at_speed(v):
    return TRLAT / (1.0 + min(5.0, 0.1 * v))


def peak_ceiling(v):
    return min(trlat_at_speed(v) * PEAK_SHARE, LOCK)


def maneuver_ramp(mph, ceiling):
    if mph >= RAMP_END:
        return ceiling
    f = max(0.0, min(1.0, (RAMP_END - mph) / (RAMP_END - RAMP_START)))
    return ceiling + f * (LOCK - ceiling)


def resolve_steer_ceiling(speed_mph):
    v = speed_mph * MPH
    ceiling = peak_ceiling(v)
    if speed_mph >= RAMP_END:
        return ceiling
    return max(ceiling, maneuver_ramp(speed_mph, peak_ceiling(RAMP_END * MPH)))


def ceiling(slide_deg):
    return [max(resolve_steer_ceiling(s), min(abs(slide_deg) * SLIDE_SHARE + FREE_PLAY, LOCK)) for s in SPEEDS]


curves = [
    ("cornering law alone (no slide)", ceiling(0.0), "#1f4e9c", "-", 2.4),
    ("20 deg slide", ceiling(20.0), "#2e8b57", "--", 1.8),
    ("30 deg slide", ceiling(30.0), "#c0392b", "-", 2.2),
]

fig, ax = plt.subplots(figsize=(10.5, 6.2), dpi=150)
ax.axvspan(RAMP_START, RAMP_END, color="#000000", alpha=0.05, zorder=0)
ax.text(17.5, 41.4, "maneuver ramp band (5-30 mph)", ha="center", va="bottom", fontsize=8.5, color="#555555")

for label, ys, colour, style, width in curves:
    ax.plot(SPEEDS, ys, style, color=colour, linewidth=width, label=label)

ax.axvline(RAMP_END, color="#555555", linewidth=0.8, alpha=0.6, zorder=0)
ax.text(30.5, 21.0, "ramp ends:\n30 mph", fontsize=8.5, color="#555555", va="bottom")
ax.text(31.0, 25.5, "a slide opens authority above the law:\n~24 deg of slide passes it at 30 mph,\n~20 deg at 40 mph", fontsize=8.5, color="#c0392b", va="bottom")

marks = [(40, ceiling(30.0)[80], "#c0392b", (7, -4)),
         (40, ceiling(0.0)[80], "#1f4e9c", (7, -16))]
for x, y, colour, offset in marks:
    ax.plot([x], [y], "o", color=colour, markersize=5)
    ax.annotate(f"{y:.1f} deg", (x, y), textcoords="offset points", xytext=offset, fontsize=9, color=colour)

ax.set_xlim(0, 50)
ax.set_ylim(0, 46)
ax.set_xlabel("forward speed (mph)")
ax.set_ylabel("maximum steer-in allowed (deg)")
ax.set_title("Steer-in ceiling by speed, one governor\nfleet-typical car: 40 deg lock, 22 deg authored TRlat", fontsize=11)
ax.grid(alpha=0.25, linewidth=0.6)
ax.legend(loc="upper right", fontsize=9, framealpha=0.95)
fig.tight_layout()
fig.savefig("docs/steer-ceiling.png", facecolor="white")

print(f"{'mph':>5} {'law':>9} {'slide 20':>9} {'slide 30':>9}")
for s in range(0, 51, 5):
    print(f"{s:>5} {ceiling(0.0)[s * 2]:>9.2f} {ceiling(20.0)[s * 2]:>9.2f} {ceiling(30.0)[s * 2]:>9.2f}")
