"""Steer-in ceiling by speed, one governor, mirroring Racer.cs.

Reproduces ApplySteerLimits with no damper term on the commanded side:
    ceiling = ResolveSteerCeiling
    ResolveSteerCeiling = max(peakSlipCeiling, ManeuverRamp(endSpeed, peakSlipCeilingAt(endSpeed)))

The yaw-usage governor and its Turn-In dials were removed earlier, and the slide's raise is retired by decision and
off behind `SlideLimitRaise`, so the peak-slip cap is the only governor left. The raise is still drawn, dashed and
greyed, as the reference the drive rejected: it used to take max(that ceiling, min(|slide| x share + freeplay, lock)).

Assumes the fleet-typical car: lock 40 deg, authored LateralTractionCurve 22 deg, no damper bypass.

Run: python docs/steer-ceiling.py
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOCK = 40.0
TRLAT = 22.0
# The engine scales one front wheel to this share of the steer command (TEMP_STEER_WHEEL_MULT, CWheel::SetSteerAngle,
# wheel.cpp:6445), so the ceiling is stated at the midpoint of the two front wheels, as Racer.cs does.
WHEEL_OUTER_SHARE = 0.75
PEAK_SHARE = 2.0 / (1.0 + WHEEL_OUTER_SHARE)
RAMP_START, RAMP_END = 5.0, 15.0
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


def retired_raise(slide_deg):
    """The raise as it stood while it was live: the ceiling loosened by half the slide angle plus free play."""
    return [max(resolve_steer_ceiling(s), min(abs(slide_deg) * SLIDE_SHARE + FREE_PLAY, LOCK)) for s in SPEEDS]


live = [resolve_steer_ceiling(s) for s in SPEEDS]
curves = [
    ("live ceiling: the cornering law alone", live, "#1f4e9c", "-", 2.4),
    ("retired raise, 20 deg slide", retired_raise(20.0), "#2e8b57", "--", 1.6),
    ("retired raise, 30 deg slide", retired_raise(30.0), "#c0392b", "--", 1.6),
]

fig, ax = plt.subplots(figsize=(10.5, 6.2), dpi=150)
ax.axvspan(RAMP_START, RAMP_END, color="#000000", alpha=0.05, zorder=0)
ax.text((RAMP_START + RAMP_END) / 2, 41.4, f"maneuver ramp band ({RAMP_START:.0f}-{RAMP_END:.0f} mph)", ha="center", va="bottom", fontsize=8.5, color="#555555")

for label, ys, colour, style, width in curves:
    ax.plot(SPEEDS, ys, style, color=colour, linewidth=width, label=label)

ax.axvline(RAMP_END, color="#555555", linewidth=0.8, alpha=0.6, zorder=0)
ax.text(RAMP_END + 0.5, 21.0, f"ramp ends:\n{RAMP_END:.0f} mph", fontsize=8.5, color="#555555", va="bottom")
ax.text(31.0, 25.5, "dashed: the raise, retired by decision\n(a slide used to open authority above the law)", fontsize=8.5, color="#777777", va="bottom")

marks = [(40, retired_raise(30.0)[80], "#c0392b", "raise, retired", (7, 8)),
         (40, live[80], "#1f4e9c", "live", (7, -18))]
for x, y, colour, tag, offset in marks:
    ax.plot([x], [y], "o", color=colour, markersize=5)
    ax.annotate(f"{tag}: {y:.1f} deg", (x, y), textcoords="offset points", xytext=offset, fontsize=9, color=colour)

ax.set_xlim(0, 50)
ax.set_ylim(0, 46)
ax.set_xlabel("forward speed (mph)")
ax.set_ylabel("maximum steer-in allowed (deg)")
ax.set_title("Steer-in ceiling by speed, one governor\nfleet-typical car: 40 deg lock, 22 deg authored TRlat", fontsize=11)
ax.grid(alpha=0.25, linewidth=0.6)
ax.legend(loc="upper right", fontsize=9, framealpha=0.95)
fig.tight_layout()
fig.savefig("docs/steer-ceiling.png", facecolor="white")

print(f"{'mph':>5} {'live':>9} {'raise 20':>9} {'raise 30':>9}")
for s in range(0, 51, 5):
    print(f"{s:>5} {live[s * 2]:>9.2f} {retired_raise(20.0)[s * 2]:>9.2f} {retired_raise(30.0)[s * 2]:>9.2f}")
