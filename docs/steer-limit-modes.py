"""Steer-in ceiling by speed, both governors, mirroring Racer.cs.

Reproduces ApplySteerLimits with no slide and no damper term on the commanded side:
    Yaw-Governed   = min(ResolveSteerCeiling, maneuverRamp(min(base x yawShare, lock)))
    Slide-Governed = max(ResolveSteerCeiling, min(half the slide + 0.5, lock))   # additive since 467, halved in 468

Assumes the fleet-typical car: lock 40 deg, authored LateralTractionCurve 22 deg,
Turn-In Minimum 20%, Turn-In Maximum 100%, no yaw, no damper bypass.

Run: python docs/steer-limit-modes.py
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
TURN_IN_MIN, TURN_IN_MAX = 0.20, 1.00
MPH = 0.44704
SPEEDS = [i * 0.5 for i in range(0, 101)]


def trlat_at_speed(v):
    return TRLAT / (1.0 + min(5.0, 0.1 * v))


def peak_ceiling(v):
    return min(trlat_at_speed(v) * PEAK_SHARE, LOCK)


def maneuver_ramp(mph, ceiling):
    if mph >= RAMP_END:
        return ceiling
    f = (RAMP_END - mph) / (RAMP_END - RAMP_START)
    f = max(0.0, min(1.0, f))
    return ceiling + f * (LOCK - ceiling)


def resolve_steer_ceiling(v):
    ceiling = peak_ceiling(v)
    mph = v / MPH
    if mph >= RAMP_END:
        return ceiling
    return max(ceiling, maneuver_ramp(mph, peak_ceiling(RAMP_END * MPH)))


def yaw_governed(mph, yaw_usage):
    v = mph * MPH
    if v <= 0.0:
        return LOCK
    base = resolve_steer_ceiling(v)
    share = TURN_IN_MIN + (TURN_IN_MAX - TURN_IN_MIN) * yaw_usage
    return min(base, maneuver_ramp(mph, min(base * share, LOCK)))


def slide_governed(mph, slide_deg=0.0):
    v = mph * MPH
    if v <= 0.0:
        return LOCK
    return max(resolve_steer_ceiling(v), min(abs(slide_deg) * SLIDE_SHARE + FREE_PLAY, LOCK))


def slide_replacement(mph, slide_deg=0.0):
    v = mph * MPH
    if v <= 0.0:
        return LOCK
    ceiling = min(abs(slide_deg) + 1.0, LOCK)
    return max(ceiling, maneuver_ramp(mph, ceiling))


cornering = [resolve_steer_ceiling(s * MPH) for s in SPEEDS]

curves = [
    ("Yaw-Governed, no yaw (share 20%)", [yaw_governed(s, 0.0) for s in SPEEDS], "#1f4e9c", "-", 2.4),
    ("Cornering law — Yaw at full share, Slide with no slide", cornering, "#1f4e9c", "--", 1.8),
    ("Slide-Governed, 30 deg slide", [slide_governed(s, 30.0) for s in SPEEDS], "#c0392b", "-", 2.2),
    ("Slide-Governed as a replacement (before 467)", [slide_replacement(s) for s in SPEEDS], "#888888", ":", 1.6),
]

fig, ax = plt.subplots(figsize=(10.5, 6.2), dpi=150)
ax.axvspan(RAMP_START, RAMP_END, color="#000000", alpha=0.05, zorder=0)
ax.text(17.5, 41.4, "maneuver ramp band (5-30 mph)", ha="center", va="bottom", fontsize=8.5, color="#555555")

for label, ys, colour, style, width in curves:
    ax.plot(SPEEDS, ys, style, color=colour, linewidth=width, label=label)

ax.axvline(RAMP_END, color="#555555", linewidth=0.8, alpha=0.6, zorder=0)
ax.text(30.5, 21.0, "ramp ends:\n30 mph", fontsize=8.5, color="#555555", va="bottom")
ax.text(31.0, 24.5, "halved in 468:\nthe addition needs a ~20 deg\nslide to pass the law above 30 mph", fontsize=8.5, color="#c0392b", va="bottom")

marks = [(30, yaw_governed(30, 0.0), "#1f4e9c", (7, 7), "yaw cap"),
         (30, slide_replacement(30), "#888888", (7, -16), "slide replaced")]
for x, y, colour, offset, _ in marks:
    ax.plot([x], [y], "o", color=colour, markersize=5)
    ax.annotate(f"{y:.1f} deg", (x, y), textcoords="offset points", xytext=offset, fontsize=9, color=colour)

ax.set_xlim(0, 50)
ax.set_ylim(0, 46)
ax.set_xlabel("forward speed (mph)")
ax.set_ylabel("maximum steer-in allowed (deg)")
ax.set_title("Steer-in ceiling by speed, both governors\nfleet-typical car: 40 deg lock, 22 deg authored TRlat, Turn-In Min 20% / Max 100%", fontsize=11)
ax.grid(alpha=0.25, linewidth=0.6)
ax.legend(loc="upper right", fontsize=9, framealpha=0.95)
fig.tight_layout()
fig.savefig("docs/steer-limit-modes.png", facecolor="white")

print(f"{'mph':>5} {'yaw 20%':>9} {'cornering':>10} {'slide 0':>9} {'slide 30':>9}")
for s in range(0, 51, 5):
    print(f"{s:>5} {yaw_governed(s, 0.0):>9.2f} {resolve_steer_ceiling(s * MPH):>10.2f} {slide_governed(s):>9.2f} {slide_governed(s, 30.0):>9.2f}")
