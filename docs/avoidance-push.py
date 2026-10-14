"""The contact-avoidance push: what it asks for against what the tyre grants.

Mirrors the term as built in `Racer.cs`'s `ComputeSteering`:
    angle  = atan(relative lateral speed / this car's forward speed)   # the pair's relative course
    ramp   = clamp((outerGap - surfaceGap) / outerGap, 0, 1)           # daylight between the two sides
    offset = angle * ramp
and the ceiling it is added under, `ResolveSteerCeiling`'s peak-slip cap:
    grant  = LateralTractionCurve / (1 + min(5, 0.1 * speed)) * 2 / (1 + outerWheelShare)

The question it answers: is the push still visible, and where does it reach the grant? The old law is not drawn —
it was a closing-rate remap onto 15 deg times a distance scale, eight times stronger at a one metre per second
squeeze. Surface gap is between the bodies, so zero means the sides touching.

Assumes the fleet-typical car: lock 40 deg, authored LateralTractionCurve 22 deg, ceiling factor 1.00.

Run: python docs/avoidance-push.py
"""
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOCK = 40.0
TRLAT = 22.0
WHEEL_OUTER_SHARE = 0.75
PEAK_SHARE = 2.0 / (1.0 + WHEEL_OUTER_SHARE)
CEILING_FACTOR = 1.0
RAMP_END_MPH = 15.0
RAMP_START_MPH = 5.0

OUTER_GAP = 5.0                     # ContactPushOuterGapMeters
SQUEEZES = [0.5, 1.0, 2.0, 5.0]     # relative lateral speed, m/s
PANELS_MPH = [134.0, 45.0]
MPH_TO_MPS = 0.44704


def grant_deg(speed_mps):
    ceiling = TRLAT / (1.0 + min(5.0, 0.1 * speed_mps)) * PEAK_SHARE * CEILING_FACTOR
    mph = speed_mps / MPH_TO_MPS
    if mph >= RAMP_END_MPH:
        return ceiling
    if mph <= RAMP_START_MPH:
        return LOCK
    t = (RAMP_END_MPH - mph) / (RAMP_END_MPH - RAMP_START_MPH)
    return max(ceiling, ceiling + (LOCK - ceiling) * t)


def offset_deg(lateral_mps, speed_mps, gap_m):
    angle = math.degrees(math.atan2(lateral_mps, speed_mps))
    ramp = max(0.0, min(1.0, (OUTER_GAP - gap_m) / OUTER_GAP))
    return angle * ramp


fig, axes = plt.subplots(1, 2, figsize=(12.4, 4.4))
gaps = [i * OUTER_GAP / 200.0 for i in range(201)]

for ax, mph in zip(axes, PANELS_MPH):
    speed = mph * MPH_TO_MPS
    for lateral in SQUEEZES:
        ax.plot(gaps, [offset_deg(lateral, speed, g) for g in gaps], lw=1.8,
                label="%g m/s lateral closure" % lateral)
    ceiling = grant_deg(speed)
    ax.axhline(ceiling, color="black", ls="--", lw=1.3,
               label="tyre's grant at this speed, %.1f deg" % ceiling)
    ax.set_title("at %.0f mph" % mph)
    ax.set_xlabel("daylight between the two bodies (m)")
    ax.set_xlim(0, OUTER_GAP)
    ax.set_ylim(0, max(6.0, ceiling * 1.35))
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc="upper right")
    ax.annotate("the whole angle\nwhen the sides touch", xy=(0.0, offset_deg(max(SQUEEZES), speed, 0.0)),
                xytext=(0.55, max(6.0, ceiling * 1.35) * 0.62), fontsize=8,
                arrowprops=dict(arrowstyle="->", lw=0.8))
    ax.annotate("nothing at five\nmetres of daylight", xy=(OUTER_GAP, 0.0),
                xytext=(OUTER_GAP * 0.35, max(6.0, ceiling * 1.35) * 0.12), fontsize=8,
                arrowprops=dict(arrowstyle="->", lw=0.8))

axes[0].set_ylabel("steer offset asked for (degrees of road wheel)")
fig.suptitle("The contact-avoidance push: the pair's relative course angle, ramped by the daylight between them",
             fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.94])
fig.savefig("docs/avoidance-push.png", dpi=110)
print("wrote docs/avoidance-push.png")
