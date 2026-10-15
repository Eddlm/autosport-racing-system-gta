# Question: how does ARS's brake cap compare with the engine's two ABS flavours?
# Run: py "docs\abs-comparison.py"   (writes docs\abs-comparison.png and prints the numbers)
#
# All three are policies over the same quantity: the per-wheel longitudinal slip ratio the tyre
# publishes at wheel+0x174, where 0.4 is the traction peak and 1.0 is the flat point (the curve
# is indexed at field x sfTractionMinAngle with its peak at argument 1.0, wheel.cpp:4465/4497).
#
#   ours (b11e52b)      a proportional cap on the brake pedal: 1.00 at the peak, floored at 0.25
#                       one span later at twice the peak (Racer.cs, AbsKneeSlip / AbsFloorSlip).
#   vanilla standard    WF_ABS, the clamp every fleet car carries via MF_ABS_STD: the modelled
#                       brake slip is held at sfBrakeRatioLimitAbs x sfTractionMinAngleInv = 1.5 x
#                       0.4 = 0.6 while above 5 m/s (wheel.cpp:4580-4599, :4142-4146). The pedal
#                       is untouched.
#   vanilla alternative WF_ABS_ALT (MF_ABS_ALT_STD, on none of the 41 fleet cars): nothing until
#                       the wheel is already locked (slip > 0.9), then a full brake release for
#                       sfBrakeAbsTimer = 62.5 ms once the slip passes sfBrakeAbsLimit = 1.5
#                       (wheel.cpp:5268-5277, released at :3915).
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PEAK, FLAT = 0.4, 1.0          # in the field: sfTractionPeakAngle / sfTractionMinAngle, and 1.0
MAX, MIN = 1.5, 1.3            # a fleet car's fTractionCurveMax / fTractionCurveMin

KNEE, SPAN, FLOOR = 0.4, 0.4, 0.25     # ours: AbsKneeSlip, the span, AbsBrakeFloor
VANILLA_STD_CLAMP = 1.5 * 0.4          # sfBrakeRatioLimitAbs x sfTractionMinAngleInv
VANILLA_ALT_LOCK = 0.9                 # the engine's own skid/lock line, in the field
VANILLA_ALT_RELEASE = 1.5              # sfBrakeAbsLimit, compared against the raw field


def coefficient(slip):
    argument = slip / PEAK
    if argument < 1.0:
        return MAX * argument
    if argument < 2.5:
        return MAX + (MIN - MAX) * (argument - 1.0) / 2.5
    return MIN


def ours(slip):
    if slip <= KNEE:
        return 1.0
    share = min(1.0, math.tanh((slip - KNEE) / SPAN) / math.tanh(1.0))
    return 1.0 - share * (1.0 - FLOOR)


def ours_old(slip):
    knee, floor_slip, floor = 2.5, 5.0, 0.25
    if slip <= knee:
        return 1.0
    share = min(1.0, math.tanh((slip - knee) / (floor_slip - knee)) / math.tanh(1.0))
    return 1.0 - share * (1.0 - floor)


for slip in (PEAK, VANILLA_STD_CLAMP, KNEE + SPAN, FLAT, 1.5):
    print("slip %.2f (%3.0f%% of the peak): traction %5.2f = %3.0f%% of the peak%s" % (
        slip, 100.0 * slip / PEAK, coefficient(slip), 100.0 * coefficient(slip) / MAX,
        "  <- ours floors the brake here" if abs(slip - (KNEE + SPAN)) < 1e-9 else ""))

figure, (left, right) = plt.subplots(1, 2, figsize=(13.6, 5.4))

slips = [i * 1.6 / 400.0 for i in range(401)]
left.plot(slips, [coefficient(s) / MAX for s in slips], color="#1a1a1a", linewidth=2.4,
          label="the engine's traction curve (fleet max/min 1.50/1.30)")
left.axvspan(KNEE, KNEE + SPAN, color="#2f6fb0", alpha=0.16)
left.axvline(VANILLA_STD_CLAMP, color="#c2543a", linewidth=2.0, linestyle="--",
             label="vanilla standard holds the slip here (0.6)")
left.axvline(VANILLA_ALT_LOCK, color="#7a7a7a", linewidth=1.6, linestyle=":",
             label="vanilla alternative: locked from 0.9")
left.axvline(VANILLA_ALT_RELEASE, color="#8c8c8c", linewidth=1.6, linestyle="-.",
             label="vanilla alternative releases past 1.5")
left.text(0.505, 0.845,
          "ours: 100% of peak force at the knee (0.4)\n"
          "      95% still there where the pedal floors (0.8)\n"
          "vanilla standard: 97% at 0.6, pedal untouched\n"
          "flat point at 1.0 (87%) — the step is the engine's own denominator",
          fontsize=8.6, va="bottom", ha="left",
          bbox=dict(boxstyle="round,pad=0.4", facecolor="white", edgecolor="#bbbbbb", alpha=0.92))
left.set_title("Where each scheme leaves the tyre")
left.set_xlabel("longitudinal slip ratio, wheel+0x174 (0.4 = the peak, 1.0 = flat)")
left.set_ylabel("traction, share of the tyre's peak")
left.set_ylim(0.83, 1.03)
left.set_xlim(0.0, 1.6)
left.grid(alpha=0.25)
left.legend(loc="upper right", fontsize=8.4)

right.plot(slips, [ours(s) for s in slips], color="#2f6fb0", linewidth=2.6,
           label="ours (b11e52b): a pedal cap, knee 0.4, floor 0.8 at 25%")
right.plot(slips, [ours_old(s) for s in slips], color="#2f6fb0", linewidth=1.4, linestyle="-.",
           label="ours before the re-scale: knee 2.5, floor 5.0 (never engaged)")
right.plot(slips, [1.0 for s in slips], color="#c2543a", linewidth=2.0,
           label="vanilla standard: pedal untouched, the slip is clamped")
right.plot(slips, [1.0 for s in slips], color="#7a7a7a", linewidth=1.8, linestyle=":",
           label="vanilla alternative: pedal untouched, 62.5 ms full release")
right.plot([VANILLA_STD_CLAMP, VANILLA_STD_CLAMP], [0.0, 1.0], color="#c2543a", linewidth=2.0, linestyle="--")
right.plot([VANILLA_ALT_RELEASE, VANILLA_ALT_RELEASE], [0.0, 1.0], color="#8c8c8c", linewidth=1.6, linestyle="-.")
right.set_title("What each scheme does to the brake pedal")
right.set_xlabel("longitudinal slip ratio, wheel+0x174")
right.set_ylabel("brake force left to the wheel, share of the demand")
right.set_ylim(-0.05, 1.08)
right.set_xlim(0.0, 1.6)
right.grid(alpha=0.25)
right.legend(loc="lower left", fontsize=8.4)

figure.suptitle("ARS's brake cap against the engine's two ABS flavours", fontsize=12)
figure.tight_layout(rect=(0, 0, 1, 0.94))
figure.savefig("docs/abs-comparison.png", dpi=140)
print("wrote docs/abs-comparison.png")
