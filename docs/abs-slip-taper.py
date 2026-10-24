import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Slip-ratio axis, as the engine's own curve is indexed: 0 free rolling, 0.4 the traction
# peak (sfTractionPeakAngle x sfTractionMinAngleInv), 1.0 fully locked and flat beyond.
PEAK, LOCK = 0.4, 1.0

def new_law(slip):
    """Shipped in build 882: knee 0.5, straight to 0.0 at the lock point."""
    if slip <= 0.5:
        return 1.0
    return max(0.0, 1.0 - (slip - 0.5) / (LOCK - 0.5))

def old_law(slip):
    """The taper retired in 7ccaac7: knee at the peak, span 0.4, floor 0.25."""
    if slip <= PEAK:
        return 1.0
    span = slip - PEAK
    return max(0.25, 1.0 - span / 0.4 * (1.0 - 0.25))

xs = [i / 200 * 1.6 for i in range(201)]
new = [new_law(x) for x in xs]
old = [old_law(x) for x in xs]

fig, ax = plt.subplots(figsize=(9, 5.2), dpi=150)
fig.patch.set_facecolor("white")
ax.set_facecolor("white")

ax.plot(xs, new, color="#c2185b", linewidth=2.6, label="shipped (build 882): knee 0.5 -> 0.0 at lock")
ax.plot(xs, old, color="#1565c0", linewidth=2.0, linestyle="--", label="retired taper: knee 0.4, floor 0.25")

ax.axvline(PEAK, color="#2e7d32", linewidth=1.2, linestyle=":")
ax.text(PEAK - 0.015, 1.045, "peak grip  0.4", color="#2e7d32", fontsize=9, ha="right")
ax.axvline(LOCK, color="#616161", linewidth=1.2, linestyle=":")
ax.text(LOCK - 0.015, 1.045, "locked  1.0", color="#616161", fontsize=9, ha="right")

ax.set_xlabel("slip ratio  (0 free rolling, 0.4 peak, 1.0 locked)", color="#212121")
ax.set_ylabel("max brake  (fraction of the plan's pedal)", color="#212121")
ax.set_title("ARS ABS: brake allowed against wheel slip", color="#212121", fontsize=12, loc="left")
ax.set_xlim(0, 1.6)
ax.set_ylim(-0.02, 1.12)
ax.grid(True, color="#e0e0e0", linewidth=0.8)
for spine in ("top", "right"):
    ax.spines[spine].set_visible(False)
for spine in ("left", "bottom"):
    ax.spines[spine].set_color("#9e9e9e")
ax.tick_params(colors="#424242")
ax.legend(frameon=False, loc="lower left", fontsize=9.5)

fig.tight_layout()
out = "docs/abs-slip-taper.png"
fig.savefig(out, facecolor=fig.get_facecolor())
print("escrito:", out)
