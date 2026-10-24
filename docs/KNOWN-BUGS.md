# ARS — known bugs, for future attacking

**What this is**: defects that are understood well enough to describe and not worth fixing now. They are **not** in the backlog ladder (that is `AGENTS-BACKLOG.md`, which ranks work by size) and they are **not** open questions (that is `docs/OPEN-ITEMS.md`). This file is for the long game: a behaviour someone has seen, with the shape it takes and what would make it diagnosable when it is finally attacked.

**Read this when**: a session is playing with the area a bug lives in anyway, or a report arrives that might be one of these.

---

## `first-corner-eagerness` — low-grip cars over-commit to the first corner of a race

**Reported 2026-10-24, driver, while testing the apex line at a 15 mph speed floor.**

> Low-grip vehicles may be a bit too eager to line up for the very first corner of a race. High-speed corners or high-grip cars do not see this issue. If you see it — report back in detail.

**The shape**: off the line, a low-grip car starts asking for the outside earlier and more strongly than its speed justifies, so the whole field of them looks like it is diving for the outer edge on the way to the first corner.

**What is already known**:
- The apex line's window is **300 m of route** plus the `IdealLineSpeedMarginMph` speed floor. The floor is `apex speed − 15 mph`, and a low-grip corner has a low apex speed, so its floor is low: a 40 km/h apex asks for the line from 10 km/h.
- `ApexSpeedWithDownforce` includes the per-tick downforce bonus but **not** the low-grip penalty in the same way, so a 1 G car's apex speed and its actual arrival speed can differ more than a 2 G car's.
- Nothing in the line's gate looks at **race phase**: the first corner after a standing start is treated exactly like the fortieth lap.

**What to look for when attacking it**:
1. Whether the offending cars are below their apex speed on the gate or just above it — the `Corner line` log line prints `wants` and `d`, not the speed, so the first job is to add the speed to that line.
2. Whether it is the *line* asking or the **lean** (yellow): with `lineActive` false the lean takes the lane, and the lean has no speed floor at all.
3. Whether the first corner is special only because the field is bunched, in which case the real gripe is the avoidance lane fighting the apex lane rather than the apex lane itself.

## `apex-stays-referenced` — the adopted apex keeps being named after the car passes it

**Noted 2026-10-24 by the driver, not fixed, not to be changed yet.**

**The shape**: `_lineCornerNode` is only replaced when a *new* corner is adopted. `NearestApexAhead` only offers corners with
`DetectedRadius < 50 m` and `metres >= 0`, so when there is no valid corner ahead — the last tight corner of a lap followed by a long
section without one — it returns `null`, nothing replaces the node, and everything that reads `LineCornerNode` keeps naming an apex that
is already behind the car.

**Why it has not hurt yet**: the lane itself is unaffected. A null corner makes `ComputeCornerLane` return 0 and the chain falls through to
the lean, so no demand is built from the stale reference. What is stale is the *reference*, not the driving — the white debug diagonal still
points at the passed apex, which is how it was noticed.

**When it starts to matter**: the moment anything trusts `LineCornerNode` for more than drawing — a card, a brake point, a
commit decision. Then a stale node is a wrong corner.

**How to attack it**: either clear `_lineCornerNode` when its apex is passed *and* `NearestApexAhead` returns null, or give
`NearestApexAhead` an explicit release so the stickiness ends with the pass rather than with the next adoption. Falsify with the white
diagonal after the last apex of a lap: it should vanish, and today it does not.
