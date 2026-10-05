# ARS — steering simplification: contract and rulings

**The brief.** After a long run of steering changes — the slew rate, the damper's speed cap and its two subtraction
rules, two steer-limit governors, the maneuvering ramp, the slide-blend re-cut — the chain reads as delicate, and
several decisions during that run were made one gate at a time. The task is to analyse the whole steering chain and
settle a small set of **smart simplifications**: each one either behaviour-preserving or a deliberate, named change
the driver can feel and judge.

## Invariants

- **Analysis does not ship.** Every behavioural change is compile-verified, committed, and then driven by the user,
  who is the only one who can call it working.
- **Simplification means removing a part, a gate, a dial or a duplicate — not re-tuning.** A re-tune is not a
  finding; it belongs in the test plan. "Change this constant" is only a candidate if the constant exists solely
  because of a part being removed.
- **The chain's order is fixed**: track position → target speed → steering → steer limits → pedals → steer slew. A
  candidate may not quietly move a stage.
- **The comment policy binds.** A simplification that lets a comment die is worth naming as such; a comment may only
  be added for an engine citation, a deliberate asymmetry or floor, or a unit the name cannot carry.
- **Verified-by-drive state is evidence, not an obstacle.** Anything already driver-verified (`TEST-PLAN.md`,
  `AGENTS-STEERING.md`) is a behaviour to preserve unless a candidate explicitly proposes changing it, and then the
  proposal has to say so in those words.

## Deliverable and ownership

- **Writer file**: `STEERING-SIMPLIFICATION-FINDINGS.md` — the inventory and the ranked candidates. One writer.
- **Lead file**: this one — the contract, the rulings, and the accepted/rejected record.
- No code is written in this pass. Applying a ruling is a later, separate pass with the driver in the loop.

## Acceptance for the writer's file

1. Every live stage of the steering chain is named with a `file:line` anchor, its input, and what consumes it.
2. Every candidate carries three things: **what it removes**, **what it is expected to cost behaviourally**, and
   **how the driver would know** (the specific on-track symptom to look for).
3. Candidates are ranked by behaviour risk first, then by how much they remove — not by effort.
4. Anything that contradicts `AGENTS-STEERING.md` or `TEST-PLAN.md` is reported as a decision for the Lead, never
   patched into the file's own conclusions.
5. Dead code, write-only fields, and dials with no consumer are listed separately as an inventory line, since those
   are simplifications with zero behaviour risk and can be applied first.

## Interaction map (the Lead's own slice)

The chain in order, with anchors checked against `src/Racer.cs` rather than recalled.

**Command — `ComputeSteering`**

- **`_slidePriority`** (590) — the blend's weight: zero at a quarter of the authored peak slip, one at half of it.
  Consumers: the blend (617), the limiter's countersteer allowance (1193), `IsFullCountersteer`.
- **Aim reference** (594-595) — `yawTarget` from the aim point's curvature, gated off once the slide passes
  `SlidingFraction` of the peak slip.
- **Damper** (595-607) — `-gain × (yaw − target)`; then a term that would **add** in the command's sign is zeroed;
  then the part past neutral, where it is countersteering, keeps **half** its authority.
- **Assembly** (611) — side-by-side + damper + lane.
- **Slide blend** (612-618) — a lerp of the **whole** command toward `side-by-side − slide × share`, weighted by
  `_slidePriority`. It is a lerp, not an addition, so it can both raise and cut.
- **Aligned-wheel deadband** (623) — the command is zeroed when the aim bearing is small and no slide is steering it.

**Limit — `ApplySteerLimits`**

- **Ceiling** (1178-1190) — the cornering law from `ResolveSteerCeiling`: peak slip × 1.333, raised by the
  maneuvering ramp below 30 mph. In Slide-Governed, the larger of that and half the slide plus half a degree.
  Reverse and a standstill keep the raw lock.
- **Countersteer allowance** (1191-1196) — a **raise** to `|slide| × weight` on the answering side.
- **Yaw envelope** (1199-1213, Yaw-Governed only) — a **cut** of the commanded side to the yaw-usage share, which
  the ramp then raises part of the way back.
- **Damper bypass** (1217-1221) — a **raise** to full lock when the damper term *and* the command both oppose the
  rotation.
- **One clamp** (1223) closes both sides.

**Actuator** — `TranslateSteerToInput` (1542): one rate for both directions.

**Who can only raise, who can only cut.** Raisers: the maneuvering ramp, the slide addition, the countersteer
allowance, the damper bypass. Cutter: the yaw envelope alone. Both: the damper term, and the blend, which are the
two places where the wheel can end up on the far side of where the command started. The base is the cornering law.

That asymmetry — four raisers and one cutter — is where the design questions live, and it is what the writer's
candidates will be judged against.

## Arbitration log

Rulings land here as `R1`, `R2`, … — what was decided, on which writer finding, and what was rejected. The log is
the record of *why* the chain looks as it does after this pass, so a later session does not re-litigate it.
