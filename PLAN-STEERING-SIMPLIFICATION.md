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

The chain in order. Anchors were rebased onto the file as the simplification commit leaves it; the first pass here
was written against the pre-simplification revision and every line had shifted, which is worth remembering the next
time this map is quoted.

**Command — `ComputeSteering`**

- **`_slidePriority`** (584) — the blend's weight: zero at a quarter of the authored peak slip, one at half of it.
  Consumers: the blend (610), the limiter's countersteer allowance (1186), `IsFullCountersteer`.
- **Aim reference** (587-589) — `yawTarget` from the aim point's curvature, gated off once the slide passes
  `SlidingFraction` of the peak slip.
- **Damper** (589-600) — `-gain × (yaw − target)`; then a term that would **add** in the command's sign is zeroed;
  then the part past neutral, where it is countersteering, keeps **half** its authority.
- **Assembly** (604) — side-by-side + damper + lane.
- **Slide blend** (606-611) — a lerp of the **whole** command toward `side-by-side − slide × share`, weighted by
  `_slidePriority`. It is a lerp, not an addition, so it can both raise and cut.
- **Aligned-wheel deadband** (616) — the command is zeroed when the aim bearing is small and no slide is steering it.

**Limit — `ApplySteerLimits`**

- **Ceiling** (1170-1180) — the cornering law from `ResolveSteerCeiling`: peak slip × 1.333, raised by the
  maneuvering ramp below 30 mph. In Slide-Governed, the larger of that and half the slide plus half a degree.
  Reverse and a standstill keep the raw lock.
- **Countersteer allowance** (1184-1189) — a **raise** to `|slide| × weight` on the answering side.
- **Yaw envelope** (1192-1204, Yaw-Governed only) — a **cut** of the commanded side to the yaw-usage share, which
  the ramp then raises part of the way back.
- **Damper bypass** (1210-1214) — a **raise** to full lock when the damper term *and* the command both oppose the
  rotation.
- **One clamp** (1216) closes both sides.

**Actuator** — `TranslateSteerToInput` (1535): one rate for both directions.

**Who can only raise, who can only cut.** Raisers: the maneuvering ramp, the slide addition, the countersteer
allowance, the damper bypass. Cutter: the yaw envelope alone. Both: the damper term, and the blend, which are the
two places where the wheel can end up on the far side of where the command started. The base is the cornering law.

That asymmetry — four raisers and one cutter — is where the design questions live, and it is what the writer's
candidates will be judged against.

## Arbitration log

Writer findings are in `STEERING-SIMPLIFICATION-FINDINGS.md`; every claim below was checked against the tree by the
Lead before it was ruled on, and the checks that mattered are named in the ruling.

- **R1 — C1, the temporary corner diagnostic: cut on the driver's word.** Log-only and zero-risk, which is what made
  it a candidate; the Lead initially kept it because it exists to diagnose corner misses, but the driver does not read
  its output, so the tool was dead weight rather than insurance. Removed with its call, `ForwardNodes` and both log
  methods in build 472; the workstream that might want it again is told where to recover it, so nobody rewrites it.
- **R2 — C2, C3, C4, C8: accepted and applied.** Grep confirmed no reader outside the spans named: `_rawCornerLane`
  was write-only, both steer-limit fields were read only inside `ApplySteerLimits`, `CountersteerFullSlideFraction`
  multiplied by exactly 1, and the `nonLaneSteerDeg` intermediate only re-ordered a sum. The sign note — LEFT bounds
  positive commands — moved onto the locals rather than dying with the fields, because that asymmetry is deliberate.
- **R3 — C5: rejected.** `SteerDampingEnabled` and `SteerDampingAimReference` cost two constants and are the
  documented A/B handles for the damper; retiring them retires the experiment. Keep.
- **R4 — C6, the geometry ceiling fallback: accepted in principle, blocked on Q7.** It is unreachable for every
  finite state, but it is the only thing holding a ceiling when the velocity length is NaN. Do not remove it until
  that state is declared impossible at the source or replaced by a named guard; ~30 lines can wait for that.
- **R5 — C7, the duplicate forward-speed expression: rejected for now.** It is not behaviour-preserving on a pitched
  car and the reverse gate reads the same expression, so it is a cleanup with a small real handling cost and no drive
  behind it. If the pitch difference is ever suspected, that is a test, not a cleanup.
- **R6 — C9, dropping the yaw cut: the driver's call, not this pass's.** It is the only candidate that changes what
  the wheel may ask for, and the governor A/B is undriven. Not applied; it becomes a candidate once the A/B has a
  verdict.
- **R7 — Q1: the contract was wrong and the code is right.** The limiter runs after the pedals on purpose — its own
  comment at `Racer.cs:3370` says it closes the steering last so nothing escapes it — and stronger than the findings
  file first put it, since the stuck-recovery override also writes `Control.SteerDegrees` above it. The pipeline list in `AGENTS.md`
  is corrected to the code's order, and the consequence is recorded there: the overspeed arm gate and
  `SteerLimitedSpeed` read the previous tick's post-slew steer.
- **R8 — Q2: the key is declared, not the note amended.** `SteerLimitMode` was missing from
  `SettingsRepair.BuildSchema`, so `PruneOwnedFiles` dropped it on every load and the governor silently returned to
  Yaw-Governed across a reload — which would have invalidated the A/B the driver is running. Declared as `Kind.Text`
  with its two menu strings.
- **R9 — Q3: accepted, the test-plan note was wrong.** The ramp is applied to a different input per mode, so the two
  differ throughout the band and meet only at or below 5 mph. Corrected.
- **R10 — Q5: accepted; the two menu descriptions are corrected** (the damper one claimed a zero reference the live
  code does not use, the mode one described the slide as a replacement after it became an addition). The `if (1 == 2)`
  gotcha is removed from `AGENTS.md` — grep finds no such gate in any `.cs`. The remaining anchor-drift items are
  filed for the next memory pass rather than applied blind.
- **R11 — Q6 is closed by R3; Q8 stays open.** Unifying the limiter's countersteer allowance with the blend's share
  would change the allowance's never-clip guarantee, which the test plan wants kept, so they remain two laws.
- **Q4 — deferred, not ruled.** The writer's evidence that a straight-line offset *is* corrected contradicts the
  steering survey in two places, but the correction touches the survey's ranked items and needs its own read of the
  lane code. Filed, not patched.
