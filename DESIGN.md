# The design — what each system is for

This file states **the goal of each system and the shape that serves it**, so the idea survives a
refactor and a decision has a yardstick. Every section opens with what the thing is *for*, because a
question about a system is usually a question about its goal, and the goal is what settles a fork:
between two implementations that both look reasonable the one closer to the stated intent wins, and
when neither does, the intent is the thing that has to change.

The code owns every value, and `AGENTS-STEERING.md` owns the live wiring, the anchors and the measured
numbers. Where the three disagree, the code wins and this file is the intent to argue against — but a
system the code no longer serves the goal of is a **defect to file**, not a paragraph to reword.

Nothing here is a line number on purpose. If a claim needs one, it belongs in `AGENTS-STEERING.md`.

Coverage began on the steering side and now states the goal of each system in the control chain, the
pedals included.

## What the system is for

The AI should drive a racing line at the limit of grip without looking like it is fighting the car.
That splits into two authorities which are designed separately and meet at the pedals: **where the car
is pointed** (steering) and **how fast it is going** (the speed plan). A third, the *lane*, is a
position target handed to the steering authority — it says where the car should be, never how hard to
get there.

## The authority chain, and why the order is load-bearing

1. The **lane** decides a lateral target in metres: the high-speed line, the corner's line, an
   avoidance lane, the recovery's edge, or — when nothing has an opinion — the lane the car is
   already in.
2. The **aim point** is that lane offset applied to a lookahead point ahead of the car.
3. The **pursuit** turns the bearing to that point into a wheel angle.
4. The **assembly** adds what the pursuit cannot express: the yaw damper, the side-by-side heading
   match, the slide blend and its countersteer, the direct contact-avoidance push.
5. The **limiters** close the steering *after* the sum — the tyre's ceiling, then the slew — so no
   writer above can escape them. That ordering is the reason a new term cannot quietly exceed the
   tyre: it has to be above the limiters, and everything above them is bounded.

## The lane law

**The idea.** A lane is a position, so the steering that serves it is pure pursuit: proportional to
the error, with no integral, and no standing error because it acts on position rather than on a
velocity. Steering is measured against the car's *velocity* direction, not its heading, which is what
makes the loop blind to a slide in exactly the way it should be.

**The gain, and its two speed terms.** The pursuit gain on a lateral error is `2 · wheelbase /
aim distance²`. The aim distance is a **slice of travel time** rather than a distance, clamped at
both ends:

- **down with speed**: while the lookahead tracks speed, the gain falls as one over speed squared.
  This is not a fudge; a preview proportional to time is what stops a fast car overshooting its aim.
- **the floor** (a handful of metres) is what keeps a crawling car steerable: below it the gain stops
  falling, because a preview shorter than that reads noise rather than geometry.
- **the cap** exists because past a certain speed the preview would be so long that the car aims at
  the wrong corner.

**The lever.** The gain itself, and the lookahead's time. Both change how fast a lane is taken, and
neither changes what "on the lane" means.

**A deliberate asymmetry.** A lateral target that steps from wherever the car is holds a constant
error and therefore never converges; the aim is an absolute edge, so the *bearing* is what grows as
the car falls short. Any rewrite that turns the target into "wherever I am plus a bit" reintroduces a
standing offset.

## The crossing rule — the second speed term

**The idea.** What makes a lane move look violent is not the wheel angle but the **heading the car
carries relative to the road**. A couple of degrees of wheel at speed is plenty to put a car across
the track, while a couple of degrees of crossing needs a fraction of one. So the quantity to shape is
the crossing angle, not the wheel.

**The currency.** The allowance is stated as the heading that carries the car sideways at a fixed
rate — a *sideways speed*, not an angle. That makes it follow one over speed by construction, and it
makes the same number mean the same thing at every speed: half the authority is spent at the
allowance, a third at twice it.

**The shape is a scale, not a boundary.** The ask that would cross the car *further* is faded by how
far the crossing is past the allowance: it keeps half its authority at the allowance, a third at
twice it, and never reaches nothing. There is no width to tune — the allowance itself is the only
knob.

**Rejected shapes, and why they are rejected for good reason.** A hard cap on the wheel was the first
attempt: wrong quantity (a capped wheel at speed still crosses the track), and a flattened top of a
proportional law runs a big correction at a constant command, which builds lateral velocity the tail
cannot shed. A step on the crossing is the same failure shape with a discontinuity added. Both were
driven; the scale was the one that read softer without going lazy.

**Where it applies, and the three exceptions that are the whole design.**

- Only the ask that crosses the car **further** is scaled. The ask that unwinds a crossing is the way
  back and is never touched. (This mirrors the damper, which also only ever corrects one way.)
- It stands down for **the entry itself** — braking target to apex — because turn-in is a crossing on
  purpose: the car has to point into the turn to rotate.
- It stands down *only* there. Keying the exemption on "the corner line owns the lane" was the first
  cut and it was wrong: the corner's positioning window opens seconds before the corner, so on any
  corner-dense track the rule was off for most of a lap.

## The levels of the balance

The same tension — take the line properly without upsetting the car — is balanced at more than one
level, and the levels are not interchangeable: a fix at one does nothing for a complaint that
belongs to another.

**The lane level: how urgently the car takes a lane.** The law is proportional and acts every tick,
with no integral and no memory, so a car that is off its lane is always asking for a correction and
never waiting to be told twice. What bounds the ask is its own geometry rather than a softness dial:
the bearing to the aim point is capped at a right angle, so the ask saturates instead of running away,
and an aim the car is already on — inside the noise of that bearing — is dropped rather than flipped
tick to tick. The knobs are the aim distance, which is a slice of travel time, and the car's
wheelbase. Too shy here reads as late, imprecise positioning; too eager reads as a car fighting
itself. **Triage warning:** a lane complaint is usually the *target* arriving late — the lane stack's
window, the walls — and from the driver's seat the two failures look identical.

**The crossing scale is not part of that balance.** It does not exist to keep the car stable; it
exists so that a lane change does not read as a car angling itself across its own direction. It is
asymmetric and it stands down for the entry, so treating it as a stability knob misreads it.

**The steering level: never ask for more than the car can do at this speed.** The ask is clamped
after everything above it has had its say, so no lane correction, damper or maneuver can escape it.
The ceiling is a ladder rather than one number: the tyre's live peak slip at this speed, with the
grip-circle law — grip and gravity over speed squared, through the wheelbase — as the fallback, and a
slow-speed ramp that can only *raise* the ceiling so a crawling car can still place itself. Two
deliberate escapes exist, both on the answering side only: a countersteer earns authority towards the
slide it is correcting, and a damper term that is fighting the rotation is not capped.

**The speed level: the same law read the other way.** The grip-circle ceiling also exists as a speed
— the fastest a car could take a radius with the wheel it is allowed — and that speed is blended into
the plan, so a car that cannot steer a corner slows down for it. This is what keeps the steering level
and the speed plan one law instead of two that disagree, and it is the level that is easiest to
forget, because the symptom of over-asking appears on the pedals.

**The damper is where stability actually lives.** Instability is a yaw problem: it is answered by the
damping term, whose gain falls with speed and is weaker on high-grip cars, with the ceiling as the
hard backstop beneath it. The lane law is not where oscillation gets solved, so a lane change that
reads nervous is a damper or a ceiling complaint far more often than a gain one.

## Traction control — the AI's throttle against the tyre's own verdict

**Goal:** get the driven wheels working at the slip the tyre actually wants, so the throttle the plan
asks for can be *delivered* — the plan keeps its number, the tyre keeps the veto. The plan is a model of
grip, gravity and radius, and a model can be optimistic about a cold tyre, a crest, or a surface it read
a moment ago; the wheels are the ground truth, so TCS reads them and trims the *authority to execute*
rather than the plan's number. What that buys is not only less spinning: past the peak a tyre produces
*less* force, so the wheelspin a hopeful throttle creates is drive thrown away.

**Why the AI needs this at all.** A driver feels the wheel let go and lifts; the AI has no feel of its
own — its throttle is the plan's number, and the engine's own traction aids are coarse and not ARS's to
read. Reading the wheel is what stands in for that feeling. The throttle cap is this goal's
*implementation*, not the goal itself: anything else that holds the wheel at the slip it wants — a
better grip model, a different pedal law — serves the same end and would leave this system's goal
untouched.

**Why the target sits below the peak.** The tyre's maximum is a peak, not a set point: there is more
force just under it than just over it, so a controller that aims at the maximum spends half its time on
the wrong side of it. TCS holds a chosen operating point *under* the peak, which is a deliberate trade
of a slice of force at this instant for the tyre's potential over the whole corner — and the goal is the
corner, not the instant. Nothing below the target is touched, so part-throttle, balancing and a lift stay
entirely the plan's business, and the cut has a floor because a car that lifts completely gives up the
balance it needs to drive out of the problem.

**One axis each.** What TCS attacks is longitudinal slip on the driven wheels; lateral slip belongs to
the steering side, and TCS never reads it. The lateral benefit is real but it is a consequence rather
than a target: a tyre spends one friction budget across both axes, so slip kept out of the longitudinal
one is grip left over for the corner.

**It reads the tyre, not the torque.** The wheels are read directly, and the sign matters: a wheel
locking under braking reads on the same axis with the opposite sign, so it can never trigger a throttle
cut. Off track the target scales with the surface, which is the same idea applied where the whole curve
is lower.

**The twin on the other pedal.** The brake side shares the mechanism but not the goal: throttle-side
control *regulates* — it holds a curve point — while brake-side control *contains*, because full lock
does not sit on the curve at all but in the flat region past it. Same instrument, different question.

**What TCS is not.** It adds no grip and models no differential, it never lowers the plan's speed
target, and it is not there to be fast on its own — it is there so the lap the plan promised is actually
delivered on a tyre that is not behaving. A change that makes TCS alter the plan's speed has moved to the
speed level and left this system's goal behind.

## What this file does not cover yet

The damper's law, the slide blend and its countersteer, the steer limiter's ceiling, the brake side's
own caps, the recovery state machine and the speed plan's own radius handling all have design intent
worth writing down — they live in `AGENTS-STEERING.md` and `AGENTS-TECHNOTES.md` today only as live
description. Naming them here is deliberate: this file is expected to grow, one system at a time, and
nothing in it should be read as a claim about a part that is not written up yet.
