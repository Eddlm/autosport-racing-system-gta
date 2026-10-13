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

## The shape of a section

A system gets its own section, and the section opens with its **goal in one blunt sentence** — what the
system exists to do, phrased as the reason someone would build it — and only then argues: the trade it
makes, where its goal stops, and the shapes already rejected. The goal line stays short and unhedged
because its job is to be quotable in an argument, and the meandering belongs underneath it. A section
with elaboration but no goal line is a description, and descriptions are the code's job. A **standing
limit** gets a section of the same shape but states the limit in its first line instead of a goal,
because what the design cannot know is a decision input just as much as what it is for.

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

## Corners — the future the car plans against

**Goal:** give every car a precomputed list of the bends ahead — where each one is, how tight it is, and
where its turn-in starts — so the car brakes and places itself for a corner it already knows about,
instead of reacting to the curvature under its wheels.

**The table is built from the route at load, not perceived.** A corner is cut out of the track when the
track is loaded: a stretch whose tightest radius is tight enough to be a corner rather than a bend becomes
one record — an apex node, an entrance and an exit walked out from it by geometry, with the region's own
measurements attached. Nothing in it is a sensor reading, and that is the point: a car cannot brake for
what it has not seen, and a bend arrives faster than it can be measured, so the knowledge has to exist
before the race does. The price is that the model can simply be wrong, and every wrong-number complaint
about cornering lives in that price.

**A corner is a region with an apex, not a point.** The span either side is what makes an *approach* a
real thing: the stretch where the car positions itself for the entry, the braking window in front of it,
and its own length each way. Two apexes close enough in *time* — or two that aim the same way — are one
bend, and the survivor is the tighter of the pair; keeping them apart would put two entries where the
driver sees one.

**Two radius readings, and only one of them is trusted.** The region yields the tightest radius any node
in it reads and the smoothest one; the tight figure is the noisy tail of a short-chord measurement, so it
is the *smoothed* radius the systems key on, and the tight one survives as the record's own reference
speed rather than as a number anything drives to. Reading the wrong rung is the kind of defect this file
exists to catch, and the audit of that whole family is filed rather than settled.

**What hangs off one record.** The speed plan takes the apex speed, computed with the car's own downforce,
and brakes for it from a target node a fixed lead before the turn-in. The lane takes the corner's line,
and it *outranks* the ordinary lane while the corner owns the stretch. The crossing rule's entry
exemption reads the same corner, standing down from the braking target to the apex. The shape warnings
ride along as data — a chicane, a crest that unloads the car — so several systems consume one description
instead of each holding its own idea of where the corner is. The record also reserves a field for a
lifting lip before the apex, which *should* pull that braking point earlier and which nothing arms today:
a designed behaviour the code does not implement, filed as a gap rather than described as live.

**Per car, three ahead.** A car holds a queue rather than a single corner: the next three apexes with
their radii and their speeds, and *the next corner* advances once the apex before it is behind the car.
The queue is what lets the plan see past the imminent bend, and the successive-corner rule — a corner
promoted while the car is already inside the previous approach's window is skipped for that approach —
falls out of the same advance rather than being a mechanism of its own.

**It is also the clock.** Besides the two preparations — a speed and a line — the record is the shared
answer to *where the car is in the corner*: the entry exemption for the crossing scale, the brake
learner's sample window, the permission to start a maneuver, and the ban on nitro mid-corner all key on
the same span. Without one shared answer to "before the braking point, in the entry, past the apex", each
of those would invent its own, and they would disagree at the edges — which is exactly how a car ends up
doing two things at once.

**The corner is a reference, not an agent.** Nothing here acts: the car is what coordinates, and it
coordinates *from* this record — which every car in the field shares, while each keeps its own context
about it: what this car knows of the corner's braking, its positioning and whether its approach has
started. That split is the design, and it is why the AI cars behave consistently instead of each
inventing its own corner out of the road under it.

**Where the reference stops.** The tyre's authority, the other cars, the ramp and surface scans and the
recovery all sit outside it: the limiter is its own ladder, the walls and the cards read other cars, ramps
come from a sweep of their own, and a stuck car belongs to no corner. Covering one of those is a boundary
change, not a new field on the record.

**What is the table's and what is the car's.** The table answers *what and where*: which bends exist, where
each one starts and ends, how tight it is, and what shape or surface warnings it carries. Each car answers
*when and how hard*: which corner is next for it, what apex speed its own downforce allows, where its
braking point falls, whether the approach has started, whether it needs to brake or to position, and
whether it is inside the entry yet. Those are recomputed per tick or per approach from the car's own speed
and place, and they are what "the approach" means — the table has no opinion about a car that has not
arrived. Between the two sits a third kind: data precomputed in a shape that scales, like a crest's
severity recorded at a probe speed and rescaled by the car's own speed squared, so the expensive part is
done once and the car only applies its own factor.

## The lane law

**Goal:** put the car on the lane it was told to be on — as directly as the geometry allows and no
harder, so the correction is always present and never a fight.

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

**Goal:** keep a lane change from reading as a car crossed up against its own direction — fade the ask
that would cross the car further, and never touch the ask that unwinds a crossing.

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

**Goal:** get the most out of the driven wheels — hold longitudinal slip inside the band where the tyre
still produces force, so the drive is delivered and the corner keeps some grip.

**The tyre keeps the veto.** The plan keeps its number and the tyre keeps the veto: the plan is a model of
grip, gravity and radius, and a model can be optimistic about a cold tyre, a crest, or a surface it read a
moment ago, so TCS reads the wheels — the ground truth — and trims the *authority to execute* rather than
the plan's number. That is worth more than a tidy number, because past the peak a tyre produces *less*
force: the wheelspin a hopeful throttle creates is drive thrown away.

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

## The standing limit — theory reaches a guess, not the truth

**The limit:** how hard this car can brake for this corner, today, is not derivable from the track. Every
number the plan can compute ahead of time — the grip circle, the radius, the geometry of the braking zone
— is a *model* of the tyre, and a model of a tyre is a good first approximation and nothing more: it
cannot know what the surface under the wheels is really offering at that moment, and the moment is the
only thing that decides the stop. So the design cannot answer "how hard do I brake here" from theory, and
it does not pretend to.

**What it does instead.** It seeds a per-car, per-corner braking factor from a guess and lets the car earn
the rest: it measures how much of the braking phase was spent at full pedal, and once past the apex it
corrects the factor by how far that share missed the target. The evidence has to be clean — a car that
slid on the way in was already on a tyre that had let go, so that sample is discarded, and a slide on its
own is a verdict that cuts the factor without waiting for the numbers. The seed carries a per-car draw,
because identical guesses would produce identical braking errors. The same shape recurs wherever the
model is the weak part: TCS reads the wheels rather than believing the plan, and the steering ceiling
takes the lower of the model law and a measured allowance.

**Three buckets, sorted by staleness.** Everything the car acts on sorts by whether the truth goes stale,
and by how fast. An unknown that never changes is *known* and precomputed — where the corners are, how they
bend, where each span starts — so nothing about it is ever measured; a car reads its own state every tick,
but state is not an unknown and needs no discovering. An unknown that changes over laps is *learned*: read
once, corrected, and kept per car and per corner, which is what the braking factor is. An unknown that
changes per tick is *read*: the tyre's answer to the throttle, the slide, the crossing, the slip a wheel is
already carrying.

The appropriate throttle shows why the fast bucket cannot be avoided, and it is not a number that exists
to be known: it depends on what the tyre does in response to it — the surface, the load, the slip already
there — so the answer is only defined inside the loop, and it is a measurement question even though the
*policy* behind it, holding the tyre just under its peak, is a decision. The rule that follows is a
property of the unknown rather than a preference: learn the ones that repeat, read the ones that do not.
Getting the sort wrong costs in both directions — a learner pointed at a fluctuating quantity is a filter
carrying yesterday's answer, and a live reading pointed at a repeating one throws away every lap's worth of
evidence. Put the way the codebase puts it, the slower a truth changes the further it may live from the
pedal: compute once and publish under a staleness budget, read here as a property of the world rather than
of the code.

**Where each kind may live.** The sort decides that too: a known number belongs in the shared table, a
learned one in that car's own memory, and a read one in the per-tick path, where it must not be cached at
all. It is why the corner table is common to the whole field while the braking factor is per car, and why
slip is read fresh on every tick.

**Five responses to an unknown.** Reading is the moment's answer, used once and gone: slip, the crossing,
the slide. Predicting is that answer carried forward — position, velocity and recent acceleration
extrapolated a short way, `pos + v·t + ½·a·t²`. Three things consume it: the lane preview leads the lane
error with the motion the car is already committing to, so the steering anticipates drift instead of
chasing it; the side to pass a rival on is picked from the predicted heading over the next second; and when
the projection lands on a corner's outside it caps the combined input — full throttle on the centre line,
none at the track edge, a light brake beyond it. That last one shows how a prediction usually ends: as a
*bound* rather than a command, because the plan keeps asking for what it wants and the projection takes the
lower. The horizon is short and the law crude, so a prediction belongs only where a false alarm is cheap,
and a spurious wheel correction is far cheaper than a spurious lift. Arbitrating is when two sources disagree and
one wins for now: take the lower of the model law and the measured one, or let the reading veto what the
plan asked for — settled and forgotten, so the next lap meets it again from the same place. Adapting is
state kept inside the loop while a condition lasts and released when it ends: an integrator rather than a
memory, which is what the parked slip allowance is and what the older throttle control was. Learning is the
durable meeting — state kept against an object rather than a moment, per car, per corner or per track,
corrected on clean evidence and used to change the model next time. The test between the last two is
whether the state outlives the condition that created it, and whether it is attached to something the car
will meet again.

**What a learner can and cannot settle.** A reconciliation has a ceiling worth knowing: it settles
*magnitude*, not *form*. A learner can discover that this corner takes more braking than the model
believed, but it cannot discover that the braking-distance law has the wrong shape, because the only thing
it may move is the single number that law multiplies. When a learner keeps landing against its own limit,
the shape is what wants re-deriving, not the factor.

**What actually learns, and for how long.** Exactly one behavioural learner exists: the braking factor, per
car and per corner. It does not survive the race — the reset that prepares a car clears it — so the field
arrives at every race on the seed and earns its braking again over the laps that follow, and nothing carries
between sessions. Two other mechanisms are often mistaken for learning and are not: the memory layer
*discovers* the game's own offsets once and caches them, and the stability check *calibrates* a car's
at-rest ride height while the car is standing still, both of which are one-shot measurements rather than
accumulating corrections. The pedal-side glides and the parked slip allowance are not learners either: they
adapt within a tick or within a corner and forget immediately. Persisting the braking factor per car and per
track is therefore a real and separable feature — and it would need an invalidation rule first, because a
lesson is only worth keeping while the car and the track are still the same.

**Theory, reality, practice.** The three read as a slogan — what reasoning settles, what the moment
settles, and what experience settles — but the axis behind them is staleness and not truth: the theory is
exact about the track and only approximate about the tyre, and a live reading is true only for as long as
the tyre takes to change. Practice carries the same caveat from the other side: it counts only when the
attempt was honest, which is why a braking sample taken while the car was already sliding is thrown away
rather than learned from.

**A limit can be bounded, not measured, unless it is exceeded.** A car driving well never reveals how much
grip it has: at the apex of a corner taken inside the limit, the lateral g it produces is the *demand*, not
the tyre's maximum, so the peaks a well-behaved car records are a floor on its grip and nothing more —
learned from those alone, the model would only ever be confirmed in its own timidity. The maximum shows
itself exactly once, when the car steps over it, and the sample is contaminated by the slide that produced
it. So the usable evidence is two-sided and comes from events that happen anyway: sustaining a lateral g
without breaking away proves the model is not too high by at least that much, and breaking away at a demand
below the model's promise proves it is too high. Bracketing needs no test track and no clean room — it needs
a filter that decides when a sample was *honest*, which is the same requirement the brake learner already
meets by discarding a sample taken while the car was sliding. Asking for a flat, solitary corner would be
asking for a laboratory this game cannot provide; ask instead for events that can be recognised.

**Except in the braking zone, where the car is at the limit by choice.** The stop is the one manoeuvre the
AI makes at the tyre's edge deliberately: a straight, no lateral work competing for the budget, and full
pedal as the normal command. And the engine's tyre model gives both slip axes **one** friction ceiling —
`GetTractionCoefficientFromRatio` (`wheel.cpp`) returns the same curve for either axis, with the lateral
angle merely mapped into the same normalised domain through `fTractionCurveLateral`, so that value sets
*where* the peak falls and not how high it is, and a combined clamp then makes the two axes share one
budget. So the limit measured under full pedal is the coefficient the corners are planned with, which makes
a braking zone an instrument where a corner is not. The sample still needs tagging — speed, because
downforce adds normal load; grade; and the surface class — plus a cleanliness test, and the plausibility
filter has to become a bracket rather than a discard, since a reading above the model is the evidence
rather than the error.

**Why it is worth stating.** "Can we compute this ahead of time?" has a third answer besides yes and no:
*precompute the shape and let the car apply its own factor* — an apex speed for the downforce the car is
actually making, a crest severity rescaled by speed squared, a braking factor corrected by the car's own
last attempt. That is the pattern to reach for, and the cost is paid in laps: on a corner this car has
never braked for, the plan is running on the seed, which is a guess. It is why the first lap of a corner
is not the one to judge the AI on, and why a learner that corrects on bad evidence is worse than no
learner at all.

## What this file does not cover yet

The damper's law, the slide blend and its countersteer, the steer limiter's ceiling, the brake side's
own caps, the recovery state machine and the speed plan's own radius handling all have design intent
worth writing down — they live in `AGENTS-STEERING.md` and `AGENTS-TECHNOTES.md` today only as live
description. Naming them here is deliberate: this file is expected to grow, one system at a time, and
nothing in it should be read as a claim about a part that is not written up yet.
