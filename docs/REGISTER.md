# ARS — pending work register

**What this is**: everything still open, in one place, sorted by what it costs to close — work that is decided and not
built, work that is waiting on a decision, and defects that are understood and deliberately not fixed.

**Read this when**: starting a session, picking the next piece of work, or when a report arrives that might already
have a name here.

**What it deliberately does not carry**: constants, build numbers and line numbers. Those live in the code and a
copy of them here rots within a session — this file has quoted a stale pedal scale, a stale lift rate and stale line
pointers in the same week. Slugs and short commit hashes are the handles; the source is the authority.

**Where the rest lives**: finished work is in the git log and `AGENTS/AGENTS-WIP.md`; the ranked ladder by size is
`AGENTS/AGENTS-BACKLOG.md`; the drives still owed are `docs/working/TEST-PLAN.md`; the designs are `docs/DESIGN.md`.

---

## 1 · Decided, not applied

- `outside-leg-half-rate` — the outside leg of the lane move runs at half the follower's rate. Compile-verified and
  never driven: take a corner that asks for the outside and watch the lane settle without lingering (`650cab9`).
- `avoid-hold-timeout` — the hold's backstop is compile-verified and never driven. Hold a rival alongside past the
  timeout and check the allowance reopens (`5ad52c1`).
- `abs-taper` — the taper sits behind a forced-off, greyed `AbsEnabled`, and has not been driven in either shape. It
  returns with one line.
- **Card batch** — the defend and dive windows and the dwell/cooldown pair carry no recorded drive; the divebomb's
  owed drive stands.
- `stability-speed-scaling` — recorded, not started. The first decision is *which* law: a speed-scaled fall, or a
  wheel-off debounce.
- `personality` — the spec is written and nothing is built (`AGENTS/AGENTS-PERSONALITY.md`).
- `pi-controls-merge` — decided, not built: one typed row writing the two keys behind it, because a mode switch would
  otherwise reinterpret a target as an offset.
- `weaponized-grid-filter` — decided, no code: the roster file in the deployed folder still needs the manual edit.
- `player-nitrous-fire` — parked. Reopen only from a run at Log Level `Info`: no `Player nitrous fired` line means the
  key path is never reached.
- **Older drives still owed**, each with its section in `docs/working/TEST-PLAN.md`: `merge-span-gap`,
  `feature-entrance-pull`, `overspeed-thresholds-setting`, `gravity-grip`, the creator's route-table rebuild, and the
  fleet-wide `apex-radius` read.

## 2 · Waiting on a decision

- **The engine's forced ABS on AI cars.** The engine re-grants the ABS flag every tick to any car in the physics
  status, so clearing the model bits changes nothing on an AI car. Settle it by deciding whether ARS writes the
  player status into the AI vehicle's status word — which also moves damage thresholds, population culling and
  collision response — or accepts a player-only ABS.
- `slide-brake-rampdown` — what the brake reduction is a share *of*: the plan's brake, or the cap the ABS guard
  leaves. One line settles it; the shape stays instant in, ramped out.
- `track-node-editor` — how much re-derives from a moved node: the whole section, or the two halves either side of it.
- `feature-entrance-pull` — the pull is provisional and the apex cap binds on nearly every corner. Decide whether the
  pull stays or the span moves.
- **Agenda A's aftermath** — a bend before a chicane now registers as its own corner. Read that track's apex table and
  decide corner or merge.
- `LaneCrossingScale` — the crossing scale is off while a comparison is owed: one corner, scale on and off, same
  entry speed, compare how the lane settles.
- **Recorded ideas with no decision**, each needing a yes or no before it can be ranked: `manual-apex-placement`,
  `creator-apex-preview`, `surface-stats-revival`, `section-roughness-sweep`, `launch-slip-allowance`.

## 3 · Filed, understood, not fixed

### The pedal law has no deceleration term

A car far from its braking point and a car on top of it, both over their target by the same speed, get the same
pedal: the law is one proportional error against the target the plan publishes, and nothing in it knows how much
road is left. The plan hides most of it by descending at the rate the car can actually shed speed — which is why the
symptom is rare and why it is worth keeping. **Falsify** by logging the speed error and the pedal at both ends of a
single braking zone.

### The curvature term has no time dimension while the corner does

The route's speed reference is a function of radius alone, so its target can step by a large fraction of the whole
brake range between one node and the next, where the corner plan descends smoothly. Where a corner plan is live this
is masked on purpose — the curvature is stood down and the corner alone decides. Where no plan covers the corner, the
step reaches the pedal. **Falsify** by logging the route term per node against the ramp of the corner it is
approaching.

### One sentinel value means two different failures

The route uses the same number for "the radius is effectively infinite" and for "the solve degenerated", so a curve
can read as a straight and nothing in the log says whether the road is straight or the geometry failed. **Falsify**
by reading the route radius through a genuinely tight curve and again where the solve is degenerate.

### `grip-native-vs-lateral-g`

The grip native has never been checked against measured lateral g, and every grip law scales off it. If the two
disagree, the error is in every speed the car plans. **Falsify** by logging both at a steady-state limit corner.

### The traction pair is unverified across the fleet

`fTractionCurveMax` and `fTractionCurveLateral` are supposed to sit on one line relating them; a car off that line
invalidates the slide band, the sliding fraction, the steering ceiling and the ABS taper at once. **Falsify** by
dumping the pair for every car in the roster and plotting it against the line.

### `first-corner-eagerness` — low-grip cars over-commit to the first corner of a race

Reported by the driver while testing the apex line near a speed floor: off the line, a low-grip car asks for the
outside earlier and harder than its speed justifies, and a whole field of them looks like it is diving for the outer
edge.

What is already known: the apex line's window is a distance of route plus a *speed floor below the apex speed*, so a
low-grip corner — whose apex speed is low — gets a floor that sits near walking pace; the downforce path does not fold
in the low-grip penalty the same way, so a low-grip car's planned apex speed and its real arrival speed can differ
more than a high-grip one's; and nothing in the line's gate looks at race phase, so the first corner after a standing
start is treated exactly like the fortieth lap.

**When attacking it**: (1) whether the offending cars are below their apex speed at the gate or just above it — the
corner-line log line prints the demand and the distance, not the speed, so the first job is to add the speed; (2)
whether it is the *line* asking or the **lean**, which has no speed floor at all; (3) whether the first corner is
special only because the field is bunched, in which case the gripe is the avoidance lane fighting the apex lane.

### `apex-stays-referenced` — the adopted apex keeps being named after the car passes it

The adopted apex node is only replaced when a *new* corner is adopted, and the nearest-apex search only offers tight
corners ahead — so after the last tight corner of a lap, nothing replaces the reference and everything that reads it
keeps naming an apex that is already behind the car.

It has not hurt yet because the lane is unaffected: with no corner, the corner lane returns nothing and the chain
falls through to the lean, so no demand is built from the stale reference. What is stale is the reference, which the
debug diagonal still draws — that is how it was noticed.

**When it starts to matter**: the moment anything trusts that node for more than drawing — a card, a brake point, a
commit decision.

**How to attack it**: clear the adopted node when its apex is passed *and* the search returns nothing, or give the
search an explicit release so the stickiness ends with the pass rather than with the next adoption. **Falsify** with
the debug diagonal after the last apex of a lap: it should vanish.

*(2026-11-09)* The lane now lets go of its held corner when `CornerSpent` declares it spent or its apex passes, and
re-takes one from the search every tick, so the reference is refreshed rather than kept. Whether the diagonal still
lingers after the last apex of a lap is unverified — the falsification drive above is still owed.

## 4 · Filed from the session audits

### `lane-exit-invasion` — an exit walk can overwrite the next corner's entrance

The shape builder stops an exit run at the next corner's entrance only once it has written the minimum exit nodes. When
the bound arrives before that minimum it steps past it, so one corner's exit profile can be written over the entrance
the next corner's entry pass had already laid down. A node whose `LaneShapeApex` names the previous corner inside the
next one's entry span is the signature. **Suspicion, not a proven defect**: it needs a load log from a track whose
corners sit closer together than the exit minimum allows.

### `requirement-pass-per-frame` — the corner requirement pass runs every frame to feed a half-second refill

`UpdateCornerRequirements` is the detector the refill asks, and it walks every corner to decide whether anything
changed. Its cost has never been measured against the frame budget; nothing observed says it matters, and nothing says
it does not.
