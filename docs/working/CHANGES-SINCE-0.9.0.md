# What changed since 0.9.0

**Released:** 0.9.0, 30 October — build 1381.
**This brief covers:** 196 commits, 31 October to 16 November, builds 1382 to 1876.

## New — Nearby Cars

- A new Rivals row takes the cars already standing around a loaded track as rivals, out to 30 m.
- Three choices: **Ignore**, **Join As Is**, **Join And Tune** (the last also lets Auto Styling restyle them). Ignore ships.
- A blue bobbing chevron marks every car that can be adopted. It only marks empty, race-fit cars and says why a car was skipped.
- Adopted cars join the field like any other — sorted by Grid Sorting, never pinned to the front — and they are **extra**: the pool still fills Grid Size.
- The marked list is captured *before* the pool spawns and adoption runs *after*, so a menu closing mid-build does not empty it.

## New — the ground under the tyres is measured

- At load, a ray probe samples the material under every fourth track node; each node carries its own grip.
- Grip is the game's own material multiplier, so grass, sand and kerbs read slower than tarmac. A corner takes the **worst** surface in its span; the route term takes the worst in its window.
- Both speed terms read it: grip enters the apex speed and the braking distance through `sqrt(grip)`.
- A corner only gets an outside approach where the ground is genuinely road — below 0.875 of tarmac grip it does not.
- The load log prints every material hash it met, what it decided, and what share of the route is off road, so a missing road hash shows up rather than hiding.

## Corners are found and measured differently

- A corner is admitted on **how far it turns** (10° minimum) instead of the old blur-node gate; the ceiling moved to 500 m.
- Apex speed reads the circle **fitted to the arc**, not the tightest sample of it. The least-squares fit is gone — it measured tighter, not wider.
- The speed radius is a share of the way from the apex radius to the widest arc that fits the road: the old *Corner Line Share* row is now **Corner Radius Blend (%)**, 0–100, shipping at 20.
- An S is the curvature flipping, confirmed a track width either side; a chicane is judged on its own span and gets no out-in and no outside prep.
- Two bends of different sizes inside two seconds are two corners, and the tighter one governs.

## The AI's line

- The out-in is back as its own move: a capped 2 m lead out, then the shaped line into the apex.
- The prep's creep is a clock, not a per-tick lead — 0.1 m opening to the full cap over three seconds — so it survives a race and no longer re-keys every tick.
- The corner is released at the apex, with four seconds to position, and **coasting past the entrance spends it early** (under 0.05 g of longitudinal G).
- A stopped car is not coasting. A corner the plan skipped keeps no entrance and gets no lean.
- Outward is never a request: the out-in may only pull in, and only the prep asks out.

## Pedals and brakes

- Full brake now asks for a **4.5 m/s** speed error, not 3 — the floor arrives later.
- **Brake Aim Depth** is a row: move what the brake aims at from the corner's entrance to its apex.
- The projection is a pedal ceiling with a floor, not a speed offset, and it is armed by the steer ask.
- The steer limit became a ramp on the tyre's own peak slip: about 1× costs nothing, 1.5× takes the pedal to half, 2× closes it.
- Countersteer is one law on the slide ratio.
- The lane throttle floor was rebuilt: full at the inside edge, 0.1 five metres short of the outer edge, nothing at the edge — gated to the slide band and a ±2 m lane window.

## Race loop fixes

- **You can save again.** Racing set the mission flag, which blocks saving, and the clear never worked in the engine; the flag is no longer touched at all.
- An empty grid says which reason it is empty instead of blaming the roster, and a stale "instanced" flag rebuilds.
- Nothing race-shaped draws on a track that is merely loaded.
- Every apex gets a yellow map blip while the corner visual is on. Blips live for the race, and clearing them says so.

## Quick join from the chevron

- Taking a race at a track's world chevron no longer uses your menu settings: it runs its own fleet (the full roster, six opponents, ±4 PI) and restores your menu afterwards.

## New debug tools

- A centred screen-space **debug panel**: six throttle bars, brake and TCS followers, margin meters, the Gs floor and a stability bar.
- **Show RPMs**: engine rpm derived from wheel speed, gear, and the game's own rev ratio.
- **Show TRlateral** (experimental): the tyre's peak slide angle as a horn against the car's live slide angle.

## Settings and menus

- The ABS row says what it does; "Oprahs" became "Oprah" in the sample names.
- The bench flags are compile-time constants now, not switches anyone restores later.

## Code health

- A declaration sweep through the big files: fields group by what they describe, no line over eighty columns, the small enums read one line each.
- Comment budget back under target.
- Dead code out: `ChainedApex`, the geometric racing-line experiment, the least-squares radius fit, and the retired lane laws.
- The law document (`docs/steering-and-pedals.md`) was rewritten to lead with **why**, in physics terms, before any constant.

---

**Caveat.** This is a list of changes, not of approvals. `docs/working/VERIFY-NOW.md` is the authority on what has actually
been driven in game; the last commit in this range is a batch of debug experiments committed as-is.
