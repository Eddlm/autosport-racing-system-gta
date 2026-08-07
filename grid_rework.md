# Grid Rework — How the player chooses what cars race

Status: **brainstormed, not designed/implemented** (7 Aug 2026)
Scope: replace/upgrade the discipline-tag system with a performance-bracket selection the player can actually configure from the menu.

---

## Current system (facts, as of 7 Aug 2026)

- Cars live in `Scripts\ARS\Vehicles\*.xml` (848 files in the game folder). On save, each file auto-tags:
  - the GTA vehicle **class** (Super, Sports, Muscle, ... lowercased)
  - the **model name** + sanitized variant
- `_disciplineFilter` is a plain string (default `"sports"` in code, `"muscle"` in `Options.ini`) parsed in `Fill_cachedCandidates` (AutosportRacingSystem.cs ~4340) with a hidden mini-DSL:
  - `tag` → optional (any match qualifies)
  - `+tag` → required
  - `-tag` → banned
  - `*tag` → priority (protected from trimming)
- Pool is scored, shuffled randomly, trimmed to grid size, duplicated if too small (`AllowDuplicates`).
- **The player cannot configure any of this in-game.** The LemonUI menu has exactly three root actions (Start Race, Freecam, Debug). Disciplines/GridSize only exist in `Options.ini`. Old `Disciplines`/`GridSize` enum entries (line ~34-35) are dead leftovers.

## Raw material we already have

`Fill_cachedCandidates` already computes a per-model score at grid-build time:

```
stats = GET_VEHICLE_MODEL_MAX_SPEED(m) / 100   // ~0.4–0.8
      + GET_VEHICLE_MODEL_ACCELERATION(m)      // 0–1
      + GET_VEHICLE_MODEL_MAX_TRACTION(m) / 2  // 0–0.5
```

Slow beater ≈ 0.7, top supercar ≈ 2.3. Currently used only to sort before shuffle.

`Racer.cs` also computes a richer per-spawned-vehicle `PerformanceIndex = TopSpeed*5 + Grip*100 + Accel*500` — but only at race time, so not usable at grid-build time.

## Design space

### 1. What defines "performance"?
- Reuse the 3-axis blend (top speed / accel / traction). Cheap, already computed.
- Caveat: it's straight-line bias — says nothing about cornering/handling feel. A modded muscle car and a supercar can have equal top speed but race completely differently.
- Option: a single scalar vs. a **tuple** (top speed band × grip band) — much closer to real race dynamics, harder to put in one control.

### 2. How the player expresses the bracket
- **Band slider**: "target power" 0–10, grid fills with cars within ±1.
- **Quantile tiers**: "Street / Club / Pro / Elite" — pool divided into percentiles, every tier always populated regardless of pool contents. Most robust for small/weird pools.
- **Live count feedback**: menu shows "N cars match" while dragging. Kills the empty-bracket surprise.
- **Preview list**: scrollable list of qualifying models under the current bracket; optionally tick/un-tick.

### 3. Interaction with the tag system
- **Replace** tags entirely (bracket is the only axis).
- **Layer**: bracket first, tags as secondary filter ("muscle class, club tier"). Preserves existing 848-file vocab.
- **Coexist silently**: bracket picks, tags still provide priority/ban semantics within the bracket.

### 4. Grid filling inside a bracket
- Pure random within bracket (current post-shuffle behavior).
- **Spread**: sample across the bracket to avoid 5 identical Adlers.
- **Anti-duplicate**: `AllowDuplicates` exists, but a tight bracket + grid of 8 needs dupes or widening.

### 5. The scale problem
- Raw score (0.7–2.3) is meaningless to a player.
- Expose it as labels ("Street" → "Elite"), real-world-ish mph, or just list qualifying car names instead of a number.

## Lean direction (not decided)

Quantile-tier slider + live "N cars qualify" readout + scrollable preview of qualifying models. Stays honest whether the pool is 848 cars or 12 cars. Tags remain an optional secondary filter for power users. Consider snapshotting the per-vehicle score into the saved XML at car-save time (one native call per save, done once) so brackets don't require re-scanning everything at race start — though 848 files currently scan fast enough.

## Open questions (decide before designing)

1. **One scalar or a speed+grip tuple?** Single power index = simpler. Two-axis (top speed × grip) = truer to race feel, harder UI.
2. **Do existing tags stay meaningful?** Does the bracket fully own selection, or layer on top?
3. **Quantiles vs. absolute bands?** Quantiles adapt to the pool; absolute bands give predictable "this is a 120-mph race" outcomes.
4. Where does the config live — Options.ini, a new section of the LemonUI menu, or both?
5. Does the player who *is* racing need any special-casing (their car in the grid), or is grid purely pool-driven (current stance: player may or may not participate — no matching).
