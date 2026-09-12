# ARS — Deferred backlog details (companion to AGENTS.md)

> Split from `AGENTS.md` on 2026-10 to keep AGENTS.md under the DSH auto-load size cap
> (~65 KB — AGENTS.md had grown to ~73 KB and its tail was silently truncated).
> AGENTS.md keeps one-line pointers to the sections below; the text here is verbatim
> (the electric-pace item is new, 2026-10). New deferred TODO details go here,
> with a one-liner mirrored in AGENTS.md.

## Council review backlog (2026-10, commits 9783143 + b845170)

Deferred findings to check out when touched again:

1. **Absolute E-join leak — FIXED (2026-10)**: E-join used to mutate `PowerTargetScale`/`PowerBracketScale` for the whole session — the next menu-start race silently used the E-join pace while the menu showed the persisted value. Fix applied: `InstanceGrid`/`StartRaceFromMenu` take an `anchorToPlayerCar` flag (set only by the E-join site), and the Absolute branch resolves `_resolvedPaceTarget = ComputePlayerCarPaceIndex()` for that grid. `PowerTargetScale` now has exactly two writers — its own menu item and the store restore in `RefreshPowerControls`. Deliberate consequence: after an Absolute E-join, a later menu-started (or restart-fallback) race uses the **persisted** target again instead of the leaked one.
2. **Closest-N fill — DONE (2026-10)**: `VehicleSelector.SelectClosestByPace` ranks the pool by |pace − target| in one pass and takes the N nearest, so the widen loop is gone. Remaining: it **re-parses every vehicle XML from disk per call** (no pool cache) — still worth caching if the pool grows or calls multiply.
3. **Widen default-on silently voids the user's bracket — MOOT (2026-10)**: the bracket itself was retired with the window→ranking change (menu item, `PowerBracketScale`, `Options.WidenBracketFill`, widen loop, `VehicleSelector.Select`), so there is no bracket left to void. Note for the record: the toggle's consumer was removed one step *before* the toggle was, so any build between those two steps carried a dead Debug checkbox.
4. **MenuSettings save-per-scroll**: every ItemChanged does a full ini rewrite on the script thread (41 saves across one PaceOffset scrub at the 0.5 step); dirty-flag + flush on idle/menu-close, skip unchanged values. Also a restore-clamped invalid value (e.g. unknown GridSorting) shows index 0 in UI while the ini keeps the stale value.
5. **AIRacerAutofix per-tick store reads** in Racer.cs hot path → cache in a field at menu-change time.
6. **Minors**: GridSize persists the list *index* while every other key persists values; join chevron could use `(MarkerType)21` (ChevronDownx1) instead of 180° pitch; `arssettings` reload leaves pace-mode enable-states stale until next refresh; Relative-as-default silently changes tuned setups (consider seeding Absolute on migration); `_lastKnownPlayerPace` fallback gives no notification; the absent-key middle-autoselect depends on LemonUI event-ordering (a restore-suppress flag would make it explicit); **`SettingsFile`/`DevSettingsFile` null NRE — FIXED (2026-10)**: `LoadSettings` used to assign them only inside `if (File.Exists(...))`, leaving both null for a missing `Options.ini`/`DevSettings.ini` while ~10 read sites assumed non-null (CATCHUP, `Racer.cs` laps/CATCHUP-behind, `TrackLoader` ReverseRoutes, `OnTick` LoadAtStart/hotkeys, creator defaults, and the `AllowDuplicates` read that has since been deleted). The settings repair now guarantees both files exist before the loads, and both are loaded unconditionally — the null case is structurally gone rather than guarded.

## Key ownership — one setting, two files (Laps FIXED 2026-10)

**`Laps` had two homes and the menu's knob was not the one the race read.** The Race menu's `Laps` item writes `Menu-Race.ini` (`RaceMenuStore.Set("Laps", …)`) and the load migrates `Options.ini`'s value into it, but every *race-logic* reader read `SettingsFile` (= `Options.ini`): `OnTick` (finish/timed block), the reward seed, and `Racer.cs` twice (progress total + the lap-crossing finish test). On the live install that was `Menu-Race.ini` **Laps = 5** vs `Options.ini` **Laps = 3** — the menu said 5, the race ran 3, and changing the menu item changed nothing. **FIXED 2026-10**: those four readers now call `RaceMenuStore.GetInt("Laps", 5)`, so the menu governs race length and `Options.ini`'s `Laps` survives only as the one-time migration source (`RaceMenuStore.Migrate("Laps", …)`). Visible consequence on an existing install: the race now runs the menu's lap count (5, not 3).

Same class, lower stakes: `Options.ini` was documented as "legacy-only, read once to seed", but `CATCHUP` (`OnlyLastHalf`, `OnlyBehindPlayer`) and `ReverseRoutes` are read from it at runtime and have no menu-store home at all — those are legitimately live, so the doc claim is what is wrong, not the reads.

## Entrance brake buffer vs brake learning — revisit together

**Entrance brake buffer vs brake learning — revisit together.** The corner-overshoot symptoms ("blows one specific corner", slide-offs at angled entrances) appeared when the entrance brake buffer was shortened from ~1 s to ~0.6 s. The brake-learning rework (per-corner committed factor, sampling gated at the entrance node, **input-share target** — the share of sampled brake *input integral* delivered at full brake, not a time-share of full-brake moments: the time-share target overshot even when tuned low — proportional gain, factor range, mid default) was built as compensation and masks part of it. When revisiting: the buffer decides how much pre-entrance braking authority the map keeps; the learning factor only scales assumed decel ±. Root-cause candidates: entrance-node detection on square/angled corners (StartNode ≈ apex → almost no planned braking distance), and the all-or-nothing apex-queue invalidation (low-speed rule) leaving cars planless mid-slide. **(2026-09, leading hypothesis: under-braking via the pedal gain.** The speed→pedal loop's full-pedal speed-error divisor was ~6.5 m/s — brakes ramped proportionally weakly through the whole braking zone, so the car entered the corner with a decel deficit it could not recover; the learning rework was compensating for this. The divisor was cut to 3 m/s in the same session — if overshoot symptoms largely vanish, this TODO collapses to just the buffer/entrance-node question.)

## Electric pace balancing — unverified (2026-10)

**Electric pace balancing — unverified (2026-10)**: the electric corrections (accelRaw ×3 pre-multiplier before the common ×30 slope, top speed ×0.9) are user estimates from a two-car sample (ICE sports car ~0.3 G raw, equivalent electric ~0.15 G); in-game check of electric pace numbers vs comparable ICE pending; if off, sample more electric models' raw natives before touching the multipliers again. (The pace-score bullet's old "if wrong in-game, sample real electrics' raw natives first (Electric TODO)" phrasing points here.)

## Corner approach tied to braking plan (idea)

**Corner approach tied to braking plan (idea)**: start the outside-line move ~1 s before the car must brake for the apex (computed from current speed, apex speed, braking decel) instead of a fixed seconds-to-apex gate — matches the lane transition to each corner's actual speed profile.

## Optional update checker as a separate DLL (idea)

**Optional update checker as a separate DLL (idea)**: extract the compiled-in update checker into a small `ARS.UpdateChecker.dll` loaded via reflection only if present (users who dislike network checks delete the DLL); simpler alternative: an `Options.ini` toggle.

## Rear-end prevention — open ideas

**Rear-end prevention** (`ComputeTargetSpeed`): current live behavior (closest rival ahead within 6 m on roughly the same line, lateral offset within `r.CombinedSize.X`, speed blend saturating at 1 m, floored at `rivalSpeed`) is summarized in AGENTS.md; the corner-specific **Yield** maneuver is separate — it arms only when trailing a *faster* rival near a corner entrance **and overrides the speed blend locally**. No further open-idea notes existed in the original file beyond the summary itself (honest note, 2026-10 restructure).