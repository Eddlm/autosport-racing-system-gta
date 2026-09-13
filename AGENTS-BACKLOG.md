# ARS — Deferred backlog details (companion to AGENTS.md)

> Split from `AGENTS.md` on 2026-10 to keep AGENTS.md under the DSH auto-load size cap
> (~65 KB — AGENTS.md had grown to ~73 KB and its tail was silently truncated).
> AGENTS.md keeps one-line pointers to the sections below; the text here is verbatim
> (the electric-pace item is new, 2026-10). New deferred TODO details go here,
> with a one-liner mirrored in AGENTS.md.

## Complexity ladder — the open items, simplest → most complex (2026-11)

> Ordering by effort × design uncertainty × risk, not by importance. Item names match the
> `AGENTS.md` open-items list and each one's detail lives in its own section below.
> Tiers are meant to be pickable: **0–2 need no wheel time** (code, menus, or a decision);
> **3+ need the user driving to verify**, or a design call first.
> **Docs drift — the code wins.** Any line here may lag the source; check before relying on it,
> and correct the line you are touching rather than reconciling the file wholesale.

**Tier 0 — decisions, no code**

1. Dist shipped defaults — pick the values in the live install, build so `RefreshDist` propagates, commit.
2. Menu-persistence verification — delete each `Menu-*.ini`, restart, click every toggle; fresh install and existing one.

**Tier 1 — localised edits, low risk**

3. Council minors — save-per-scroll dirty flag; the per-tick store reads; `arssettings` reload staleness; join-chevron marker; `GridSize` index-vs-value; the silent pace fallback.
4. ~~Six unreachable writers prune~~ — **DONE (2026-11)**, see the teardown section below; the file tools turned out to be usable (drive them from a `line=length` map).
5. Weaponized grid filter — a marker in `cars.txt`; small, deferred by choice.
6. Update checker as a separate DLL — self-contained extraction.

**Tier 2 — verify in game, then tune**

7. ~~Side-by-side heading assist~~ — **VERIFIED in game (2026-11)**.
8. Start-line flare placement — geometry on a disabled pipeline.
9. ~~Stuck-recovery escalation~~ — **already in the code**: `ApplyStuckRecoveryOverride` teleports the racer to the nearest track edge after 5 failed reverse attempts, using the same math as the player's `ResetToTrack()`; user confirms it works for player and AI (2026-11).

**Tier 3 — single-method changes**

10. Electric slow-electrics class question — data plus a balance judgment (see the electric section).
11. Off-track projection aggressiveness — three candidate fixes, all inside `ApplyOffshootBlend`.

**Tier 4 — coupled systems**

12. Corner approach tied to the braking plan — lane timing starts reading the braking map.
13. ~~Entrance brake buffer vs brake learning~~ — **RESOLVED (2026-11)**: the pedal-gain divisor was the cause (user verdict); its two residual hypotheses explained the same symptom and stay dormant while that stays gone.
14. Steer-limiter throttle cut rate — needs a grounded derivation for values tuned by feel.

**Tier 5 — new controllers and design decisions**

15. Snap-oversteer D-term — a term that does not exist yet; dedicated session.
16. Gravity vs grip & speed — settle how gravity scales grip against each site that multiplies it again.
17. Pace: model-theoretical vs instance — structural: pre-race selection is spawn-free by design.

**Tier 6 — subsystem reworks**

18. Two-projection route speed — replaces the geometric route speed, the sweeping-corner authority.
19. Track-creator revival — entry point, shared-statics ownership, mutation policy, the `Wide` off-by-one.

## Council review backlog (2026-10, commits 9783143 + b845170)

Deferred findings to check out when touched again:

1. **Absolute E-join leak — FIXED (2026-10)**: E-join used to mutate `PowerTargetScale`/`PowerBracketScale` for the whole session — the next menu-start race silently used the E-join pace while the menu showed the persisted value. First fix routed the join through a per-grid `anchorToPlayerCar` flag; **final resolution (same day) deleted the override entirely**: `Pace Mode` is now the sole authority on the anchor for every grid — menu start, restart fallback and E-join alike — so nothing outside the menu can influence it and the leak is impossible by construction rather than merely avoided. `PowerTargetScale` has exactly two writers, its own menu item and the store restore in `RefreshPowerControls`; `ComputePlayerCarPaceIndex` (the join's fallback wrapper) is deleted with it. Consequence worth knowing: an Absolute-mode E-join now builds the field around the fixed Pace Target, so the player chooses "race the car you're in" by choosing Relative.
2. **Closest-N fill — DONE (2026-10)**: `VehicleSelector.SelectClosestByPace` ranks the pool by |pace − target| in one pass and takes the N nearest, so the widen loop is gone. Remaining: it **re-parses every vehicle XML from disk per call** (no pool cache) — still worth caching if the pool grows or calls multiply.
3. **Widen default-on silently voids the user's bracket — MOOT (2026-10)**: the bracket itself was retired with the window→ranking change (menu item, `PowerBracketScale`, `Options.WidenBracketFill`, widen loop, `VehicleSelector.Select`), so there is no bracket left to void. Note for the record: the toggle's consumer was removed one step *before* the toggle was, so any build between those two steps carried a dead Debug checkbox.
4. **MenuSettings save-per-scroll**: every ItemChanged does a full ini rewrite on the script thread (41 saves across one PaceOffset scrub at the 0.5 step); dirty-flag + flush on idle/menu-close, skip unchanged values. Also a restore-clamped invalid value (e.g. unknown GridSorting) shows index 0 in UI while the ini keeps the stale value.
5. **AIRacerAutofix per-tick store reads** in Racer.cs hot path → cache in a field at menu-change time.
6. **Minors**: GridSize persists the list *index* while every other key persists values; join chevron could use `(MarkerType)21` (ChevronDownx1) instead of 180° pitch; `arssettings` reload leaves pace-mode enable-states stale until next refresh; Relative-as-default silently changes tuned setups (consider seeding Absolute on migration); `_lastKnownPlayerPace` fallback gives no notification; the absent-key middle-autoselect depends on LemonUI event-ordering (a restore-suppress flag would make it explicit); **`SettingsFile`/`DevSettingsFile` null NRE — FIXED (2026-10)**: `LoadSettings` used to assign them only inside `if (File.Exists(...))`, leaving both null for a missing `Options.ini`/`DevSettings.ini` while ~10 read sites assumed non-null (CATCHUP, `Racer.cs` laps/CATCHUP-behind, `TrackLoader` ReverseRoutes, `OnTick` LoadAtStart/hotkeys, creator defaults, and the `AllowDuplicates` read that has since been deleted). The settings repair now guarantees both files exist before the loads, and both are loaded unconditionally — the null case is structurally gone rather than guarded.

## Key ownership — one setting, two files (Laps FIXED 2026-10)

**`Laps` had two homes and the menu's knob was not the one the race read.** The Race menu's `Laps` item writes `Menu-Race.ini` (`RaceMenuStore.Set("Laps", …)`) and the load migrates `Options.ini`'s value into it, but every *race-logic* reader read `SettingsFile` (= `Options.ini`): `OnTick` twice (per-racer leaderboard freeze + reward seed) and `Racer.cs` twice (progress total + the lap-crossing finish test). On the live install that was `Menu-Race.ini` **Laps = 5** vs `Options.ini` **Laps = 3** — the menu said 5, the race ran 3, and changing the menu item changed nothing. **FIXED 2026-10, in two passes — the first was incomplete**: `Racer.cs`'s two readers were repointed in `3fa7737` (which claimed all four; it was two), and the two in `AutosportRacingSystem.cs` followed in the next step — the per-racer freeze (`racer.Lap >= raceLaps`) and the reward seed; the freeze read was also **hoisted out of the per-frame per-racer loop** while there. Half-fixed was actively wrong, not merely stale: with the menu at 5 and `Options.ini` at 3, cars were classified finished at lap 3 while their own finish test waited for lap 5, freezing leaderboard positions early. `Options.ini`'s `Laps` now survives only as the one-time migration source (`RaceMenuStore.Migrate("Laps", …)`). **Lesson: after repointing a key, grep for the key name — not for the call sites you remember.**

Same class, lower stakes: `Options.ini` was documented as "legacy-only, read once to seed", but `CATCHUP` (`OnlyLastHalf`, `OnlyBehindPlayer`) and `ReverseRoutes` are read from it at runtime and have no menu-store home at all — those are legitimately live, so the doc claim is what is wrong, not the reads.

## Pace is model-theoretical, not instance-measured (noted 2026-10, not started)

**The pace index describes the catalogue model, not the car in front of you.** Every pace number in the system — the selection anchor, the player's pace in Relative mode, the leaderboard prefix — comes from *model-level* natives called on a model hash: `TryComputePlayerCarPaceIndex` checks `ModelPaceIndexCache` first (the catalogue cache) and only then falls back to model-hash natives, and `Racer.Initialize` recomputes `VehicleData.PowerScale` the same way. Nothing on that path reads the spawned vehicle or the handling struct, so **a stock model and a fully upgraded one score identically**.

Where the gap shows:
- **The player's own car.** A tuned player car is paced at its stock number, so the Relative anchor (and any `PaceTarget` picked to match it) describes a car that no longer exists — the field gets matched against a fiction. Note the cache is checked *before* anything else, so a cache hit on the model hash **precludes** any instance read by construction.
- **AI grid cars — no longer a gap (corrected 2026-11).** This bullet used to be the other half of the problem: AI cars were built with upgrades (`AITuningLevel` 0–3, plus a supplier-XML `<Acceleration>` node driving the `EnginePowerMultiplier` loop in `ApplyAccelerationOverride`), so their real pace could exceed the number they were selected on. **All three are gone** — those paths died with the vehicle-XML teardown, and `RandomTuning` (the only other performance applier) was already unreachable before it was deleted. AI grid cars are stock models today, so the model-theoretical number describes them correctly and **the live gap is the player's car alone**.

**The machinery for the precise reading already exists** — this is the useful part. `Racer.Initialize` already reads *instance* data (`Handling.Grip` from the spawned-handle native `0xA132FB5370554DB0`, plus `Handling.EstimatedTopSpeed` and `Handling.Acceleration` out of the handling struct) to build `VehicleData.PerformanceIndex`, a separate integer index that no selection path uses. So the work isn't "invent instance reads"; it's "route the pace through the instance reads the Racer already performs".

**Two directions, in dependency order:**
1. *Precise player pace (independent, doable now)*: read the spawned instance — the spawned-handle natives are deliberately different from the model-level pair (native table in `AGENTS-TECHNOTES.md`) — plus upgrade state (engine/transmission/turbo/suspension mods, `EnginePowerMultiplier`) and the handling-struct grip. Fairness consequence to keep in view: the player's pace anchors the whole grid in Relative mode, so a more accurate player number changes the field the player races.
2. *Inferred pace for upgraded AI cars (do together with racer upgrades)*: once racer upgrades are a real system, pace must be a function of the **built** car — the applied upgrade set and tuning level feeding the same `ComputePaceIndex` — not the catalogue entry. Preserve the invariant that the leaderboard number equals the selection metric (both derive from `Racer`'s spawn-time recompute, so they must move together).

**Noted for later consideration (2026-11, user request) — the discrepancy, and one idea on file.** *Discrepancy:* the grid's PI describes the **model**, so a tuned instance's real performance is not what the field was matched to — it only matters for the player's car today (AI cars are stock, see above). *Idea:* the upgrades **are** readable per instance, so "what the mods did to the power" is measurable rather than guessed — mod levels (`GET_VEHICLE_MOD`, `IS_TOGGLE_MOD_ON`, `GetModCount`, the turbo toggle) plus the handling-struct fields those mods mutate (drive force, brake force, traction, top speed; offsets in `VehicleMemory.cs`). Two uses: **(1)** read the player's spawned instance for a true anchor; **(2)** sample drive force/braking before and after each mod level *once, in game*, to build a per-level multiplier table — which would also let AI pace be **inferred from an upgrade set** without spawning anything, i.e. exactly what the design tension above needs. Not started; recorded so the idea isn't lost.

**The real design tension** (worth settling before either direction): reading the *stock* model is what makes pre-race evaluation free and spawn-free — the grid is chosen *before* any car exists. The moment pace needs an instance, AI selection either has to spawn-and-read (expensive, and structurally too late) or keep inferring from an upgrade set. That choice, not the native selection, is the decision to make when this is picked up.

## Vehicle XML teardown — writers pruned (2026-11)

**What happened**: the roster moved to `Vehicles\cars.txt` (one model key per line) and the whole XML car-info system was retired — save-car, driver save, discipline tags and the per-car appearance reads. The *entry points* are gone: cheats (`arssavecar`, `arssavedriver`, `arscarlisten`), the listen-mode Jump trigger, the `ListenMode` field, `Options.SaveThisCar`/`SaveDriverModel`, and every consumer of the XML reads.

**Pruned (2026-11, DONE)**: the six writers that outlived their entry points — `RandomTuning`, `LoadDriver`, `CreateDriver`, `CreateVehicle`, `CreateVehicleFromName`, `CreateVehicleFromHash` — are **deleted** from `src\AutosportRacingSystem.cs`: 590 lines, a **pure deletion** (numstat 0 added / 590 deleted, exactly three hunks), file 3979 → 3389 lines. Build green and a race verified in game (script loads, grid spawns, no exception in either log). `DisplayHelpText`/`DisplayHelpTextTimed` sat *between* the first two regions and survived — reading each span before cutting is what made that boundary safe. Nothing was orphaned: the six used only natives, `ScriptsFolder`, `Log`, `UI`, `CanWeUse` and `GetRandomInt`, all live elsewhere, and `Drivers\` no longer appears anywhere in the code.

**Correction to the note this section used to carry**: the whitespace-only lines were **not** the blocker. Exact-match edits are fail-safe — a mismatched `old_string` changes nothing — so the deletion was driven by a `line=length` map of each region (`Get-Content` + `.Length`, which makes blank runs and trailing-space lines unambiguous); what actually failed in the early attempts was **miscounting blank-line runs by eye**, never the spaces. Neither option proposed here (rewrite the methods whole with `write`, or normalise the trailing whitespace first) was needed, and the "no scripts to edit code files" rule held throughout.

**Deleted (2026-11)**: the 844 `Vehicles\*.xml` files and the `Drivers\` folder (9 files) are gone from the live install *and* `Dist`, so both now ship `Vehicles\cars.txt` alone. Every one of those 853 `Dist` files was git-tracked, so `git checkout` restores them if the rebuild ever wants a sample. Nothing read them: `VehicleCatalog` reads `cars.txt`, and the one recursive `*.xml` sweep in the code (`MenyooAppearance.GetFiles`) targets the game's own `menyooStuff\Vehicle`, not the ARS folder — worth knowing before a future "unread files" pass mistakes it for a reader. Note the boundary for such a pass: `Options.ini` is live (`CATCHUP`/`ReverseRoutes`), and `Settings.ini` + `Menu-Settings.ini` are each read **once** as legacy migration inputs, so none of the three is unread.

## Dist shipped defaults — pending decision (noted 2026-10)

**What is pending**: `Dist\AutosportRacingSystem\Settings\*.ini` is the version-controlled, ship-ready mirror of the live install, and its settings currently hold *whatever the last dev session left behind* rather than deliberate values — as did the committed set before it (it came from an even earlier session). The keys that shape a first-run experience: `Laps`, `GridSize`, `Track`, `PaceMode`, `PaceOffset`, `PaceTarget`, `ReverseRoute` in `Menu-Race.ini`; the racer/AI knobs in `Menu-Racers.ini`; every debug toggle in `Menu-DevSettings.ini`. Decide them **on purpose** before publishing a release.

**Why it matters**: `Dist` is what a new installer receives. A personal favourite track, an arbitrary pace target/grid size and arbitrary debug toggles (which decide a player's out-of-the-box visual aids) are a poor first impression for a WIP release.

**Mechanics — direction matters**: the `RefreshDist` build target robocopies **game install → Dist**, so editing `Dist` alone is overwritten by the next build. The order that sticks: set the values in the live install (pick them in the menu, or edit the live inis), then build so `RefreshDist` propagates them, then commit `Dist` as a **content** change (only data is git-tracked; `Dist/**/*.dll` is ignored).

**Until then**: those files sit dirty in the working tree on purpose. Stage files explicitly on every commit — never `git add -A` — so the drift cannot ride along with unrelated work (also a workflow rule in `AGENTS.md`).

## Entrance brake buffer vs brake learning — RESOLVED (2026-11)

**Resolved (2026-11, user verdict): the pedal gain was it.** The full-pedal speed-error divisor was cut sharply in the same session and the symptoms went with it — exactly the collapse this note predicted. The two root-cause candidates below (entrance-node detection on square/angled corners; the all-or-nothing apex-queue invalidation) explained the same symptom, so they are **dormant, not open** — reopen only if overshoot returns. **Kept for the trail:**

**Entrance brake buffer vs brake learning — revisit together.** The corner-overshoot symptoms ("blows one specific corner", slide-offs at angled entrances) appeared when the entrance brake buffer was shortened from ~1 s to ~0.6 s. The brake-learning rework (per-corner committed factor, sampling gated at the entrance node, **input-share target** — the share of sampled brake *input integral* delivered at full brake, not a time-share of full-brake moments: the time-share target overshot even when tuned low — proportional gain, factor range, mid default) was built as compensation and masks part of it. When revisiting: the buffer decides how much pre-entrance braking authority the map keeps; the learning factor only scales assumed decel ±. Root-cause candidates: entrance-node detection on square/angled corners (StartNode ≈ apex → almost no planned braking distance), and the all-or-nothing apex-queue invalidation (low-speed rule) leaving cars planless mid-slide. **(2026-09, leading hypothesis: under-braking via the pedal gain.** The speed→pedal loop's full-pedal speed-error divisor was ~6.5 m/s — brakes ramped proportionally weakly through the whole braking zone, so the car entered the corner with a decel deficit it could not recover; the learning rework was compensating for this. The divisor was cut to 3 m/s in the same session — if overshoot symptoms largely vanish, this TODO collapses to just the buffer/entrance-node question.)

## Electric pace balancing — resolved, one sub-item open (2026-11)

**Superseded (2026-11)**: this section used to describe hand-fitted corrections (accelRaw ×3 before the common slope, top speed ×0.9) derived from a two-car G sample, with the in-game check pending. Both multipliers are gone and the check happened: the flag comes from a verified model-hash native and the raw G is scored at the **midpoint of the game's own drive-force ramp** (×5 at standstill falling linearly to ×0.9 at top speed) with **no top-speed discount**. Live detail is in `AGENTS.md` → pace score — trust that side if the two ever disagree; do not resurrect the old numbers.

**Settled (2026-11, user decision)**: class membership must **not** influence an electric's treatment — no class-aware special-casing. Cyclone / Voltic / Rocket Voltic are simply mis-classed by R* and keep their ramp-derived numbers (sitting ~40 PI under the Super median is accepted). Recorded so it does not get re-opened.

## Corner approach tied to braking plan (idea)

**Corner approach tied to braking plan (idea)**: start the outside-line move ~1 s before the car must brake for the apex (computed from current speed, apex speed, braking decel) instead of a fixed seconds-to-apex gate — matches the lane transition to each corner's actual speed profile.

## Off-track projection behaviour — braking is right but too aggressive (TODO, next session)

**Off-track projection behaviour (TODO — user's pick for the next session)**: verdict is that the projection's off-track response **does** brake and the braking is correct in principle, but it is **too aggressive**. The mechanism to change is `ApplyOffshootBlend` (shape in `AGENTS.md` → Speed pipeline; the ramp endpoints and floor are in the code). Candidates, roughly in the order worth trying: (1) make it a **decay** instead of the instant clamp — the clamp is the likely jerk source, and a short slew should keep the safety while removing the stab; (2) soften the **ramp itself** (where the ramp begins past the edge, and the brake floor it ramps to) rather than the trigger, since the trigger fires on a projection the user considers correct; (3) gate by **how far** past the edge the projection lands (depth) instead of a past-the-edge boolean, so a marginal projection is nudged rather than stamped on. Related idea in `AGENTS-TECHNOTES.md`: the pessimistic projection (ease speed *before* the edge rather than an after-the-fact cap) — the better end state if this keeps fighting.

**Do not confuse it with Speed Offset**: the projection cap governs **input**, the offset governs the **target**. While the cap is active a raised `Intention.Speed` produces no extra pedal, so "the offset does nothing" in a corner is expected there — the tell is the debug HUD's intended speed moving while the pedal trail doesn't.

## Optional update checker as a separate DLL (idea)

**Optional update checker as a separate DLL (idea)**: extract the compiled-in update checker into a small `ARS.UpdateChecker.dll` loaded via reflection only if present (users who dislike network checks delete the DLL); simpler alternative: an `Options.ini` toggle.

## Track creator — dormant, extracted 2026-10 (+ the prune list)

**State**: the in-game track creator is **unreachable**. `_routeEditorActive` has exactly two writers, both `= false` (its initializer and `CleanEverything`), so all seven read sites are dead branches, and no menu item, cheat or hotkey enters creator mode — the `arscreatetrack` string only ever existed inside the "No tracks found" notification (since reworded to point at the Tracks folder). The route-recording loop, its section preview and the route-node visuals therefore never run.

**Extraction (behavior-preserving, byte-verified)**: the feature now lives in its own files; `class ARS` is `partial`:
- `src\AutosportRacingSystem.TrackCreator.cs` — the dormant editor: `HandleTrackCreator`, `GenerateBezier`/`Bezier2`, `DrawRouteNodes`/`DrawSection`/`GetPerpendicular`/`PlayerOrCameraNearPos`, plus the mode flag and its knobs (`_routeEditorActive`, `_routeSection`, `_bezierStartAnchor`, `_bezierScale`, `_pathWidth`, `EditNodeHalfWidths`).
- `src\AutosportRacingSystem.TrackFile.cs` — the track XML writer: `UpdateRoute` (**live**, the `arsupdroute` cheat), `SaveRoute` (dead, zero callers) and `FindCustomProps` (sole caller is `UpdateRoute`).

Only location changed: `partial class` means no visibility plumbing and no call-site edits, and every moved block is byte-identical to the original. The loader (`TrackLoader`/`TrackRepository`) and the shared statics were deliberately left in place — a real class + interface split is the refinement pass's job, not the move's.

**Process lesson (paid for twice)**: when moving code mechanically, verify the moved text against the **original** (a git revision), never against your own insertion — a check that compares the inserted block to itself passes happily on wrong content (it did; the compiler caught it). And HEAD line numbers refer to the *committed* file, not to the line numbers an older inventory recorded.

**Prune status (2026-10, commit `626cbb1`)** — done, conservatively and on purpose:
- **Pruned**: the four half-migrated `TrackRepository` shims (`LoadXmlOrThrow`, `GetTrackTags`, `GetTrackStartPos`, `GetRacerModel` — each returned the migrated call on its first line with an unreachable body beneath, i.e. the migration's own leftovers), the unused `QuadraticBezier` helper, and the duplicated `src\TrackLoader.cs` csproj entry.
- **Deliberately kept — this is the important part, none of it is cruft**: the creator's own dead mass and the embedded `if (_routeEditorActive)` branches below are the **revival material** (deleting them now means rewriting them in the refinement pass, which is the opposite of why the code was isolated); the flare pipeline is documented as kept-for-rebuild; `GetSurfaceHash` together with the `Angles`/`TerrainGripMultipliers` fields reads as a **parked surface-grip experiment**; `Racer.UpdateRouteTarget` and its probe fields (`RouteTargetNode`/`RouteTargetRadius`/`ResetRouteProbe`/`RouteProbeSeconds`) are a superseded AI route probe, still intertwined with fields a future AI pass may want; and the dead `Options` members include the creator's **own menu entries** (`SaveTrack`, `CreateTrack`, `ExitCreator`). If a future session is tempted to "finish the prune", it should revive the feature first or ask — everything pruned or skipped is recoverable from git.

**What stayed behind (the original list, kept for that revival pass)**
- Six `if (_routeEditorActive)` branches still sit inside *live* methods (`OnTick`'s freecam-update and `TrackVisuals.DrawRoute` calls, one more `OnTick` gate, and three inside draw helpers). Each is dead but embedded, so removing them is a real edit, not a move.
- Shared statics the module reads but does not own: `RouteNodes`, `NodeHalfWidths`, `CustomProps`, `CurrentFile`, `FreeCamRide`, `_freeCam`, `DevSettingsFile`. A future class split must define ownership — prop ownership is already ambiguous (loader-spawned props are re-serialized by `FindCustomProps`, whose exclusion rules disagree with `SaveRoute`'s on radius and on which props to skip, and the `AutoGeneratedProps` list meant to mark loader-built props is never populated).
- Dead mass **inside** the moved code: `SaveRoute` (~200 lines), the write-only `_bezierScale` and `_bezierStartAnchor`, `_routeSection` (only filled inside the dead branch, so the "circuit closed" logic is inert), `SaveRoute`'s whole Trackside block (its `Model`/`Frecuency`/`Frozen` outputs have no reader, and the one attribute that *is* read feeds flare code behind an unconditional `return;`), and 15+ unreferenced `Options` enum members (`SaveTrack`, `CreateTrack`, `ExitCreator`, …).
- **Live bug in a live path**: `UpdateRoute` writes each node's `Wide` from the *previous* iteration (node 0 always writes 5) and clobbers the editor's `_pathWidth` knob on the way. `SaveRoute`'s equivalent loop is correct. Normalising this changes every re-exported track file — decide it deliberately.
- `SaveRoute` writes `Flares="false"` where `TrackLoader.ReadFlareColor` expects the documented 9-digit RGB form; the two disagree.
- The csproj lists `src\TrackLoader.cs` twice.

**Mutation policy — the first WIP release ships with NO track mutation (2026-10)**: no create, edit/update or delete. It is a property of the build, not a promise: the creator is unreachable (above), and the only live writer, `UpdateRoute`, has its sole entry point — the `arsupdroute` cheat in `HandleCheats` — gated with `if (1 == 2)` and a log line that says the cheat is ignored. `SaveRoute` has no callers. Verified by grep: every `Tracks\*.xml` write lives in `AutosportRacingSystem.TrackFile.cs`, and the only `File.Delete` in the codebase is SettingsRepair's own temp-file cleanup (unrelated). Re-enabling means removing that gate deliberately — and settling the `Wide` off-by-one first.

**Why it stays off — the streaming/memory audit (2026-10)**: a report of "textures stopped loading a few minutes after using `arsupdroute`" produced two findings, both worth keeping:
- **The focus mechanism is real but not ours.** The documented failure mode of a misplaced streaming focus is exactly that symptom — *"shadows disappear, textures go extremely low res"* when the focus entity is >300 units from the player — but the codebase **never sets a focus**: grep finds zero `SET_FOCUS_ENTITY` / `SET_FOCUS_POS_AND_VEL` / `IS_ENTITY_FOCUS`, and the only focus call anywhere is a lone `Function.Call(Hash.CLEAR_FOCUS)` at the end of `TrackLoader.BuildTrackLimits` — the *restore* direction, on the **track-load** path, not in `UpdateRoute`. So: if LOD/texture collapse follows *track loads*, that naked `CLEAR_FOCUS` (no matching set, and the native DB carries no description of it) is the thing to scrutinise; if it follows `arsupdroute`, focus is ruled out by grep.
- **The memory side has a real cause on that path.** `UpdateRoute(true,true,true)` → `FindCustomProps()`, which calls `World.GetAllProps().ToList()` and then, **per persistent prop**, `RouteNodes.OrderBy(v => prop.Position.DistanceTo(v)).ToList().First()` — it sorts *every route node to take the minimum*, allocating a sorted list + key array each time, synchronously on the script thread with no `Yield`, inside the save. Route nodes are 1 per metre, so a 3 km track is ~3000 nodes: on the order of 1000 props × 3000 nodes → tens of millions of comparisons and tens of MB of transient garbage per invocation. In a memory-pressured session that is a plausible trigger for the game's streaming loader to degrade, which fits a delayed effect. **Fix before re-enabling**: one O(nodes) nearest-node scan (or index props by node) instead of a sort — and note that changing it changes *which props get written*, so it is a semantics decision, not a drive-by optimisation.
- Unrelated but resolved while checking: the long-unidentified raw native `0x10D373323E5B9C0D`, called at the end of every track load and in `CleanEverything`, is **`BUSYSPINNER_OFF`** (ex `_REMOVE_LOADING_PROMPT`) — it just dismisses the bottom-right loading prompt. Benign.

**Revival (the refinement pass)**: with the code isolated, reviving creator mode is a deliberate design step — an entry point (cheat and/or menu item) plus a decision about what the feature should become — rather than restoring a hint for a cheat that never existed.

## Rear-end prevention — open ideas

**Rear-end prevention** (`ComputeTargetSpeed`): current live behavior (closest rival ahead within 6 m on roughly the same line, lateral offset within `r.CombinedSize.X`, speed blend saturating at 1 m, floored at `rivalSpeed`) is summarized in AGENTS.md; the corner-specific **Yield** maneuver is separate — it arms only when trailing a *faster* rival near a corner entrance **and overrides the speed blend locally**. No further open-idea notes existed in the original file beyond the summary itself (honest note, 2026-10 restructure).

## Details moved out of AGENTS.md (2026-10 trim)

AGENTS.md is the auto-loaded file and sits closest to its hard size cap, so its open-items list now carries one-line pointers and the full text lives here. Nothing was dropped in the trim — these are the entries whose detail used to sit in AGENTS.md in full.

### Start-line flares — disabled, and why
The flare prop/particle pipeline (`TrackLoader.SpawnGatePair` + both `AttachFlare`s) is disabled by an unconditional `return;` at each method top. The code is kept for a later rebuild, but the **placement/heading alignment must be fixed first** — the misalignment is why they were disabled. **The original disable was broken**: it used `if (1 == 2) return;`, which never returns, so the pipeline stayed live in every build until a grunt audit found it (the inverted-gate gotcha in AGENTS.md came from this). Unrelated: the Horsetrack `<Objects>` prop removal was *not* a flare. Also confirmed while checking: the creator's `Trackside` Model/Frecuency is written but has **no reader** — the trackside-prop feature is dead in the loader, and a point-to-point track would get a second flare pair at the finish if the pipeline were re-enabled.

### Stability awareness — partially implemented
Only the throttle-side rules are live: not all wheels on ground → reduce `MaxThrottle` (rates and floor in code); and steer angle exceeding the grip-based limit → the same reduction. The steer-into-airborne-side rule is **disabled** (`if (1 == 2)`-gated in `Racer.cs`). `AvgGroundStability` itself is hardcoded to 1 (see the per-racer bullet in AGENTS.md).

### Stuck recovery — one system, escalation included (verified 2026-11)
One system: the reverse-rock (steer straight, reverse throttle) for a fixed window; when the window ends it re-arms after the stuck-check time, alternating straight reverse with a steer toward the nearest track point. The old position-lerp-to-track escalation was removed — **but an escalation does exist**: after 5 failed attempts `ApplyStuckRecoveryOverride` teleports the racer to the nearest track edge, using the same math as the player's menu `ResetToTrack()` (nearest `TrackPoint`, lane side from `SignedLaneOffset`, heading along the track, ~10 mph forward), then clears its own stuck state. The 2026-10 "**planned** AI escalation, gated on `ResetToTrack` being player-verified" note was **wrong** — it was already in the code. User-verified in game for both player and AI (2026-11).

### TCS — the controller
A P-controller on `MaxThrottleFromTCS` targeting an ideal wheelspin: a tame fixed target off-track, more permissive on-track as the slide angle grows (values in code), with the output clamped well above zero — it never cuts below a floor fraction of throttle. Wheelspin is signed: negative = spin, positive = lockup.

### Steer-limiter throttle tie-in
When `ApplySteerLimits` actually cuts the steer (`_steerLimitedThisFrame`), `MaxThrottle` drops at a slow rate with a floor and recovers once the limiter disengages. **TODO: revisit the cut rate** — tuned to "feels right", with no grounded justification.

### Snap-oversteer counter — TODO, deferred to a dedicated session
Slide response is purely proportional (countersteer = slideAngle × scale); there is no derivative term to catch the *rate of yaw acceleration*. A snap-oversteer situation spikes yaw faster than a proportional correction can track — a D-term or a yaw-rate change threshold would catch it before the car snaps around.

### Gravity vs grip & speed — a design decision to set in stone
A TEMP experiment bakes the gravity multiplier into the base grip (`UpdatePerceivedGrip`). It feels good in-game but is **not settled**: `CurrentMechanicalGrip` also feeds sites that separately multiply by `Handling.Gravity` (route speed, corner speed, braking decel) — a possible double-count. The decision to make: define how gravity scales grip versus each speed/decel consumption site, principled rather than this temporary bake.

### Side-by-side heading assist — verified in game (2026-11)
Per steering update, for longitudinally overlapping rivals: mirror their live forward-heading angle, ramping from a small effect at wide lateral separation to the full effect near the combined vehicle width + buffer. The "unverified" flag simply predated the check — the user confirmed it in game and never reported it back.

### The reverse throttle path is gone from the pedal pipeline
No writer ever commands a negative `Intention.Speed`, so the `wantsReverse` branch in `ConvertSpeedToPedals` was dead code and was deleted — a negative intended speed simply brakes. The only reverse left in the AI is the stuck-recovery override.

### Nitro pacing — the charging rule
Every car is charged once at race launch and recharged on each lap increase (`Racer.NitroChargedLap` stamp, reset in `Initialize`; a mid-burn lap crossing retries until `IS_NITROUS_ACTIVE` clears). AI bottles are gated on `AiNitro != Never`; the player's own bottle is never gated; there is no free-roam refill. The charge target sits slightly above full so the below-full detection latch stays meaningful.

### Corrections are corrections — never reuse one as an absolute target
A value stored as a correction ("subtract this from the PD", like `_slideCountersteerDegrees`) carries the **opposite sign** of the steer direction it wants. Lerping the final steer *toward* it as a target steers **into** the slide — caught in-game 2026-09 (car sliding rightwards steered leftwards). When blending toward a slide-response target, reconstruct the full target (trajectory terms with the correction applied), never the bare correction value.

### Menu persistence — thorough in-game verification still pending (release gate)
The 2026-10 menu restructure (per-menu ini stores, Pace Mode under Settings, Reverse Route in the Race menu — Pace Mode has since moved *into* the Race menu, above Pace Offset, and Menu-Settings.ini is retired), the seed-on-read tidy-up and the dropped auto-migrations were only spot-checked. Before trusting persistence for a release, verify on both a fresh install and an existing one: delete each `Menu-*.ini` → it regenerates fully on load; toggles survive restarts; live readers still pick up store values. `5edc8b6` is the restructure commit; the settings-repair paths added afterwards *have* since been verified in-game (snap, fill, drop, create — see the repair evidence in AGENTS-TECHNOTES.md).
## Smart Tuning (the grid auto-tuner) - moved to AGENTS-SMARTTUNING.md

The tick-queue rationale, the two-livery-spaces trap, style taxonomy, colour precedence and whitelists, brand evidence and open items all live in that companion now.

