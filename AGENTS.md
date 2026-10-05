# ARS — GTA V Racing Mod

Source: `F:\Archivos Seguros\Mis Archivos\Codigo\GTAV\NewRacingSystem` · Target: C# / .NET Framework 4.8 / ScriptHookVDotNet 2

## This file is the agent's long-term memory
No memory survives between sessions, so this is the durable record: quirks, non-obvious decisions and their *why*, invariants, override order. **Aggressively save durable memories here**, but **do not explain systems — give the gist and a `file:line` pointer**; the code owns every value, so never record a constant, threshold or knob name, and never date a claim (anchor it to a commit hash or the code).

**Two sentences per concept, maximum.** DSH auto-loads this file and **truncates it at ~65 KB — the tail is what gets silently dropped**, so an addition must earn its bytes: pointers here, depth in a companion, and a block that outgrows a few lines moves out whole. **Every pointer carries its filename** (`Racer.cs:391`) so it can be jumped to and machine-checked; lines are anchors, not contracts, so grep the symbol if one misses.

## Companion memory files (NOT auto-loaded — read when the topic matches)
**Convention**: this file orients; a companion carries the depth. Each line leads with its **trigger vocabulary** — when a request, the code or the bug touches those words, open that file *before* answering.
- `AGENTS-DUEL.md` — **rivals, overtaking, side-by-side, cards, maneuvers, DiveBomb / DefendLane / Yield / ChillOut, nitro, time-to-apex**: the Duel model design. Status: designed, **not implemented**.
- `AGENTS-BACKLOG.md` — **"deferred", "later", "TODO", "open question", "did we fix X?", "already done?", open GitHub issues / triage**: every open item ranked simplest→most-complex, the Council backlog, key ownership, pace theory, and detail trimmed out of here.
- `AGENTS-SMARTTUNING.md` — **liveries, paint, colours, cosmetic mods/parts, "Smart Tuning", Menyoo appearance, brand or mod naming**: the grid auto-tuner's design, colour rules, brand evidence, open items.
- `AGENTS-TECHNOTES.md` — **natives, hashes, memory offsets/pointers, rendering detail, ini key lists, settings-repair internals, UI/item inventories, formulas, crashes / minidumps / "did my change crash this?", "why was X removed?", cross-session hindsight, grid car selection / roster, sideloaded handling**: native/settings/menu inventories and everything the size trims moved out of here — **including the Dependencies & UI block and the Debug submenu inventory**.
- `AGENTS-STEERING.md` — **steering controller, PID, P/I/D, pure pursuit, Stanley, aim point, lookahead, gain, feedforward, anti-windup, countersteer, opposite lock, oscillation, loop order, damping ratio, "why is it stable", yaw damper, course error, slip angle, Ackermann, steer authority, oversteer vs sliding, "how other games do it", per-frame pipeline, lane laws, speed plan, corner lifecycle**: the live chain analysed as a control loop, the external survey measured against it, **and the five system sections moved out of this file**.
- `AGENTS-SHVDN.md` — **SHVDN / ScriptHookVDotNet, SHVDNE / SHVDN Enhanced, install or compatibility, "the mod doesn't load", asi / API dll / version, game build, release requirements**: the asi↔API-dll matched-pair rule, the verified install matrix, `VerifyScriptBridge`'s ordering constraint, the release requirement, and the `Util\shvdn-compat\` and `Util\shvdne-lab\` labs.
- `AGENTS-WIP.md` — **first WIP, release, release blocker, test plan, packaging, clean install, artifact, release notes, pre-release verification**: the human acceptance and package gate, and user-facing limitations.
- `AGENTS-VANILLA-STEERING.md` — **vanilla / reference steering, "how does GTA steer", GTA AI bearing law, `CCarAI` / `FindMaxSteerAngle` / `CVehicleIntelligence`, `CVehControls` / `m_fSteerInput`, `SET_VEHICLE_STEER_BIAS`, driving-style flags, leaked source, "the real GTA V code does X", Franklin's special ability, per-character skills/stats, player-only assists, "does the player get more grip or damping"**: what the leaked engine + script trees do, and that their native hashes are **not** the ones to use.
- `AGENTS-FLAGS.md` — **vehicle flags, handling.meta flags, `strModelFlags` / `strHandlingFlags` / `strAdvancedFlags` / `strDamageFlags` / `strFlags`, `MF_` / `HF_` / `CF_` / `DF_` / `SF_` / `FLAG_` prefixes, `CAdvancedData` / `CCarHandlingData`, "what does this flag actually do", a car that behaves unlike its handling values**: every flag set the engine has, with a consumer site for each, the parse path out of `handling.meta`, the fleet's own flag profile, and the list of things ARS is blind to. Extracted from the leaked tree — a community flag list is not the authority here.
- `AGENTS-TEAMS.md` — **teammate, Agent Teams, Lead, shared task / task board, write scope, claim / complete, arbitration, dispatch, "discuss or plan a system", parallel writers, subagent vs teammate**: how delegation is routed here, and the write / claim / verify discipline a teammate must keep on this repo.

## Coding style
- Keep consecutive `&&` / `||` conditions on one line; do not break them across lines.
- Use `ARS.IsBetween` (`AutosportRacingSystem.cs:542`) for inclusive range checks.
- **Method names must be honest** — extra filtering/computing → refactor.
- **Do not use scripts to edit code files** — direct file tools only (scripts OK for XML/meta).
- **C# 7.3 ceiling** (no `LangVersion` override): no switch expressions, records, init-only properties, top-level statements or `??=`.
- **Comments are a failure signal**: if a line needs one, the code is not clear enough — rename, extract or restructure instead, and aim for none. The only comments that earn their place are an engine or leaked-source citation, a deliberate asymmetry or floor a reader would otherwise "fix", and a unit or sentinel that the name cannot carry. **Never restate the next line.**
- **Corrections require sources** — correct the user only when confident and after checking the code; if uncertain, say so.
- **Compute once, publish, consume under an explicit staleness budget** — a car reads the value another car or subsystem published rather than re-deriving it from that car's raw pose, and a consumer that cannot tolerate the published freshness states its own deadline instead of assuming. The route-frame hit test is the shape (`DataStructures.cs:200`); where the codebase still violates this is audited in `PLAN-RATE-PRECISION.md`.
- **FiveM reference tree hazard** (`S:\FiveM\server-data\resources\`): `[gameplay]\chat\` hides a huge `node_modules`, so any `-Recurse` walk from `resources\` times out. Target `[eddlm]\[ars-fivem]` directly.

## Workflow
- **Docs drift; the code wins** — check a claim against the source before relying on it, and fix the line you are touching rather than reconciling a file wholesale.
- **Scrutinize, never rubber-stamp.** When an instruction conflicts with physics, a known invariant or the code's behavior, say so *before* implementing; the user's standing rule is "call me out on these, never be a yes-man".
- **Ask when an instruction is open to interpretation** (thresholds, which rule it replaces, kept-vs-dropped) — a wrong guess costs a build cycle. **Ask in prose**; the multiple-choice ask tool is only for a decision that blocks progress, never for a design discussion.
- **Never fail silently.** A path not found, a drive unreachable, a search returning nothing — stop and tell the user immediately so they can help or redirect. Continuing blind wastes both people's time.
- **No 1.0 is scheduled** — ongoing updates, not a feature freeze; don't reflexively talk the user out of features, but keep changes modest.
- **Commit early, in coherent batches, without being asked — a commit is the only thing that survives the session.** That is the standing instruction, for code and docs alike: do not hoard work waiting for a milestone or a prompt.
- **A code change commits after a compile, and may commit before its drive if the body says so.** The verification state belongs in the message — compile-verified only, not yet driven — and the outstanding drive is tracked in `TEST-PLAN.md`; the human is still the verification gatekeeper and a successful compile is not "verified".
- **Documentation-only changes** may be committed without in-game verification if the build passes.
- **Act → the user tests → THEN document a result.** A plan, a decision or an audit is not a result and commits freely; never write a claim up as though it had been driven.
- **Stage files explicitly — never `git add -A`.** `Dist` mirrors the live install for tracks and the roster, so a blanket add sweeps session data into unrelated commits.
- **Commit per step** and grep for a key name after repointing it instead of trusting remembered call sites (a "fixed all four readers" claim was once wrong — two were missed).
- **Commit bodies carry no attribution of any kind** — not the driver, the author, an agent or teammate, or a third party: state what changed and why, never who found, reported, verified or asked for it. A **bibliography entry, a licence notice and a triage `@handle` are not attribution**, and evidence status ("driver-verified") is provenance of confidence, not credit. Nothing is pushed until asked, so rewrite the relevant unpushed message to fix what landed wrong rather than correcting it later.
- **Cutting large dead regions** (`6bce252`, −590 lines): exact-match edits are fail-safe, so drive `old_string` from a `line=length` map rather than counting blank runs by eye, and read each span first — a live method sat between two dead ones.
- **Surface the open items — session start, session end, and every push**: name what is ready for a decision (the top of `AGENTS-BACKLOG.md`'s ladder) instead of only filing it. Standing request from the user, who does not want to go looking for them.

## Agents: subagents and teammates
**Simple work is a `subagent`; planning a system is a teammate.** A `subagent` always runs and takes one self-contained deliverable — fetching, extraction, or a slice I do not want in context. A `spawn_teammate` teammate is durable, addressable and shares this filesystem, exists only when the user asks for one, and earns the round trip for **discussing and planning a system** (propose → argue → settle → apply) or for parallel writers on disjoint scopes; its task board, write scopes and arbitration are in `AGENTS-TEAMS.md`.
**The Council reviews work already done** — a commit range or the uncommitted work — as **two `subagent` calls on two different models**, and is **never dispatched automatically**; findings are summarized without applying changes. `spawn_teammate` takes no model route, so model diversity is always two `subagent` calls. Roles, pinned models and dispatch rules: the global `~\.dsh\AGENTS.md`.

## Build & deploy
- **The project auto-copies on build** (`PostBuildEvent` + `CopyArsDll`): Debug and Release both fire it, so *whichever builds last wins* — run Release last.
- **Build:** `& "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" NewRacingSystem.csproj /v:minimal /nologo /p:Configuration=Release`.
- **Via dotnet** (errors only, hides the SHVDN2 deprecation wall): see the `NewRacingSystem.sln` form in `AGENTS-TECHNOTES.md`. The VS install is **localised**, so filter its output on `: error `, not on "Build succeeded".
- Build output (game): `D:\SteamLibrary\steamapps\common\Grand Theft Auto V\Scripts\AutosportRacingSystem\`, and the artifact the game loads from there is **`ARS.dll`**; SHVDN's log is `ScriptHookVDotNet.log` in the game root.
- **`Scripts\AutosportRacingSystem\Log.log` is truncated at script init**, so an earlier session's evidence is gone — copy it out live when debugging.
- **A rebuild does NOT need a game restart: the SHVDN reload binding reloads the scripts live** (`Insert` alone on this install — a combo value in that ini is read as its first token and reloads on every sprint), even though the in-game console is unavailable — so a test cycle is build → reload → drive.
- Sub-200 ms lags don't matter; don't restructure call order to kill one-frame quirks.
- **Default branch is `master`** (the `2026` branch is redundant). Rollback points: **`54b33a7`** = last commit before the live corner-creation experiments; tag **`checkpoint-pre-gs-aware`** = before the Gs-aware preview steering experiment.
- **A dev build number auto-increments per compiled build** (`GenerateBuildNumber` in `NewRacingSystem.csproj`) and the init banner prints it beside the DLL's write time. It is deliberately **not** the assembly version — that stays the release identity and only moves by hand — and its `Inputs`/`Outputs` make it skip exactly when `CoreCompile` does, so it can never tick ahead of the DLL it labels. The counter lives in a gitignored `build\` so a build never dirties the tree; CI supplies its own via `-p:BuildNumber`.

## Dependencies and UI

**Moved whole to `AGENTS-TECHNOTES.md`** — the declared ini shape, the menu invariants, the ship mirror and the LemonUI/API-dll staging rules are there.

- **Two folder constants drive every path** (`ARS.ScriptsFolder`, `ARS.SettingsFolder`), and the `.csproj` mirror of `ScriptsFolder` is kept in sync by hand.
- **A setting read as a static applies only when assigned in its own `ItemChanged`**, and menu option lists come from their enum so only a renamed member needs a migration.

## SHVDN build compatibility (release-critical)
- **SHVDN build compatibility** — the asi and the API dll are a matched build pair with no version check, and `VerifyScriptBridge()` must run before any native call; the install matrix, the SHVDNE alternative and the live-install ini quirk are in `AGENTS-SHVDN.md`.

## Code map
- `AutosportRacingSystem.cs` — orchestration: race flow, track/corner generation, grid, leaderboard, helpers (`Remap`/`Clamp`/`Circumradius`), native wrappers, and the static AI math (`CornerApexSpeed` `AutosportRacingSystem.cs:2611`, `MaxSpeedForBrakingDistance` `AutosportRacingSystem.cs:2853`). `class ARS` is **partial**, with the two files below split out byte-verbatim.
- `AutosportRacingSystem.TrackCreator.cs` — the in-game track creator, **live**: the Track Creator submenu is wired into the root and the pool in `InitializeMenu` (`_arsMenu.AddSubMenu(creatorMenu)` `AutosportRacingSystem.cs:1204`, `_menuPool.Add(creatorMenu)` `AutosportRacingSystem.cs:1216`; the menu and its Start/Save/Exit items build at `AutosportRacingSystem.cs:836`). `StartTrackCreator` (`AutosportRacingSystem.TrackCreator.cs:25`) is the entry point and takes over the freecam, `HandleTrackCreator` (`AutosportRacingSystem.TrackCreator.cs:57`) records only while it is active, sections are **constant-radius circular arcs** (`GenerateArc` `AutosportRacingSystem.TrackCreator.cs:253`), tangent-continuous at the joints, replacing the quadratic Bézier whose radius was graded within a section and stepped at each joint.
- `AutosportRacingSystem.TrackFile.cs` — the track XML writer. `SaveRoute` (`AutosportRacingSystem.TrackFile.cs:29`) is **LIVE** behind the creator's Save Track, so creating a track does write `Tracks\*.xml`; the rewriting path (`UpdateRoute` and its `Wide` off-by-one) was cut as dead code (`c2ad2de`) — there is no update path. `SaveRoute` rounds coordinates to 2 decimals, coarse enough to perturb the measured `PreciseCurveRadius` — see the open item below.
- `Racer.cs` — per-car intelligence: the steering/speed pipeline, pressure, maneuvers, TCS, stuck recovery, debug drawing.
- `DataStructures.cs` — `RacerBrain`, `Rival`, `TrackPoint`, `CornerPoint`/`Corner`, `VehicleControl`, `VehicleState`, `HandlingData`, `Maneuver`.
- `VehicleMemory.cs` — **the single memory layer**: the control writes (`VehicleMemory.cs:19`), handling reads (`VehicleMemory.cs:31`) and the **memoized** AOB scanner they share (`VehicleMemory.cs:98`). A second copy of any of it is the defect the consolidation removed.
- `SmartTuner.cs` — the auto-tuner (`SmartTuner.cs:241` `Enqueue`, `SmartTuner.cs:259` `Tick`); design in `AGENTS-SMARTTUNING.md`.
- `SettingsRepair.cs` / `MenuSettings.cs` — the declared ini shape and its store.
- `TrackLoader.cs` (`TrackLoader.cs:175` `GenerateRouteInfo`, `TrackLoader.cs:228` `BuildApexTable`, `TrackLoader.cs:588` `BuildGridSlots`) / `TrackRepository.cs` / `TrackVisuals.cs` / `GridBuilder.cs` / `VehicleCatalog.cs` / `VehicleSelector.cs` — track, grid and roster helpers.
- `MenyooAppearance.cs` / `UpdateChecker.cs` / `FreeCamController.cs` — the focused one-purpose helpers.
- `PersonalitySet.cs` / `SkillSet.cs` — leftover per-racer scaffolding.

## Track, lane, speed and corner systems

**Moved whole to `AGENTS-STEERING.md`** — read it before touching the per-frame pipeline, the lane laws, the steering authority chain, the speed plan or the corner lifecycle. What the code owns is there; what stays here is the order and the gotchas.

- **Pipeline order** (`Racer.cs`): track position, then target speed, then steering, then the steer limits, then the pedals, then the steer slew.
- **Track convention**: node counts are treated as metres, circuit lookaheads use modulo and point-to-point clamps, and the test circuit measures about 1 m per node.

## Grid car selection (pace-matched)
- **Grid car selection** — selection is a ranking rather than a window, and the load path's threading contract (natives on the main thread only) is load-bearing; the pace score, roster and Force-Fill detail are in `AGENTS-TECHNOTES.md`.

## Sideloaded fleet handling
- **Sideloaded fleet handling** — ARS reads the live `CHandlingData` from memory, never the `Sideload\` resource files; the file of record and its flag-parsing path are in `AGENTS-TECHNOTES.md` and `AGENTS-FLAGS.md`.

## Per-racer state
- **Aggression** (grid-assigned by position, first lowest to last highest; the player stays mid) scales only the avoidance buffer — the old note that it scaled allowed TCS wheelspin is retired: `TcsCapLevel` reads no aggression. **Pressure** is proximity × aggression, rising slowly and falling quickly, and it drives divebomb/defend/arming.
- **Instability owns the throttle cut now, and it is a first cut (`Racer.cs:1556`)**: the 3 Hz `WheelsOnGround` latch, `AirborneThrottleLevel` and `MaxThrottleFromStability` are **removed**, and `IsUnstable` instead reads every tick, firing when the chassis rides above the at-rest height captured in `Launch` or when its yaw rate demands more lateral acceleration than the known grip allows. `CurrentMechanicalGrip` still carries no stability factor, and nothing models roll, pitch, load or suspension.
- **Maneuvers are cards** (`Maneuver` on the racer, no legacy arm blocks): a card **plays** into the slot and **folds** on its own condition, priority **ChillOut → DefendLane → DiveBomb → Yield**, with **Nitro slotless**. A played card is **never reconsidered mid-play** — unplaying causes thrash.
- **ChillOut** halves throttle and holds station behind the closest rival ahead until the field thins, arming only above a minimum speed so a slow car cannot become a roadblock. **DiveBomb** targets the closest reachable rival with a deeper braking target while its card lives.
- **DefendLane** covers the inside against a faster chaser, and **Yield** lets a faster overlapping rival by near the entrance. The corner-commit lane (`Racer.cs:727`) engages inside the outside-approach window and ignores the one-way hold latch and the per-corner positioning decision while the card lives.
- **Nitro** has four situations (contested / defended / lonely / finish-spender) behind vetoes on braking, off-track, big steer angle and low gear on high-geared RWD cars. **AWD is exempt** via the handling struct's drive bias; full rationale in `AGENTS-TECHNOTES.md`. The player fires via key press (`TryFireNitrous` `Racer.cs`), the AI via `TryPlayNitrousCard`; both share one shot per lap, the same charge cycle, and the same `SetOverrideNitrousLevel` native path.
- **Rule of thumb: the player and the AI follow the same rules.** When adding a mechanic, default to one code path for both; diverge only when the game engine forces it (e.g. the player's special ability, the AI's free ABS).
- **Passengerize** shifts AI drivers to the passenger seat while a rival overlaps, so they cannot swerve into contact, and is never applied to the player. **The ghosting removed in the avoidance cleanup is back as the No Collision option, untested (`0623276`)**: it clears rival detection outright and makes every racer pair pass through by calling the native in its permanent mode — the mode the removed ghosting got wrong.
- **The player's car and the AI's get different engine systems** — the special ability writes a real per-wheel grip multiplier and a faster steering ramp while it slows the world clock, `Automobile.cpp`'s player-only block adds a sideslip auto-centre, and the AI side gets free ABS and time-sliced wheel collisions; ARS neither detects nor blocks any of it (`AGENTS-VANILLA-STEERING.md`).

**Duel model (designed via Council, not implemented) — full design in `AGENTS-DUEL.md`.** Physics-aware card play on one shared time-to-apex primitive taken from each car's own live plan; **anything about rivals, overtaking or card play starts in `AGENTS-DUEL.md`.**

## Debug (LemonUI Debug submenu)

**Moved whole to `AGENTS-TECHNOTES.md`** — the toggle keys, what each visual owns and where the lane lines are drawn are there. The rules that hold: retiring a toggle retires its key, new toggles append, and the lane lines are drawn from `AutosportRacingSystem.cs` off the debug-focus racer, never the player.

## Leaderboard (frozen results board)
`DrawLeaderboard` (`AutosportRacingSystem.cs:1756`) is gated by its toggle, and **positions freeze per racer** on crossing the line via `LeaderboardFinish` (`AutosportRacingSystem.cs:201`) mirrored into `RacePosition`. Finishers draw in locked order, the player's row is yellow, and the finish block awards `RaceReward`, sets `RaceStatus = Finished` and calls `CleanEverything` (`AutosportRacingSystem.cs:1354`) — so the HUD only draws during Countdown/InProgress. Draw order is in `AGENTS-TECHNOTES.md`; the PI column **is** `VehicleData.TextPerformanceIndex`, written once per race and read only by that board (`AutosportRacingSystem.cs:2010`) — not a write-only field. `LapTimes` **is** read, as the best-lap column (`BestLap` `Racer.cs:1961`, drawn by `DrawLapStats` `AutosportRacingSystem.cs:2002`).

## Durable gotchas — do not "fix" these
- **The pursuit's steer saturates at the bearing it clamps to, and there is no separate "recovery law" to preserve** — `PursuitSteerFromBearing` (`Racer.cs`) is the `atan` of the pursuit's own sine term, so past the clamp the command would fold back down instead of flattening, and any saturation or crossover angle is a derivation off that form rather than a value the code holds. The `ARS.Clamp` NaN trap is already the NaN discipline bullet below, and the line cited for the "recovery law" (`Racer.cs:727`) is a corner-logging loop, not a clamp.
- **`if (1 == 2) return;` is an INVERTED gate — it disables nothing**, because the return only fires when the condition is true and it never is. Correct idioms are a bare `return;` or the block form; this bit the start-line flare disable (`bdd1b29`) for weeks.
- **Remap with a descending output range + `clamp=true` is inverted** by `Clamp` when `min > max`, and NaN compares less-than-anything. Keep output clamps ascending and use a descending *input* range for a reversed map.
- **NaN discipline**: `Clamp(NaN, -limit, +limit)` returns the min bound, i.e. instant full-lock — guard steering outputs and any clamp input that can be non-finite. Never hand a non-value sentinel to a *seeding* getter either, or it writes the sentinel into the ini.
- **A null-check on a freshly `new`-ed object is dead code.** Two "cannot find file" popups lived behind `if (new XmlDocument() == null)` while the real failures *throw*, so those paths have **no** error handling.
- Synchronous setup work can pause `OnTick` and temporarily suppress per-frame debug visuals; make it incremental if that matters.
- **`Handling.Downforce` reads `0x0014` in `VehicleMemory.GetDownforce` (`VehicleMemory.cs:34`) and the code is right** — an earlier note claiming `0x0010` was a transcription slip off an off-by-one source line number. With the right field the principled grip divider matches the engine's own formula, so don't bring back the old curve-fit hack.
- **Oversteer and sliding are different quantities, and what a steering law can see depends entirely on its reference vector** — never state a reference's consequences as general properties. The live pursuit bearing is velocity-referenced, which is the *blind* one: for a pure body rotation it reads as perfectly tracked **while the car is sideways**, and the slide authority is the blend instead.
- **The aim-error PID / measured-yaw-rate steering law was tried, driven and ABANDONED — the pre-PID chain was restored in preference to it.** Do not resurrect it blind; the durable lesson is that **it had no cross-track term at all**, so its standing line error could only be fought with P. Full record in `AGENTS-STEERING.md`.
- **Offroad gravity needs a baseline reset before the multiplier** — `Initialize` (`Racer.cs:323`) runs every race and on respawn, so setting gravity immediately before the offroad multiply is load-bearing or it stacks across restarts.
- **Speed asymmetry is intentional** (see the Speed pipeline) — don't "fix" it. **`Intention.SteerLimitedSpeed` is live** (`Racer.cs:1420`, blended 70/30 into `followTrackSpd`), and it is fed the *clamped* steer of the previous pass — the older note here claiming it was write-only was wrong.
- **`Options` enum values are positional**, so retiring a member renumbers the rest and one must never be persisted or exchanged as an int. Nothing does today, but re-check before any numeric consumer appears.
- **`World.DrawMarker` has a per-frame render budget**: markers past it are **silently not drawn** and draw order decides who loses, so long debug geometry belongs in `DRAW_LINE` instead. The budget is shared with every other marker the script draws that frame, so a visual can break because something *else* got greedy.
- **Two traps the flare rebuild exposed, both reusable**: a prop's own `RightVector` is perpendicular to *that prop's* forward, so an offset applied after its heading is turned 90° comes out parallel to the track — offset from the source direction instead; and a **named ptfx asset streams between frames**, so requesting it in a same-frame loop can never succeed — queue the entity and attach it from a tick (`TrackLoader.TickQueuedFlares`), logging a failed start rather than swallowing it.
- **Do not go hunting for an old limiter that used the static TRlat — the ceiling never reads it.** `Handling.LateralTractionCurve` is only the *input* to the speed-scaled peak (`LateralPeakAtSpeed`, `Racer.cs:900`) plus the countersteer gate thresholds, the slide blend and the TCS spin map, and every limiter function (`AckermannCeilingDegrees`, `GeometrySteerCeiling`, `PeakSlipCeilingAt`, `ResolveSteerCeiling`, `ApplySteerLimits`) has a caller.
- **A POSITIVE steer command steers LEFT — do not re-derive this.** `Racer.cs`'s own lane law gives it away (`laneSteerDeg = -laneErrorMeters * laneGain` against the `+ = right` **lane** convention, which is a different convention), the rival repulsion and corner-commit lane agree, and `TrackPoint.Angle` is already in the steer's convention (a left-hand corner is positive), so a reference built from it must be used as-is. `SteerLimitLeft` bounds positive commands and `SteerLimitRight` negative ones, which is what `Racer.cs`'s clamp now reads (`92d5867`), while the Show Inputs fan draws each limit on its true side — orange left, red right. A rename here is a *side* swap, not a symbol swap, so check every use by side rather than by name.

## Known TODOs / open items
One line each; **a simplest→most-complex ranking of the whole list sits at the top of `AGENTS-BACKLOG.md`**, and the detail for each is in `AGENTS-BACKLOG.md` or `AGENTS-TECHNOTES.md`.
- **Recovery redesign — landed untested (`0623276`)**: it reverses, drives out, and snaps; the drive is `TEST-PLAN.md` and the detail is `AGENTS-BACKLOG.md`.
- **DNF is final death — untested (`0623276`)**: parks on the shoulder or drops under the track, hidden from detection behind its option, and the finish count, the rewards and recovery all subtract it (`Racer.cs`).
- **Rival detection is route-frame — untested (`0623276`)**: arc gap, along-track closure, track-relative corridor (`Rival.ComputeTimeToReach`). **Residuals: a stopped car on a hairpin's opposite leg, and the off-track node rescan on a stacked deck.**
- **The code-disagrees-with-itself pile is code work, not memory** — values that exist twice, unreachable branches and write-only fields are inventoried in `AUDIT-COMPUTE-ONCE.md`; fix them in the code rather than annotating them here.
- **Rate and precision are planned, not built**: stages, the deferred perception manager and the route-frame ranking migration are in `PLAN-RATE-PRECISION.md`.
- **The track creator is LIVE and driver-verified**; `SaveRoute` writes `Tracks\*.xml` and there is no update path — the rewrite path was cut in `c2ad2de`, so reviving it means restoring it from git (`AGENTS-BACKLOG.md`).
- **`SaveRoute` rounds coordinates to 2 decimals**, which alone shifts a perfect arc's measured `PreciseCurveRadius` low; raising the precision rewrites every saved file, so decide it deliberately.
- **Apex speed reads the biased radius** (`SupposedRadius`, the noise tail) while the steadier `DetectedRadius` feeds the positioning gate and merge survival; moving the speed onto the smoothed radius is a behaviour change wanting its own drive (`AGENTS-BACKLOG.md`).
- **A constant-radius generator has no radius to open**, so a creator-built corner's span comes from the detection gate or the next section, not from the corner — treat the gate as the lever (`AGENTS-BACKLOG.md`).
- **Preview the apex table inside the creator (idea)** — the merges and blips visible while laying a track out, not only after saving.
- **Place apexes by hand (idea)** — sidesteps an apex that is arbitrary along a constant-radius arc.
- **A surviving merge does not reach its own first corner's entrance** — its span starts after the absorbed apex (Figureight's absorbed one is a 112 m bend), inert while that corner needs no braking but re-opening the slow-first-corner case if a braking-worthy one falls inside the window.
- **The two corner-merge rules need a stress test** — the short geometry-blind window and the longer same-direction one sit close together, and the aligned window is validated on one track only, so a genuine kink at 3.5-4 s elsewhere is the case to watch: it splits, the safe direction, at the cost of the pair's absorbed lead-in (`AGENTS-BACKLOG.md`).
- **Start-line flares are LIVE and driver-verified (`4779a31`)** — one pair per track, on the last node at the track edges, the effect burning in the colour the track's `Flares` attribute asks for (`false` = off); the creator's `Trackside` Model/Frecuency props still have no reader.
- **Council review backlog**: what remains is MenuSettings save-per-scroll, per-tick store reads in `Racer.cs`, and the listed minors.
- **Pace is model-theoretical, not instance-measured** — the player's tuned car is paced at its stock number; an upgrade-set multiplier table is the idea on file.
- **Electric pace sits deliberately at the ramp's peak (`f3fc274`)** — the fastest electrics outscore every ICE model, so they cannot be selected against an ICE target; `96f5a72`'s midpoint is the one-line revert if that bites.
- **Corner approach tied to the braking plan (idea)** — in tension with a shipped principle: the corner line deliberately keeps grip out of the lane (`ComputeCornerTargetLane` `Racer.cs:798`).
- **Steering batch residuals** — the countersteer allowance is still twice the half-slide target, the slip-balance knee has never had a drive, and the damper's crossing is still a share of a grip-scaled excess (`AGENTS-BACKLOG.md`).
- **The off-road gravity flag's fix is DECIDED** — gravity carries the multiplier and the effective grip stays the pure coefficient, both published for every consumer; the bake in `UpdatePerceivedGrip` goes and the hard-coded gravity sites read `Handling.Gravity`. Offroad cars end up slower through corners and braking later, so it wants its own drive (`AGENTS-BACKLOG.md`, `AGENTS-TECHNOTES.md`).
- **The flags reference leaves four things unactioned** — rally tyres' opposite grip curve, the spoiler flag's downforce, unmodelled camber and off-throttle friction, and `m_AdvancedData` (`AGENTS-FLAGS.md`).
- **Instability is tuned by driving, not derived** — the ride-height margin, yaw tolerance and min speed are guesses, roll and pitch rates are fetched then thrown away, and nothing models load transfer (`Racer.cs:1556`).
- **The overspeed correction is due a re-look after its drive** — a coarse ladder with no proportional region, a 3 Hz refresh against a per-tick glide, and an uncompensated grade term, and `wheelGs` is a *reported* drive power rather than a traction-limited force (`Racer.cs:3174`).
- **TCS is a reason cap, not a P-integrator (`1ebbd2c`)** — proportional under curve-anchored targets, and the steer-limiter tie-in was removed as a no-op.
- **Low-grip gate SATISFIED (driver-verified)**; one unverified number sits under it — the grip native was never checked against measured lateral g.
- **Snap-oversteer counter** — the damper has no spike detector, and a snap spikes faster than a proportional term can track.
- **The yaw damper is referenced to the AIM POINT now**, and the earlier zero-reference drive was confounded; if it ever rings the lever is a scalar on the reference, **not** `SteerDampingGain` (`AGENTS-STEERING.md`).
- **The low-speed ramp's end target reads a different peak than its ceiling** (`Racer.cs:949`) — a one-line fix if it ever matters.
- **The player's special ability and the AI's free ABS are open decisions** — an advantage the AI cannot match, and anti-lock brakes the engine grants one side (`Automobile.cpp:3774`) with no native to query (`AGENTS-BACKLOG.md`, `AGENTS-VANILLA-STEERING.md`).
- **Overrotation via the pedal (idea)** — the pedal is the lever on rear grip, and the sign flips between power and load-transfer oversteer (`AGENTS-BACKLOG.md`).
- **The corrections rule** — never blend toward a correction value as if it were a target; it steers into the slide.
- **Reverse throttle path** is removed by design, and nitro is charged at launch and per lap with no free-roam refill.
- **Two-projection route speed (future idea)** — a ballistic plus a pessimistic projection.
- **Off-track projection → reaction is closed (`67d8da7`)**, re-checked in game.
- **Immersive join points** — a chevron marks the nearest start line and the join follows Pace Mode.
- **Optional update checker as a separate DLL (idea)** and the rear-end prevention notes live in the companion.
- **Slide brake rampdown** returns at v0.9+: re-derive it against the current steering ceiling, since its absence is the full throttle while sliding (`AGENTS-BACKLOG.md`).

## Cross-session hindsight notes
- **Cross-session hindsight** — the culture leak, the crest behind-guard and the brake learner's threshold are in `AGENTS-TECHNOTES.md`.
