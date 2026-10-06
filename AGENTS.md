# ARS — GTA V Racing Mod

Source: `F:\Archivos Seguros\Mis Archivos\Codigo\GTAV\NewRacingSystem` · Target: C# / .NET Framework 4.8 / ScriptHookVDotNet 2

## This file is the agent's long-term memory
No memory survives between sessions, so this is the durable record: quirks, non-obvious decisions and their *why*, invariants, override order. **Aggressively save durable memories here**, but **do not explain systems — give the gist and a `file:line` pointer**; the code owns every value, so never record a constant, threshold or knob name, and never date a claim (anchor it to a commit hash or the code).

**Two sentences per concept, maximum.** DSH auto-loads this file and **truncates it at ~65 KB — the tail is what gets silently dropped**, so an addition must earn its bytes: pointers here, depth in a companion, and a block that outgrows a few lines moves out whole. **Every pointer carries its filename** (`Racer.cs:391`) so it can be jumped to and machine-checked; lines are anchors, not contracts, so grep the symbol if one misses.

## Companion memory files (NOT auto-loaded — read when the topic matches)
**Convention**: this file orients; a companion carries the depth. Each line leads with its **trigger vocabulary** — when a request, the code or the bug touches those words, open that file *before* answering.
- `AGENTS-DUEL.md` — **rivals, overtaking, side-by-side, cards, maneuvers, DiveBomb / DefendLane / Yield / ChillOut, nitro, time-to-apex**: the Duel model design. Status: designed, **not implemented**.
- `AGENTS-BACKLOG.md` — **"deferred", "later", "TODO", "open question", "did we fix X?", "already done?", open GitHub issues / triage**: every open item ranked simplest→most-complex, the Council backlog, pace theory, and detail trimmed out of here.
- `AGENTS-SMARTTUNING.md` — **liveries, paint, colours, cosmetic mods/parts, "Smart Tuning", Menyoo appearance, brand or mod naming**: the grid auto-tuner's design, colour rules, brand evidence, open items.
- `AGENTS-TECHNOTES.md` — **natives, hashes, memory offsets/pointers, rendering detail, ini key lists, settings-repair internals, UI/item inventories, formulas, crashes / minidumps / "did my change crash this?" / game won't load, "why was X removed?", cross-session hindsight, grid car selection / roster, sideloaded handling**: native/settings/menu inventories and everything the size trims moved out of here — **including the Dependencies & UI block and the Debug submenu inventory**.
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
- **Commit bodies carry no attribution of any kind** — not the driver, the author, an agent or teammate, or a third party: state what changed and why, never who found, reported, verified or asked for it. A **bibliography entry, a licence notice and a triage `@handle` are not attribution**, and evidence status ("driver-verified") is provenance of confidence, not credit. Nothing is pushed before its batch is **resolved** — driver-verified or explicitly killed — and the run of commits since the last push is the advancement gauge, read as commits per push; rewrite an unpushed message to fix what landed wrong rather than correcting it later.
- **Cutting large dead regions** (`6bce252`, −590 lines): exact-match edits are fail-safe, so drive `old_string` from a `line=length` map rather than counting blank runs by eye, and read each span first — a live method sat between two dead ones.
- **Surface the open items — session start, session end, and every push**: name what is ready for a decision (the top of `AGENTS-BACKLOG.md`'s ladder) instead of only filing it. Standing request from the user, who does not want to go looking for them.

## Agents: subagents and teammates
**Simple work is a `subagent`; planning a system is a teammate.** A `subagent` always runs and takes one self-contained deliverable — fetching, extraction, or a slice I do not want in context. A `spawn_teammate` teammate is durable, addressable and shares this filesystem, exists only when the user asks for one, and earns the round trip for **discussing and planning a system** (propose → argue → settle → apply) or for parallel writers on disjoint scopes; its task board, write scopes and arbitration are in `AGENTS-TEAMS.md`.
**The Council reviews work already done** — a commit range or the uncommitted work — as **two `subagent` calls on two different models**, and is **never dispatched automatically**; findings are summarized without applying changes. `spawn_teammate` takes no model route, so model diversity is always two `subagent` calls. Roles, pinned models and dispatch rules: the global `~\.dsh\AGENTS.md`.

## Build & deploy
- **The project auto-copies on build** (`PostBuildEvent` + `CopyArsDll`): Debug and Release both fire it, so *whichever builds last wins* — run Release last.
- **Build:** `& "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" NewRacingSystem.csproj /v:minimal /nologo /p:Configuration=Release`.
- **A rebuild does NOT need a game restart: the SHVDN reload binding reloads the scripts live** (`Insert` alone on this install — a combo value in that ini is read as its first token and reloads on every sprint), even though the in-game console is unavailable — so a test cycle is build → reload → drive.
**Moved to `AGENTS-TECHNOTES.md`** → "Build & deploy — toolchain, output, rollback": the dotnet form (never actually recorded anywhere — see the note there), the game output path and `ARS.dll`, the log truncation, the lag rule, the branch and rollback points, the dev build number. Triggers: deploy, dotnet, output, log, rollback, build number.

## Dependencies and UI

**Moved whole to `AGENTS-TECHNOTES.md`** — the declared ini shape, the menu invariants, the ship mirror and the LemonUI/API-dll staging rules are there.

## SHVDN build compatibility (release-critical)
- **SHVDN build compatibility** — the asi and the API dll are a matched build pair with no version check, and `VerifyScriptBridge()` must run before any native call; the install matrix, the SHVDNE alternative and the live-install ini quirk are in `AGENTS-SHVDN.md`.

## Code map
Full per-file detail: `AGENTS-TECHNOTES.md` → "Code map".
- `AutosportRacingSystem.cs` — race flow, track/corner generation, grid, leaderboard, helpers, static AI math. `class ARS` is **partial**.
- `AutosportRacingSystem.TrackCreator.cs` / `.TrackFile.cs` — the partial-class splits: in-game creator; track XML writer.
- `Racer.cs` — per-car intelligence: steering/speed pipeline, pressure, maneuvers, TCS, stuck recovery.
- `DataStructures.cs` — `RacerBrain`, `Rival`, `TrackPoint`/`Corner`, `VehicleControl`/`State`, `HandlingData`, `Maneuver`.
- `VehicleMemory.cs` — the single memory layer.
- `SmartTuner.cs` — the auto-tuner.
- `SettingsRepair.cs` / `MenuSettings.cs` — the declared ini shape and its store.
- `TrackLoader.cs` / `TrackRepository.cs` / `TrackVisuals.cs` / `GridBuilder.cs` / `VehicleCatalog.cs` / `VehicleSelector.cs` — track, grid and roster helpers.
- `MenyooAppearance.cs` / `UpdateChecker.cs` / `FreeCamController.cs` — the focused helpers.
- `PersonalitySet.cs` / `SkillSet.cs` — leftover per-racer scaffolding.

## Track, lane, speed and corner systems

**Moved whole to `AGENTS-STEERING.md`** — read it before touching the per-frame pipeline, the lane laws, the steering authority chain, the speed plan or the corner lifecycle. What the code owns is there; what stays here is the order and the gotchas.

- **Pipeline order** (`Racer.cs`): track position, then target speed, then steering, then the pedal reason caps and the pedals, then the recovery, then the **steer limits**, then the slew — the limiter closes the steering last on purpose so no writer above it escapes. `SteerLimitedSpeed` inside `ComputeTargetSpeed` therefore sees the previous core tick's post-slew steer; the overspeed arm gate inside `UpdateThrottleReasonCaps` sees instead the command `ComputeSteering` has just written — this tick's, pre-limiter and pre-slew — because the caps run after the steering and before the limiter.
- **Track convention**: node counts are treated as metres, circuit lookaheads use modulo and point-to-point clamps, and the test circuit measures about 1 m per node.

## Grid car selection (pace-matched)
- **Grid car selection** — selection is a ranking rather than a window, and the load path's threading contract (natives on the main thread only) is load-bearing; the pace score, roster and Force-Fill detail are in `AGENTS-TECHNOTES.md`.

## Sideloaded fleet handling
- **Sideloaded fleet handling** — ARS reads the live `CHandlingData` from memory, never the `Sideload\` resource files; the file of record and its flag-parsing path are in `AGENTS-TECHNOTES.md` and `AGENTS-FLAGS.md`.

## Per-racer state
**Moved to the companions**: the shipped cards, rivals and nitro are in `AGENTS-DUEL.md` → "Live per-racer state"; instability and the throttle cut in `AGENTS-STEERING.md` → "Instability"; the player-vs-AI engine divergence in `AGENTS-VANILLA-STEERING.md`. The rule of thumb stays.
- **Rule of thumb: the player and the AI follow the same rules.** When adding a mechanic, default to one code path for both; diverge only when the game engine forces it (e.g. the player's special ability, the AI's free ABS).

**Duel model (designed via Council, not implemented) — full design in `AGENTS-DUEL.md`.** Physics-aware card play on one shared time-to-apex primitive taken from each car's own live plan; **anything about rivals, overtaking or card play starts in `AGENTS-DUEL.md`.**

## Debug (LemonUI Debug submenu)

**Moved whole to `AGENTS-TECHNOTES.md`** — the toggle keys, what each visual owns and where the lane lines are drawn are there. The rules that hold: retiring a toggle retires its key, new toggles append, and the lane lines are drawn from `AutosportRacingSystem.cs` off the debug-focus racer, never the player.

## Leaderboard (frozen results board)
**Moved to `AGENTS-TECHNOTES.md`** → "Leaderboard - drawing and data detail": the freeze-on-crossing mechanism, the player's row and the finish block, plus the PI and best-lap columns. Triggers: leaderboard, results board, position freeze, PI column, best lap.

## Durable gotchas — do not "fix" these
Full explanations: `AGENTS-STEERING.md` → "Durable gotchas — steering and pipeline"; `AGENTS-TECHNOTES.md` → "Durable gotchas — code, helpers, rendering, settings". One hazard per line.
- Pursuit steer saturates at the clamp — there is no separate "recovery law".
- A descending-output `Remap` with `clamp` is inverted; NaN compares below everything.
- NaN discipline: `Clamp(NaN)` returns the min bound = full lock; never seed a getter with a sentinel.
- An early-continue guard is the accept filter negated — De Morgan turns its NaN case into a pass; keep the positive accept form.
- A null-check on a freshly `new`-ed object is dead code.
- Synchronous setup pauses `OnTick` and can blank per-frame debug visuals.
- `Handling.Downforce` reads `0x0014` — the code is right; don't restore the curve-fit.
- Oversteer ≠ sliding; a reference's blindness is a property of the reference, not of steering.
- The aim-error PID law was driven and ABANDONED — do not resurrect it blind.
- Offroad gravity needs its baseline reset before the multiply, or it stacks across races.
- Speed asymmetry is intentional; `SteerLimitedSpeed` is live and reads the post-slew applied steer, one core tick behind the limiter.
- `Options` enum values are positional — never persist or exchange one as an int.
- `World.DrawMarker` has a silent per-frame budget — long geometry belongs in `DRAW_LINE`.
- Prop `RightVector` and streaming ptfx — offset from the source direction; queue the entity.
- No old limiter reads the static TRlat; every limiter function has a caller.
- A POSITIVE steer command steers LEFT — a rename is a side swap, not a symbol swap.

## Open items — the decision-ready top

Every open item carries a stable kebab-case slug and its description lives in the ladder at the top of `AGENTS-BACKLOG.md`; a slug cited here describes nothing, so read it there. This section names only what a session start owes. **State tags** (`live`, `untested (hash)`, `driver-verified (hash)`, `DECIDED`, `not built`, `parked`, `(idea)`, `closed (hash)`) and slug keys are fixed in `memory-note-style.md` §"The conventions, fixed" — decode there.

- `dnf-final-death` — driver-verified (`0623276`)
- `rival-detection-route-frame` — driver-verified (`0623276`), residuals unobserved
- `player-special-ability`, `ai-free-abs` — parked
- `lane-repulsion-ceiling` — parked
- `crest-gate` — entry move driver-verified (`27ee097`), gating constants wait on the instrumented run
- `steer-ramp-band` — untested (`e59e0a9`, the band, the damper speed and the ceiling factor riding one drive)
- `ceiling-factor-review` — untested (`e59e0a9`); the owner owes the ceiling law a personal read, asked for
- `ceiling-factor-range`, `tcs-cap-floor-setting`, `overspeed-thresholds-setting` — three ranges the settings-extremes audit found constrained: widen the steer ceiling past 1.00, and expose the TCS floor and the overspeed thresholds
- `launch-slip-allowance` — (idea) more slip off the line, as a setting or a per-racer personality; the removed launch ramp last existed in `a6fcf7d`
- `rival-publish-core` — not built
- `apex-radius` — not built
- `slide-brake-rampdown` — parked
- **Pushed (`ac5514b..c2e7e6b`)**: everything held from the sessions before — the debug-visual train since `dc8ab14`, the slide-raise retirement, the ramp/damper batch, the overspeed ramp (`2dcbbcc`) and the lane base (`dd484d3`) — plus the four fix commits the two Council passes earned (`e59e0a9`, `57b3ac5` and their two doc commits). **Undriven in that run**: the ramp band and its ceiling factor, the overspeed ramp, the lane base and the Offshoot gate it inverted; the drives owed are the last section of `TEST-PLAN.md`.
- **Session artifacts**: `AUDIT-COMPUTE-ONCE.md` (the code-disagrees pile — fix it in the code, never annotate it), `PLAN-RATE-PRECISION.md` (rate and precision stages, the deferred perception manager, the route-frame ranking migration), `TEST-PLAN.md` (the outstanding drives); the flags reference's four unactioned items are in `AGENTS-FLAGS.md` and the corrections rule in `AGENTS-TECHNOTES.md`.

## Cross-session hindsight notes
- **Cross-session hindsight** — the culture leak, the crest behind-guard and the brake learner's threshold are in `AGENTS-TECHNOTES.md`.
