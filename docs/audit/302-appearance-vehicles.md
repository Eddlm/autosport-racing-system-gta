# AUDIT 302 — appearance and vehicles: the surfaces no report opened

**Surface**: `src/SmartTuner.cs` (menu label *Auto Styling*), `src/MenyooAppearance.cs`, `src/VehicleSelector.cs`,
`src/VehicleCatalog.cs`, and the four call sites in `AutosportRacingSystem.cs` that drive them — `SmartTuner.Tick`
(`:1887`), `SetupRace`'s `Enqueue` (`:2637-2641`), `SpawnRacer`'s `MenyooAppearance.Apply` (`:4150`), and the two
`FillCachedCandidates` paths (`:377` load, `:1557` grid). Not re-opened: anything `201`–`212` already carries.
`grep -l` over those twelve returns `MenyooAppearance`, `VehicleSelector` and `VehicleCatalog` **nowhere** and
`SmartTuner` only in passing — this is a read from zero.

**Date**: 2026-10-26, 01:14.
**Build audited**: **1105**. `build/build-number.txt` and the deployed
`Scripts\AutosportRacingSystem\build-number.txt` both read `1105`; `bin/Release/ARS.dll` and the deployed
`ARS.dll` are one file, sha256 `0e3b2d13ec0f7b09eedc347f3cb16369174e1b7c709e7745baae2a6a8a261a65`; source
`HEAD 6a1fcbc`, tree clean except `Dist\...\Tracks\Figureight.xml` (session data) and untracked `docs/`.
⚠️ **The card says 1103; the tree and the install both say 1105.** Nothing here rests on which of the two.
**Method**: read-only. Source read at the lines quoted; `libs/ScriptHookVDotNet2.dll` decompiled with
`ilspycmd` wherever a claim depends on the API rather than on ARS; the leaked engine tree
(`E:\GTA\GTAVSP\GTAV Source\src\dev_ng`) read for what the natives themselves do with a value; one replica
measurement run in the scratch directory; the live install read for the log level and the roster. Nothing was
edited, built, deployed, committed or driven, and the only file written is this one.
**Verdict**: **no blocker.** The surface is cosmetic helpers, and the two shapes that look like blockers both
close on evidence: a Menyoo file's out-of-range slot/value is bounded by the **engine's own** `SCRIPT_VERIFY`
(A5), and a dead or empty roster is caught before the race (A4, `12-F13`). What is here: **four release-note
rows and ten file rows**, of which the sharpest are the cross-writer collision between Menyoo and Auto
Styling (A3), a menu description that misstates its own skip rule (A1), and a *wrong on-screen reason* for a
refused race (A4).

---

## Direct answers to the four questions the card asks

**What SmartTuner writes, and what it restores.** Writes, per car: `InstallModKit` (`:286`), the livery — mod
slot 48 or the older livery list (`:295-299`), the style's cosmetic slots at 75% each (`:342-348`), and four
colour slots (`:403-406`). Restores: **nothing, anywhere** — `grep -rniE 'restor|revert' src/` finds no undo
path, no snapshot, no teardown hook. The look is a property of the car entity for as long as the car lives,
which includes a car the player takes over, because `Racer.Delete` deliberately keeps any car the player owns or
is sitting in at teardown (`Racer.cs:5312-5319`, read here: "a player-owned racer keeps its vehicle").

**Player changes car with the tuner applied.** The "your own car is never restyled" rule is decided **once, at
enqueue** (`:235`, `IsInVehicle` on the player's car) and never re-tested: `Tick`'s guard is
`!SmartTuning || !CanWeUse || !Exists` (`:251`) with no player test. So a car entered *after* the enqueue but
*before* its own tick is restyled under the player. The window is one car per tick — ~0.3 s for a 20-car field
— so it is narrow and I did not reproduce it (PROBABLE; not filed as its own row, since nothing observed
depends on it). The reachable half is the other direction: the
queue is drained by the time the player can move, so what the player actually inherits is a *finished* AI car
with a permanent look and random mods, whenever they keep one.

**MenyooAppearance: reads, touches, and what it leaves half-done.** Reads
`<game root>\menyooStuff\Vehicle\**\*.xml` (`:16`, `:31-42`), matching on `/Vehicle/ModelHash` alone (`:64-72`);
the root is derived from `ScriptsFolder` two levels up. Touches: the six colour slots, livery, wheel type,
window tint, every `Mods` child, and the neons (`:111-131`). **Leaving the menu has no bearing on it** — the
pass is not menu-driven, it runs once per spawned car (`:4150`). The real partial-application surfaces are
narrow: a toggle the file turns *off* is discarded (A8), and a car whose driver ped fails is deleted with the
appearance work already spent on it (`:4153-4158` — wasted work, no leak). A throw mid-apply is **not**
reachable: every setter it calls is a bare `Function.Call` on the SHVDN side and cannot throw, so the
`SetValue` without a `try` (`:142-148`) is not a half-applied path (recorded below).

**VehicleSelector/Catalog read path.** *Empty pool* → zero candidates, the race refuses and says so on screen
(`12-F13`, and A4 for the case where that message is the wrong one). *Unknown model* → the line survives to the
pool as a key with no stats (`:95` writes the key before validating), so the pace path skips it silently
(`TryGetValue`, `VehicleSelector.cs:21`) and the log's own "N of M paced models" counts it in neither bucket
(`VehicleSelector.cs:29`). *A car that no longer exists* is the same case; a model cannot disappear mid-session once indexed.
Two more silent outcomes exist purely by the **form** of the hand-written line: a duplicate key (A6) and an
unsigned hash (A7).

---

## Findings

Severity: **blocker** = must change before the tag; **release-note** = true as shipped, needs a decision or a
line in the notes; **file** = a defect or a cost worth recording in a tracked file. **Blockers: none.**
Rows are 302's own numbering.

| # | Sev | Claim | Evidence | Status | How to reproduce |
|---|---|---|---|---|---|
| **A1** | release-note | ⚠️ **The Auto Styling menu text is wrong in both clauses, and believing it would break the feature.** It promises "skipping your own and any car that already has a mod kit"; the skip test is `GetMod(VehicleMod.Engine) != -1` — a non-stock **engine mod**, not a mod kit — and **every car ARS spawns already has a mod kit installed** one statement after it is created. A literal implementation of the description would skip the whole grid. | `AutosportRacingSystem.cs:1007` (the text), `SmartTuner.cs:236` (the test, with its own comment "leave cars that are already tuned alone"), `AutosportRacingSystem.cs:4143` (`car.InstallModKit();` in `SpawnRacer`), `SmartTuner.cs:286` | VERIFIED (code) | Read the Rivals → Auto Styling tooltip; compare with `SmartTuner.cs:236` and `:4143`. |
| **A2** | release-note | **The one line that would tell you whether Auto Styling ran is a no-op on a shipped install.** It logs at `Info`, the shipped `LogLevel` is `Error`, and `Log` returns before writing. `AGENTS/AGENTS-SMARTTUNING.md:24` calls that line "deliberately self-diagnosing"; at the shipped level the feature is unobservable, which is the first question a tester asks. The same muting makes the tuner's own "queued for N cars" line count **racers, not queued cars** (`:2639` prints `Racers.Count` while `Enqueue` filters out the player's car and every already-tuned car), so that number can be published as 21 when the queue holds 5, or none. | `SmartTuner.cs:301` (the only per-car line), `AutosportRacingSystem.cs:205` and `SettingsRepair.cs:169` (default `Error`), `:3765` (the early return), live `Menu-Settings.ini:27` = `LogLevel = Error`, live `Log.log` = 82,928 B with **0** `Smart tune` lines; the count-vs-queue mismatch: `:2639` vs `SmartTuner.cs:232-238` | VERIFIED (code + live file) | On any install at the shipped level: `grep -c "Smart tune" Log.log`. |
| **A3** | release-note | **The two appearance writers collide, and which one wins is decided by a slot the Menyoo file may or may not carry.** Menyoo applies at spawn (`:4150`); Auto Styling queues later, at `SetupRace` (`:2637-2641`), and skips any car carrying an engine mod. So a Menyoo file that includes engine slot 11 → Auto Styling declines the car and the Menyoo look **stands**; a file that carries paint only → Auto Styling takes the car and lands last, so its paint **overwrites** the Menyoo paint. Both toggles default on, so the shipped configuration is the one where they meet, and neither outcome is logged (`:301` is the only tuner line, and it is muted — A2). The player-visible symptom is that some cars keep their Menyoo look and others do not, with no rule a player could state. | `AutosportRacingSystem.cs:156` and `:160` (both default `true`), `:4146-4151` (Menyoo first), `:2637-2641` (Auto Styling after), `SmartTuner.cs:236`, `:403-406` (paint last, so it wins on the cars it takes) | VERIFIED (the mechanism, and that both default on); **PROBABLE** that a real Menyoo file carries slot 11 — not checkable here (see "Not verifiable") | Two Menyoo files for one model, one with an engine mod; spawn both. |
| **A4** | release-note | **A race can be refused for the track's geometry and the player is told it was the car roster.** `GridSlotsAvailable` is `Max(0, GridPositions.Count - 2)`; a **circuit** only ever fills it for a route longer than 50 nodes, so a shorter circuit yields zero slots → `gridSize` 0 → `FillCachedCandidates(0)` returns an empty list (the selector breaks on `candidates.Count >= 0`) → the refusal message names "the Vehicles folder". The true cause is logged one line earlier at `Info`, which the shipped level suppresses, so the player has no correct information on screen or in the log. `12-F13` covers the *roster-empty* case; this is the second cause of the same empty list. | `AutosportRacingSystem.cs:271` (`GridSlotsAvailable`), `:1554-1556` (the cap; `:1556` logs the truth at `Info`), `:1563` (the wrong `Notify`), `TrackLoader.cs:1218-1225` (`BuildCircuitGrid`: `for (int i = nodes.Count - 1; i > 50 && …)` — zero iterations below 52 nodes), `VehicleSelector.cs:37` | VERIFIED (code + arithmetic) | Hand-author a circuit of 20-50 nodes, load it, Race → Start Race: "No vehicles found with a performance index". |
| **A5** | file | **A wild slot or value out of a Menyoo file is dropped by the engine, not by ARS — there is no crash and no memory-corruption path, and nothing is logged.** `ApplyMods` forwards the XML node number and the parsed value with a single `slot < 0` test, and SHVDN2 validates **nothing** (its `SetMod`/`ToggleMod`/`WheelType`/`WindowTint` setters are bare `Function.Call`s). The engine is what bounds it: `ConvertVehicleModTypeFromScript` is total arithmetic with three branches and no indexing, and three `SCRIPT_VERIFY`s reject anything outside the mod/toggle/wheel domains *after* the conversion. The worst a hostile or mistyped file achieves is a **different but valid** slot — script slot 60 converts to engine 46, which `SET_VEHICLE_MOD` explicitly allows (hydraulics). | `MenyooAppearance.cs:156-177` (the parse and the only guard), `libs/ScriptHookVDotNet2.dll` decompiled (`Vehicle.SetMod`, `ToggleMod`, `set_WheelType`, `set_WindowTint` — no validation in any), leaked `game/modelinfo/VehicleModelInfoEnums.h:220-228` (the conversion), `game/script/commands_vehicle.cpp:11158`, `:10951`, `:11247` (the three verifies) | VERIFIED (decompile + leaked source) | Point a file's `<_60>` at a car: no crash, no log line, hydraulics change. |
| **A6** | file | **Two lines that mean the same car become two entries, so the grid can field one model twice.** `FillPool` de-duplicates on the **raw line text** (`OrdinalIgnoreCase`), then `BuildPowerCache` rewrites each entry in place to its canonical key — and its `seen` set only skips the *stats read*, not the entry. So `sabregt` and its signed hash `-1685021548` both leave the pass as the identical key, twice. The closest-N fill adds both, and the stable sort keeps them adjacent, so both land in the candidate list: one duplicated car and one **fewer distinct model** than the pool holds, while `:29` reports it as two covered models. | `VehicleCatalog.cs:58-64` (text de-dupe), `:94-96` (`pool[i] = key` then `seen.Add` — the skip is the read, not the entry), `:121` (only nulls are removed), `VehicleSelector.cs:20-26` and `:32` (stable `OrderBy` keeps the twins adjacent); joaat(`sabregt`) = `0x9B909C94` → signed `-1685021548` | VERIFIED (code + computed pair) | Add `sabregt` **and** `-1685021548` (its own signed hash) as two lines of one roster, then read `Closest-pace candidates` at `Info`. |
| **A7** | file | **An unsigned hash line is silently reinterpreted as a model name.** `int.TryParse` rejects everything above `int.MaxValue` (measured), so the overflow falls through to `Game.GenerateHash(<the digits>)` — which hashes the literal text, leaving a key that matches no model. The line stays in the pool forever, matching nothing, with nothing logged; `CanonicalKey` does the same for the roster-add path. ARS itself always writes the **signed** form, so the trigger is a hand-written line — and the unsigned form is what a hash list on the web shows. Consequence is inert (a phantom key, per `12-F13`); the defect is the silence: the car simply never appears. | `VehicleCatalog.cs:92` (`int.TryParse` → `GenerateHash`), `:126-131` (`CanonicalKey`), measured: `[int]::TryParse("2609945748")` = **False**, `"2147483647"` = **True**, `"2147483648"` = **False**, `"-1685021548"` = **True**; signed form produced by `:94` and `AutosportRacingSystem.cs:561` | VERIFIED (code + measured arithmetic); **PROBABLE** occurrence (needs a hand-written line — the live `cars.txt` is 828 name-only lines, 0 numeric) | Write `2609945748` (sabregt, unsigned) in a roster and select it: the model never spawns and no line says why. |
| **A8** | file | **A Menyoo file can switch a toggle on but never off.** The `Mods` walk reads a `true`/`false` child, calls `ToggleMod(…, true)` for "true", and `continue`s on "false" — so a file that records "no turbo" cannot clear a turbo already fitted. Harmless on a fresh spawn, wrong on a car that arrived with one. | `MenyooAppearance.cs:161-165`; the toggle domain the cast lands in: `commands_vehicle.cpp:11247` (`modSlot >= VMT_TOGGLE_MODS && modSlot < VMT_MISC_MODS`) | VERIFIED (code) | A file with `<_18>false</_18>` against a car that already has turbo: unchanged. |
| **A9** | file | **The Menyoo read is `O(cars × files)` with a full XML parse of every file for every car, and no early exit.** The file *list* is cached for the session; the parse is not — `FilesMatching` re-opens and re-parses the whole directory per spawned car, collecting *all* matches rather than stopping at the first. Measured on a replica of 120 Menyoo-shaped files: **26 ms for 1 car, 137 ms for 5, 492 ms for 20, 965 ms for 40 — ≈25 ms per car, ≈0.2 ms per file, linear in both**. A 500-file collection is therefore ≈2.1 s of grid-build time at 20 cars, in ~25 ms blocks. The normal path hides it: `InstanceGrid` fades the screen out first. | `MenyooAppearance.cs:18-27` (the cached list), `:74-89` (the per-car parse, `SearchOption.AllDirectories`), `:98`; caller `AutosportRacingSystem.cs:4150`; replica in the scratch directory (scratch files, deleted after the run); the fade: `:1536` | VERIFIED (code + measured replica) | Scale by the reader's own `menyooStuff\Vehicle\*.xml` count × 0.2 ms × grid size. |
| **A10** | file | **`InvalidateCache` has no caller, so the Menyoo list is built once per session.** The only occurrence of the name in `src/` is its own definition; the cache is populated lazily on first use and never refreshed. A Menyoo file saved while the game is running cannot apply until a reload — with the method that exists to fix exactly that sitting uncalled next to it. | `MenyooAppearance.cs:29` (the definition, the only `grep -rn` hit over `src/`), `:18-27` (the sticky `_files`) | VERIFIED (grep) | Save a Menyoo file mid-session, spawn that model: no effect until reload. |
| **A11** | file | **`SmartTuner.Clear()` has no caller either, so the pass has no stop other than its own guards.** The queue is emptied only by the *next* `Enqueue`; race teardown, abort and `CleanEverything` do not call it. Because the queue drains at one car per tick the stale-queue window is ~0.3 s, so the live half of this row is the dead method itself — the tuner can be switched off mid-queue but not turned off by the teardown that removes the cars it is working on. | `SmartTuner.cs:241-244` (definition; the only `grep -rn "SmartTuner" src/` hits are `:1887` `Tick`, `:2640` `Enqueue`, and the class declaration), `:231` (`Enqueue`'s own `Clear`), `AutosportRacingSystem.cs:1818-1843` (`CleanEverything` — no call) | VERIFIED (grep + code) | Switch Auto Styling off during the countdown: the queue keeps draining, guarded per car by `:251`. |
| **A12** | file | **`HardcodedRoster` is never assigned, so `SelectHardcoded` and the whole hardcoded branch are unreachable.** The field is declared `null` and read once; nothing sets it, so the "full-roster test bypass" cannot be entered. Recorded with a caution rather than a removal: if it were ever populated it bypasses both the vehicle-class blacklist and the pace index, and every dead key in it would cost up to **500 × `Script.Wait(10)` = 5 s** inside the grid build, since the load loop has no early exit for a model that is already known invalid. | `AutosportRacingSystem.cs:700` (declaration), `:3971-3973` (the only read; `grep -rn "HardcodedRoster" src/` returns exactly these two sites), `VehicleCatalog.cs:102` (where the class filter lives), `AutosportRacingSystem.cs:4038-4047` (the 500-try wait) | VERIFIED (grep) | — (unreachable as shipped). |
| **A13** | file | **`RefreshCatalogs` mutates the vehicle pool from the menu on the main thread while the initial load task may be inside the same mutation on the background thread.** Its own header states the contract "Runs on the background load task"; through `RefreshRoster` it is also reached from the Car Roster item and both `arsbuild*` cheats, i.e. the main thread. `FillPool` is `Clear()` plus an `Add` per line with no lock, so a pool change inside the initial load window is a concurrent mutation of one `List<string>` — the failure mode the same file warns about for `Log` (`12-F10`). Narrow window, never observed. | `AutosportRacingSystem.cs:506-508` (the comment), `:518` (the mutation), `:565-570` (`RefreshRoster`), `:987`, `:2798`, `:2822` (the main-thread callers) vs `:353-357` (`Task.Run`); `VehicleCatalog.cs:56-64` | PROBABLE (reachability — needs a menu change during the load; mechanism VERIFIED) | Launch, open Rivals → Car Roster before "ARS has loaded" clears, and change the pool. |
| **A14** | file | **Three log lines call models "files".** Each prints the pool's **model** count as a file count, which is the count a driver reporting a bug will quote back. | `AutosportRacingSystem.cs:520` ("Found N vehicle files"), `:1551` ("ranking N pool files"), `:3983` ("for N pool files"); the value is `_vehiclePool.Count` in all three | VERIFIED (code) | Compare the printed number with `ls Vehicles\*.txt` — 2 files, ~470 models. |

---

## Recorded so nobody "fixes" it — each checked here and closed

- **SmartTuner's slot table is engine-correct.** SHVDN's `VehicleMod` **is** the script enum, and every member
  the table uses converts through `ConvertVehicleModTypeFromScript` onto the intended engine slot — including
  `Livery = 48` → `VMT_LIVERY_MOD` and `Windows = 46` → the engine's window slot — and all of them sit inside
  `SET_VEHICLE_MOD`'s accepted range. Verified by decompiling both enums and following the conversion's three
  branches. A "slot number looks wrong" report has to disprove this first.
- **The colour write order in both call sites is load-bearing and correct.** `set_PrimaryColor` re-reads the
  vehicle's secondary and rewrites **both** colours (`0x4F1D4BE3A7F24601`), and `PearlescentColor`/`RimColor`
  share one native each reading the other. Primary → Secondary → Pearl → Rim is the only order that leaves all
  four right; reordering silently swaps colours. Both writers use it (`SmartTuner.cs:403-406`,
  `MenyooAppearance.cs:118-123`).
- **`SetValue`'s missing `try`/`catch` is not a half-apply path** (`MenyooAppearance.cs:142-148`): the
  `WheelType` and `WindowTint` setters it wraps are bare `Function.Call`s and cannot throw. The `SetColor`
  beside it guards a setter that also cannot throw — the asymmetry is harmless in both directions.
- **`GetRandomInt` is `Random.Next`, exclusive**, so `ApplyParts`'s `random(0, count)` cannot select past the
  last mod and `random(0, 100) >= 75` is exactly the documented 25% skip (`SmartTuner.cs:344-347`,
  `AGENTS/AGENTS-SMARTTUNING.md:30`).
- **`GTA.VehicleColor` really has no `MatteSilver` and no `MatteGold`** — reflected, count 0 — so
  `MatteByFamily`'s silver→greys and gold/cream→desert-tan mapping is the correct workaround, as its comment
  and `AGENTS/AGENTS-SMARTTUNING.md:31` claim.
- **The empty roster, the phantom key and the NaN pace are all handled** (`12-F13` re-read and it holds):
  `BuildPaceIndex` filters NaN/Infinity out before the ranking can see them, an out-of-span target logs once and
  ranks from the closest end, and an unresolvable model is dropped by `TryGetValue` rather than indexed.
- **`tests.txt` ships empty (0 lines) and is offered in the Car Roster list.** Picking it lands in the handled
  path above — a refusal with a message, not a throw. Recorded because the file looks like a defect and is not.
- **A malformed Menyoo file cannot take the mod down.** Every parse is inside its own `try`/`catch` — the
  directory scan, the match walk and the apply — so a truncated or non-XML `.xml` in `menyooStuff\Vehicle` is
  skipped silently and the grid still spawns (`MenyooAppearance.cs:31-42`, `:78-88`, `:102-108`). Recursion is
  bounded by the folder, and a file whose root is not `<Vehicle>` simply matches nothing.

## Not verifiable on this machine — stated rather than guessed

- **Whether a real Menyoo file carries engine slot 11** (A3's trigger). There is **no `menyooStuff` folder on
  this install at all** (`ls "D:\SteamLibrary\steamapps\common\Grand Theft Auto V\menyooStuff"` → absent), and
  the Menyoo 2.0 clone recorded in `AGENTS/AGENTS-TECHNOTES.md:24` (`_research\Menyoo2`) is gone from the tree. A3's
  mechanism is read; its frequency is not measurable here, and the whole Menyoo path is dormant on this install
  (`Files.Count == 0` → `Apply` returns immediately).
- **The XML node names `MenyooAppearance` expects** — `Pearl`, `LrDashboard`, `Interior`, `Neons/Left|Right|Front|Back`, numeric `Mods` children — were not checked against Menyoo's own writer, for the same reason.
  The parser degrades by construction (a missing node returns early), so a wrong name costs a silently
  unapplied block, never a throw; but "these names are Menyoo's" is not established here.
- **The player's own Menyoo collection size**, so A9's scale is the replica's 120 files, not theirs — scale it
  from the file count in their install.
- **Anything driver-visible.** No lap was driven; every row above is a code, decompile, arithmetic, replica or
  live-file result. Nothing here is driver-verified.

## Cross-references — carried elsewhere, do not re-file

- `12-F13` (empty pool and phantom key handled), `12-F3`/`202` (`driftcars.txt.disabled` can never be
  selected), `12-R3`/`12-R4` (the unguarded roster read; `AppendAllLines` gluing — A7's sibling in the same
  write), `12-F12` (`AGENTS/AGENTS-TECHNOTES.md:507` describes `BuildPowerCache` as a file rewrite; A6 is about the
  in-memory list it actually rewrites).
- `07-T2` = `209-B1` (the creator's 2-node track throws in the loader) — A4 is the *other* short-track outcome,
  past the loader and into the grid.
- `13-B2` (the reload crash) — the `Script.Wait(10)` sites A12 cites live in the same walk.
- `210` (per-tick cost; nothing here adds to the live tick — A9's cost is all in the grid build).
- `AGENTS/AGENTS-SMARTTUNING.md:61` ("this pass and Menyoo are the only appearance writers") **omits
  `RandomizeCarColour`** (`AutosportRacingSystem.cs:4055-4065`), which paints every spawned car before either
  of them. ⚠️ Doc gap, not fixed here — this task is read-only.
