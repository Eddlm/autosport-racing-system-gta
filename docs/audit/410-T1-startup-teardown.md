# 410-T1 — Startup, initialisation ordering, preflight gating, teardown

- Date: 2026-10-30
- Build: 1374
- Reviewer scope: read-only, no build, no run.
- Source read (current working tree):
  - `src/AutosportRacingSystem.cs` — ctor 301–414, `VerifyScriptBridge` 403–414, `InstanceTrack` 1419–1441, `InstanceGrid` 1443–1492, `PreflightStartRace` 1505–1554, `StartRaceFromMenu` 1556–1572, menu wiring 714–837, `CleanEverything` 1788–1813, `OnKeyDown` 1818–1836, `OnTick` 1838–2247, `StartRace` 2410–2454, `CleanRacers` 2456–2471, `AddPlayerToGrid` 2473–2483, `SetupRace` 2495–2523, `OnAbort` 3236–3298, `Log` 3503–3509, `LoadGrid` 3745–3917.
  - `src/VehicleMemory.cs` — whole file (138 lines).
  - `src/Racer.cs` — `Initialize` 608–687, `Delete` 4685–4702, `ProcessTick` 3136–3150.
  - Read on demand for the consumer side of the above: `src/TrackLoader.cs` 22–160, 495–522, 1056–1112; `src/FreeCamController.cs` 139–176.

Severity: **blocker** = can cost a race or crash; **release-note** = a decision or line needed before the tag; **file** = hardening, the next reader's problem.

## Findings

| # | Site | Severity | Claim |
|---|------|----------|-------|
| 1 | AutosportRacingSystem.cs:1435–1439 | blocker | `_trackInstanced = true` runs unconditionally after a void `LoadTrack` that can refuse the track, so a refused load is recorded as a successful instance. |
| 2 | AutosportRacingSystem.cs:3245 / 3256–3261 | blocker | `Racers.Clear()` is inside the abort `try` behind `_freeCam.Toggle()` and the per-racer delete loop; a throw there leaves the **static** `Racers` list populated with racers owned by the dead script instance. |
| 3 | AutosportRacingSystem.cs:1788–1812, 2456–2470 | release-note | The end-of-race teardown clears `RaceStatus` / `PlayerParticipating` after its delete calls and has no `finally`, so a throw mid-teardown leaves the race `InProgress` and `OnTick` re-enters teardown every tick. |
| 4 | AutosportRacingSystem.cs:2224–2225 (with 1808) | release-note | `RaceStatus = RaceState.Finished` is written and then clobbered by the `CleanEverything()` on the next line; `Finished` is read nowhere in the source. |
| 5 | AutosportRacingSystem.cs:340–341 | release-note | The loader's two state resets sit outside any `try`/`finally` in `HandleLoadScriptTask`, and the completion branch has no player-facing failure text (only the `IsFaulted` branch at 325 does). |
| 6 | AutosportRacingSystem.cs:366–373 | release-note | The constructor catches a menu-build failure, logs it, and continues; `_arsMenu` stays null, so the hotkey is dead and `OnTick` throws at 2079/2101 every tick, ahead of the countdown and racer block. |
| 7 | AutosportRacingSystem.cs:2069, 2083 | release-note | The "press Context to race" entry predicate accepts `RaceState.NotInitiated`, i.e. a fully built race that is waiting for the player's throttle. |
| 8 | AutosportRacingSystem.cs:3914–3915 | file | `LoadGrid` re-writes `PlayerParticipating` from a list that at that point can only hold AI racers, so it is a second writer that always writes `false`. |
| 9 | AutosportRacingSystem.cs:1849 | file | `Vehicle v = player.LastVehicle;` is never read anywhere in `OnTick`. |
| 10 | Racer.cs:3143 / 509, 638 / VehicleMemory.cs:33 | file | `_appliedThrottleLastFrame` is written three times and read nowhere, and `GetThrottle` — its only source — returns `1f` (full throttle) when it cannot read the value. |
| 11 | AutosportRacingSystem.cs:3262 / 611 | file | `Racertext` is declared and null-guarded on abort but never assigned anywhere, so the guard is dead. |
| 12 | AutosportRacingSystem.cs:3846, 3848 | file | `r._baseName = r.Name;` is written twice in a row in `AddRacer`. |

---

### 1 — `_trackInstanced` is set even when the track was refused

`src/AutosportRacingSystem.cs:1434–1440`

```csharp
TrackLoader.LoadTrack(this, TrackLoader.LoadTrackFile(trackPath));

if (!_freeCam.IsActive) Function.Call(Hash.DO_SCREEN_FADE_IN, 500);

_trackInstanced = true;
Log(LogImportance.Info, "Track instanced");
```

`TrackLoader.LoadTrack` is `void` and has two refusal paths that `InstanceTrack` cannot see:

`src/TrackLoader.cs:31–35`
```csharp
if (xmlFile == null)
{
    UI.Notify("~r~Invalid track file.");
    return;
}
```

`src/TrackLoader.cs:146–151`
```csharp
if (ARS.RouteNodes.Count < 6)
{
    ARS.Log(LogImportance.Error, "Track refused: " + ARS.RouteNodes.Count + " route nodes, six needed");
    GTA.UI.Notify("~r~That track has no usable route: six nodes minimum.");
    return;
}
```

Failure condition:
- Unreadable/malformed file: `LoadTrack` returns **before** the `ARS.RouteNodes.Clear()` at TrackLoader.cs:44, so the previous track's route survives; `InstanceTrack` then sets `_trackInstanced = true` and logs "Track instanced". The player is told "Invalid track file" and then races the *previous* route. Because `_trackInstanced` is now true, `StartRace` (2422–2426) skips the re-load and goes straight to `InstanceGrid`, which builds a grid on the old route.
- Short route: `RouteNodes` is cleared (TrackLoader.cs:44) and left empty, `_trackInstanced` is still set true, and `InstanceGrid` → `BuildGridSlots` on an empty list (TrackLoader.cs:1056–1064, `BuildCircuitGrid` at 1100–1112 does not throw on an empty list) yields zero slots, `gridSize` 0, and the player gets only "~o~The grid came back empty." (1473) — the real reason ("six nodes minimum") is shown once and then overwritten by the generic message.

This is reachable without preflight: the *Load Track* menu item calls `InstanceTrack()` directly (line 718), as does *Start Race* after `PreflightStartRace`, which validates the **XML** node count (1529, `routePoints < 6`), not the densified `RouteNodes` count that `LoadTrack` refuses on.

Falsify: put a file with 6 route points that all sit within `DensifyAboveMeters` of each other (so `DensifyRoute` cannot grow it) into `Scripts/Tracks`, select it, click *Load Track*, then *Build Grid*. Expect a "six nodes minimum" toast followed by "grid came back empty", with `Track instanced` in the log and `_trackInstanced` true.

---

### 2 — Abort leaves the static `Racers` list owned by the dead script instance

`src/AutosportRacingSystem.cs:3243–3261`

```csharp
if (_freeCam.IsActive)
{
    _freeCam.Toggle();

}
```

```csharp
foreach (Racer racer in Racers)
{
    racer.Delete();
}

Racers.Clear();
```

`Racers` is `public static List<Racer>` (line 267), so it outlives the script instance across a reload in the same AppDomain. The clear is the last statement of the try, behind:
- `_freeCam.Toggle()`, which dereferences a prop it does not own, unguarded:

`src/FreeCamController.cs:150`
```csharp
Game.Player.Character.Heading = ARS.FreeCamRide.Heading;
```
`ARS.FreeCamRide` is only re-created by `OnTick` 1864–1870; if the drone prop was removed by the game, `Toggle()` throws inside `OnAbort`'s try and every statement after 3245 — including `Racers.Clear()` at 3261, the scaleform release at 3262–3270, and `Game.Player.Character.HasGravity = true` at 3272 — is skipped. (The five state clears at 3292–3296 do run: they are in the `finally`. Prior finding A is closed for those five only.)
- `racer.Delete()`, which touches `Game.Player.Character` and `Car.CurrentBlip` unguarded (Racer.cs:4690–4694).

Failure condition: abort (or reload) while the freecam is active and the drone prop is gone → `Racers` keeps N racers whose `Car`/`Driver` entities are gone, `RaceStatus` is reset to `None` by the finally, and the reloaded script's `OnTick` (2160–2201) calls `ProcessTick()` on those stale racers every tick, throwing inside `UpdateTickData`. The player's ped also keeps `HasGravity=false` / `IsVisible=false` / `HasCollision=false` from FreeCamController.cs:168–170, because the restore at 3272 never ran — the player is invisible and falls through the map until another reload.

Falsify: enter freecam, delete the drone prop (`ARS.FreeCamRide.Delete()` from a debug menu or by pushing it out of the streaming range), then reload the script (`.reload`/`Reload()` or Ctrl+Alt+Del → the Aborted event). Check the log for "Abort teardown failed" and the player's ped state after the reload.

---

### 3 — End-of-race teardown clears state after its delete calls, with no `finally`

`src/AutosportRacingSystem.cs:1788–1812`

```csharp
void CleanEverything()
{
    CleanRacers();
    foreach (Prop p in StartLineFlares) if (CanWeUse(p)) p.Delete();
    ...
    ApplyNoCollision(false);
    LeaderboardFinish.Clear();
    ParkedDNFs = 0;
    _routeEditorActive = false;
    _raceTimedFinishMs = 0;
    ...
    RaceStatus = RaceState.None;
```

`src/AutosportRacingSystem.cs:2459–2470`

```csharp
foreach (Racer r in Racers)
{
    r.Delete();
}
Racers.Clear();

_nextInLine = 0;
_raceTimedFinishMs = 0;
...
RaceStatus = RaceState.None;
```

Both functions put every state clear after the entity work. `CleanRacers` also clears `Racers` only after the whole loop, so one throw from `Racer.Delete()` (Racer.cs:4690 `Driver.Delete()`, 4692 `Car.CurrentBlip`, 4694 `Game.Player.Character`) aborts the loop with the list still full and `RaceStatus` untouched.

Failure condition: the completion path is `OnTick` 2218–2226 → `RaceStatus = Finished; CleanEverything();`. If `CleanRacers` throws, `RaceStatus` stays `Finished`/`InProgress`, `LeaderboardFinish` is untouched, and because `raceCompleted` is recomputed from `LeaderboardFinish.Count > 0` at 2218 on the next tick, `CleanEverything()` is entered again every tick — a per-tick exception and a race that never ends, with props and flares never deleted. This is the exact shape that was fixed in `OnAbort` (state in `finally`); `CleanEverything`/`CleanRacers` still have the old shape.

I could not demonstrate a *deterministic* throw from `Racer.Delete()` in today's source (the SHVDN2 property reads it does are mostly field reads; the `Game.Player.Character` access is the only unguarded native path). Reported as release-note: the ordering is the defect, the trigger needs a dead player ped or a deleted `Driver`.

Falsify: call `CleanEverything()` with `Racers` holding one racer whose `Driver` ped has been deleted from a debug console; the second `CleanEverything()` call from 2225 repeats, and `Racers.Count` stays non-zero in the log.

---

### 4 — `RaceState.Finished` is written and immediately overwritten; nothing reads it

`src/AutosportRacingSystem.cs:2224–2225`

```csharp
RaceStatus = RaceState.Finished;
CleanEverything();
```

`CleanEverything` ends with `RaceStatus = RaceState.None;` (1808). `grep -rn "RaceState.Finished" src/` returns exactly one hit — the write at 2224. There is no read anywhere, so the finish state never exists for even one frame and any HUD keyed on it can never render. Two writers for one value in the same statement pair (house rule 1, and dead state per rule 8).

Falsify: `grep -rn "RaceState.Finished" src/` → one line only; then break at 2224 and watch `RaceStatus` return to `None` before the next tick.

---

### 5 — Loader resets sit outside `try`; a throw in the completion branch has no player-facing message

`src/AutosportRacingSystem.cs:327–341`

```csharp
else
{

    BuildPowerCache();
    RefreshPowerControls();
    FillCachedCandidates(_intendedOpponents);
    RefreshTrackList();
```
```csharp
_isLoadingScript = false;
_loadScriptTask = null;
```

`_isLoadingScript` / `_loadScriptTask` are reset by fall-through, not by `finally`. A throw anywhere in 330–337 (e.g. `BuildPowerCache` over a half-written model cache, `FillCachedCandidates`) skips 337 (`_loaded = true`) **and** 340–341. The only "ARS failed to load" text is on the `IsFaulted` branch (325) — a throw in the completion branch is not a faulted task, so the player gets no message at all: `OnTick` 1851–1854 calls `StartLoadScript()`, which returns at 303 because `_isLoadingScript` is still true, and the tick returns before `_menuPool.Process()`. The visible result is the 20 s "Loading ARS. May take a while." help text, no menu, and one `OnTick error:` line per tick from the catch at 2243–2246. If the throw is deterministic the mod never loads and nothing tells the player why.

Falsify: add a `throw` as the first line of the `else` branch, load the script, and read the log: one error per tick, no menu, no in-game message other than the loading prompt.

---

### 6 — A menu-build failure is swallowed and produces a half-initialised, silently broken script

`src/AutosportRacingSystem.cs:366–373`

```csharp
try
{
    InitializeMenu();
}
catch (Exception menuError)
{
    Log(LogImportance.Error, "Menu build failed: " + menuError);
}
```

`_arsMenu` is assigned inside `InitializeMenu` at 679. If that throws, `_arsMenu` stays null and the constructor carries on to `LoadSettings`, the version notify, and the update check; `_loaded` later becomes true and "~g~ARS has loaded." (335) is shown. Consequences, all reachable in the same session:
- `OnKeyDown` 1820 `if (... || _arsMenu == null || ...) return;` — the menu hotkey is dead, and `Log` is a file write only (`forced` is a log-level override, not a notification), so nothing is shown in game.
- `OnTick` 2079 `if (!_arsMenu.Visible && ...)` and 2101–2103 `_arsMenu.Visible = true;` throw `NullReferenceException` **every tick**, and both sit ahead of the countdown block (2106–2157), the racer core ticks (2160–2177) and the completion check (2218–2226) in the same `try`. The exception is caught at 2243, so the tick simply ends there: a race started via the Context auto-start (2083, which is *before* 2079) builds its grid and then never counts down or ticks — every car stands still, exactly the symptom prior finding C was about, arriving through a different door.

Falsify: temporarily `throw new Exception("probe")` at the top of `InitializeMenu`, load the script, and watch the log: "Menu build failed", then "ARS has loaded", then one `OnTick error: NullReferenceException` per tick.

---

### 7 — The "press Context to race" trigger fires on a race that is already built

`src/AutosportRacingSystem.cs:2069` and `:2083`

```csharp
if (RaceStatus == RaceState.None || RaceStatus == RaceState.NotInitiated)
```
```csharp
SelectTrackInMenu(nearest.TrackPath);
StartRaceFromMenu();
```

`SetupRace` sets `RaceStatus = RaceState.NotInitiated` (2522), and in that state the race is fully built but waiting for the player's throttle (`OnTick` 2107–2113). The trigger above includes that state, so standing near any track-start marker and pressing Context during the wait re-enters `StartRaceFromMenu` → `PreflightStartRace` → `InstanceTrack` (full reload) → `InstanceGrid` → `CleanRacers` (deletes the built grid) → `StartRace`. The in-progress race is torn down and rebuilt with no prompt; because the block is keyed only on `RaceStatus`, house rule 3 applies — the predicate admits a state that means "already started".

Falsify: start a race, do not press throttle (stay in `NotInitiated`), drive/walk to a yellow start chevron within 5 m and press Context. Observe `CleanRacers` + `Track instanced` + `Grid instanced` in the log for the same track a second time.

---

### 8 — `LoadGrid` writes `PlayerParticipating` a second time, always to `false`

`src/AutosportRacingSystem.cs:3914–3916`

```csharp
RaceStatus = RaceState.None;
PlayerParticipating = Racers.Any(r => r.Driver != null && r.Driver.IsPlayer);
Function.Call(Hash._0x10D373323E5B9C0D);
```

At this point `Racers` contains only the cars `SpawnRacer` created (3854–3895) with peds from `CreateDriverPed`; the player's racer is added later, by `AddPlayerToGrid` (2473–2483) after `InstanceGrid` returns. So the `Any` can only be false. `PlayerParticipating` therefore has two writers with different laws: `CleanEverything`/`OnAbort` set it to `false` unconditionally, `AddPlayerToGrid` sets it `true`, and this line recomputes it from a list that cannot contain a player. The value is not dead — `Restart Race` branches on it at 818 — so the recompute is a silent third opinion.

Falsify: break at 3915 with a grid of 8 AI cars and check `Racers.Count(r => r.Driver.IsPlayer)` → 0, always.

---

### 9 — Dead local: `Vehicle v`

`src/AutosportRacingSystem.cs:1849`

```csharp
Vehicle v = player.LastVehicle;
```

Written and never read: it is the only occurrence of `v` between 1849 and the end of `OnTick` (2247). House rule 8.

Falsify: `awk 'NR>=1849 && NR<=2247' src/AutosportRacingSystem.cs | grep -n "\bv\b"` → one line, the declaration.

---

### 10 — The memory layer's only throttle read is discarded, and it fabricates `1f`

`src/Racer.cs:3143`

```csharp
_appliedThrottleLastFrame = VehicleMemory.GetThrottle(Car);            if (!ControlledByPlayer)
```

`_appliedThrottleLastFrame` is declared at 509, re-set at 638, and set here — and read nowhere (`grep -n "_appliedThrottleLastFrame" src/Racer.cs` returns exactly those three lines). Its only producer, `GetThrottle`, is the whole reason the throttle offset is read back:

`src/VehicleMemory.cs:31–35`
```csharp
public static float GetThrottle(Vehicle vehicle)
{
    if (!ARS.CanWeUse(vehicle) || _throttleOffset == 0) return 1f;
    return *((float*)((ulong)vehicle.MemoryAddress + _throttleOffset));
}
```

`1f` is full throttle, returned for "cannot read". Two defects in one place: dead state (rule 8) and a getter whose name promises a measurement but returns a constant guess (rule 5). It is currently harmless *only because* the consumer is dead — the day someone reads `_appliedThrottleLastFrame`, every session starts believing the car is at 100 % throttle until `SetThrottle` has run once (and `WriteInput` returns without writing on the call that learns the offset, VehicleMemory.cs:60–62, so the fabricated `1f` can outlive the first real write too).

Falsify: grep for the field (three writes, zero reads) and for `GetThrottle` (one call site, at 3143).

---

### 11 — `Racertext` is guarded on abort but never assigned

`src/AutosportRacingSystem.cs:611` and `:3262`

```csharp
public static Scaleform Racertext = null;
```
```csharp
if (Racertext != null && Racertext.IsLoaded) Racertext.Unload();
```

`grep -rn "Racertext *=" src/` returns only the declaration. The guard can never be true, so it is dead code inside the abort path — and it implies a scaleform that the abort teardown is responsible for, which it is not. Compare `InstructionalScaleform` (610), which is real.

Falsify: `grep -rn "Racertext" src/` → declaration at 611 plus this read at 3262 only.

---

### 12 — `_baseName` is assigned twice in `AddRacer`

`src/AutosportRacingSystem.cs:3846–3848`

```csharp
r._baseName = r.Name;
r.DriverName = r.Name;
r._baseName = r.Name;
```

Same value, same target, two statements. Rule 1 (one writer) — harmless today, but it is exactly the kind of duplicate that later diverges.

Falsify: read 3846–3848; the first and third statements are identical.

---

## Prior findings

**A — `OnAbort`'s five state clears behind ~15 throwable statements: CLOSED for the five clears, NOT closed for the rest of the teardown.**
The five clears now live in a `finally` (AutosportRacingSystem.cs:3280–3297) and run after a throw in the try. What remains is the residue at 3261–3272: `Racers.Clear()`, the two scaleform releases, and `Game.Player.Character.HasGravity = true` are still in the try, behind `_freeCam.Toggle()` (3245) and the racer delete loop (3256–3259). See finding 2 — the consequence is now a leaked static list and an invisible, gravity-less player rather than a half-reset race.

**B — `Racer.Delete` clears the nitrous override only while the burn is live: CLOSED.**
Racer.cs:4688 is now `if (_nitrousActiveUntil > 0) StopNitrous();`, i.e. the suggested `> 0` test, and `StopNitrous` is idempotent, so an expired burn is also released.

**C — a memory-pattern miss must gate the start: CLOSED for both start doors, open at the two grid-building doors.**
`VehicleMemory.UnsupportedBuildReason()` (VehicleMemory.cs:81–88) is called at AutosportRacingSystem.cs:1514, inside `PreflightStartRace`, and both `StartRace` (2415) and `StartRaceFromMenu` (1560) go through it; the refusal string is player-facing via `UI.Notify("~o~" + refusal)` at 1562 and 2417. The Context auto-start at 2083 also routes through `StartRaceFromMenu`. Two caveats worth a release note:
- `InstanceTrack` (menu item 718) and `InstanceGrid` (menu item 798) still run with no build check, so on an unsupported build the player can load a track and spawn a full grid of cars that will never move; only *Start Race* refuses. That is tolerable only because the refusal message then names the real cause.
- The message is produced by a `FindPattern` miss on the **main thread** during preflight, which is cached (`_patterns`, 112–121), so the cost is paid once per channel. No defect, recorded so nobody "fixes" the cache.
- `ReportMissingControlPattern` / `ReportMissingHandlingPattern` (67–79) log with `forced: true`, which is a log-level override, not a notification (`Log` at 3503–3509 writes to `LogPath` only). Those two messages are only ever seen in the log file; the player-facing path is exclusively `UnsupportedBuildReason` → `UI.Notify`.

## Undecided / not claimed

- **`SetupRace`'s `[0]` index** (2510, `Racers.OrderBy(...).ToList()[0]`) throws on an empty list, but every caller I traced (`StartRace` 2435 guard, `Restart Race` 818 gated on `PlayerParticipating`) is protected upstream. I could not construct a reachable empty list, so I am not filing it.
- **`LoadGrid`'s `RouteNodes[(Racers.Count + 1) * 10]`** (3868) assumes at least `(maxcars+1)*10 + 1` route nodes, while preflight only guarantees six *XML* points. In practice `DensifyRoute` (TrackLoader.cs:495–522) explodes any gap over `DensifyAboveMeters`, so a real track always qualifies; the exception is caught per racer at 3888–3893 and degrades into "the grid came back empty". Related to finding 1, not filed separately because I could not produce a realistic track that hits it.
- **`Log` is unguarded** (`File.AppendAllText`, 3508) and is called from the loader's background `Task.Run` (`StartLoadScript` 308–312 → `RefreshCatalogs`/`FillKnownTracks`, both of which log) while the main thread logs from `OnTick`; a concurrent append throws `IOException`, and in `OnTick`'s catch (2245) that throw escapes the handler. I did not file it because the collision is timing-dependent and the fix is a lock, which is hardening rather than a defect I can point at a line for.
- **The constructor's `File.WriteAllText(LogPath, …)` at 359 is outside any `try`** — a locked/unwritable `Scripts` folder means the constructor throws after the handlers are attached (347–349), so the script loads no further and the failure is invisible (the log is the thing that failed). Same family as finding 6, not filed separately.
- **`Racers`, `RaceStatus`, `PlayerParticipating`, `LeaderboardFinish`, `TrackPoints`, `RouteNodes`, `Corners` are all `static`** and survive a script reload. `OnAbort` is the only thing that resets the first four; `TrackPoints`/`RouteNodes` are only reset by a track load, so a reload leaves a stale route with `RouteNodes.Count > 0`, which keeps the traffic-density multipliers at zero (OnTick 2091–2099) until a track is loaded again. I did not file this: the consequence is cosmetic, and the fix (clear in the ctor) is design.
