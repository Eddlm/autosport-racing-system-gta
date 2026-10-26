# Info-level logging audit

Reached from `AGENTS/AGENTS-TECHNOTES.md` -> *Logging - levels, and why `None` is the default*. Every `LogImportance.Info` call site in `src/`, with the volume each one produced in a real session.

One session, build 1178 (ARS.log: 30,980 messages, 4.96 MB, ~7 minutes of play). Source: 109 `LogImportance.Info` call sites across `src/`.

## Headline

- **29,961 of 31,010 lines (96.6%) come from a single site**: `Racer.cs:925` (`ComputeSteering`), the steering trace, which writes one line per car per tick - every AI included - and only when the Development checkbox is on.
- Every other Info site together produced ~1,000 lines in the same session.
- So the level is not the problem. One debug switch left on is, and it multiplies by the number of racers.

## Hot paths (per tick or per racer)

| Site | Method | Message | Lines | Gate |
|---|---|---|---:|---|
| `Racer.cs:925` | `ComputeSteering` | steertrace t= | 29961 | `if (ARS.SteerTrace)` |
| `Racer.cs:1291` | `UpdateLaneMover` | Lane mover  | 639 | `if (Math.Abs(_laneRate) > 0.2f && ` |
| `Racer.cs:4094` | `ApplyPowerMultiplier` | Burn  | 52 |  |
| `Racer.cs:1235` | `UpdateLaneMover` | Lane cross  | 32 | `if (Game.GameTime > _laneCrossLogg` |
| `Racer.cs:3210` | `ConsiderManeuvers` | Card divebomb:  | 28 | `if ((partnerLost || HasPassedApex(` |
| `Racer.cs:3435` | `TryPlayDivebombCard` | Card divebomb:  | 28 |  |
| `Racer.cs:1200` | `UpdateLaneMover` | Lane guard  | 25 | `if (Game.GameTime > _laneGuardLogg` |
| `AutosportRacingSystem.cs:4206` | `TryParse` | Loading Racer:  | 8 |  |
| `Racer.cs:4500` | `UpdateCornerRequirements` | Queue skip  | 8 | `if (_skippedApexes.Add(corner.Node` |
| `Racer.cs:5299` | `UpdatePerceivedGrip` | Grip for  | 8 | `if (!_gripLogged)` |
| `SmartTuner.cs:304` | `Apply` | Smart tune  | 8 |  |
| `Racer.cs:5216` | `TryArmThrottleKick` | Kick  | 2 |  |
| `Racer.cs:5325` | `UpdatePerceivedGrip` | Nitro Gs: measured  | 2 | `if (Game.GameTime < _nitrousActive` |
| `AutosportRacingSystem.cs:2058` | `OnTick` | Creating needed freecam object. | 1 | `if (CanWeUse(FreeCamRide)) FreeCam` |
| `Racer.cs:1526` | `ComputeCornerLane` | Corner line  | 1 |  |
| `Racer.cs:1571` | `ComputeCornerLane` | Corner line held  | 1 | `if (Game.GameTime > _cornerRefusal` |
| `TrackLoader.cs:159` | `TryParse` | Loading props | 1 |  |
| `TrackLoader.cs:187` | `TryParse` | Loaded track | 1 |  |
| `Racer.cs:2484` | `MeasureProjectionOutside` | Offshoot: override= | 0 | `if (ARS.DebugToggles[Options.ShowG` |
| `Racer.cs:3090` | `UpdateBrakeReasonCaps` | ABS  | 0 | `if (locked || locked != _absWasLoc` |
| `Racer.cs:3563` | `TryFireNitrous` | Player nitrous refused:  | 0 | `if (Game.GameTime > _nitroRefusalL` |
| `Racer.cs:3567` | `TryFireNitrous` | Player nitrous fired on lap  | 0 |  |
| `Racer.cs:4062` | `UpdatePowerCompensation` | Player power cut  | 0 | `if (ControlledByPlayer && _overspe` |
| `Racer.cs:4298` | `UpdateLapRegistration` | Lap++  | 0 |  |
| `Racer.cs:4313` | `UpdateLapRegistration` | Laptime  | 0 |  |
| `Racer.cs:4819` | `ProcessTimedAI` | AutoRepair  | 0 | `if (Car.EngineHealth < EngineHealt` |
| `Racer.cs:4850` | `ProcessTimedAI` | Invincible repair  | 0 |  |
| `Racer.cs:5176` | `UpdateThrottleKick` | (mensaje dinamico) | 0 |  |
| `TrackLoader.cs:1302` | `TickQueuedFlares` | Flare:  | 0 | `if (effect == 0)` |

## Bounded and event sites

| Site | Method | Message | Lines | Gate |
|---|---|---|---:|---|
| `Racer.cs:671` | `Initialize` | TRlat for  | 24 | `if (Handling.LateralTractionCurve ` |
| `Racer.cs:677` | `Initialize` | Top speed  | 24 |  |
| `Racer.cs:685` | `Initialize` | Steerlock for  | 24 | `if (VehicleData.SteeringLock < 1 |` |
| `Racer.cs:688` | `Initialize` | Wheelbase for  | 24 |  |
| `TrackLoader.cs:22` | `LoadTrackFile` | Loading  | 13 |  |
| `TrackLoader.cs:872` | `AddCornerRegion` | Corner add: apex= | 10 |  |
| `Racer.cs:3545` | `StartNitrous` | AI nitrous:  | 9 | `if (!ControlledByPlayer)` |
| `TrackLoader.cs:927` | `AddCornerRegion` | Corner open: apex= | 9 |  |
| `Racer.cs:584` | `Racer` | Brake confidence bias  | 8 |  |
| `VehicleMemory.cs:56` | `WriteInput` | [MEMORY] Learned the  | 4 |  |
| `AutosportRacingSystem.cs:523` | `FillKnownTracks` | ------------- | 3 |  |
| `AutosportRacingSystem.cs:539` | `RefreshCatalogs` | ------------- | 3 |  |
| `AutosportRacingSystem.cs:544` | `RefreshCatalogs` | ------------- | 3 |  |
| `AutosportRacingSystem.cs:4152` | `DisplayHelpText` | Ranking took  | 3 |  |
| `Racer.cs:5056` | `StartStuckRecovery` | Kick  | 2 |  |
| `TrackLoader.cs:969` | `AddCornerRegion` | Corner skipped (radii differ by > | 2 |  |
| `AutosportRacingSystem.cs:386` | `HandleLoadScriptTask` | Initialization complete. | 1 |  |
| `AutosportRacingSystem.cs:418` | `ARS` | Script initialized -  | 1 |  |
| `AutosportRacingSystem.cs:487` | `FillKnownTracks` | Learning available tracks... | 1 |  |
| `AutosportRacingSystem.cs:522` | `FillKnownTracks` | Done. | 1 |  |
| `AutosportRacingSystem.cs:540` | `RefreshCatalogs` | Discovering the vehicle pool and tracks... | 1 |  |
| `AutosportRacingSystem.cs:543` | `RefreshCatalogs` | Found  | 1 |  |
| `AutosportRacingSystem.cs:577` | `BuildPaceIndex` | BuildPaceIndex:  | 1 |  |
| `AutosportRacingSystem.cs:1490` | `Add` | Menu built:  | 1 |  |
| `AutosportRacingSystem.cs:1570` | `InstanceTrack` | Track instanced | 1 | `if (!_freeCam.IsActive) Function.C` |
| `AutosportRacingSystem.cs:1593` | `InstanceGrid` | Grid: ranking  | 1 |  |
| `AutosportRacingSystem.cs:1633` | `InstanceGrid` | Grid instanced ( | 1 |  |
| `AutosportRacingSystem.cs:1665` | `PreflightStartRace` | Preflight: paceMode= | 1 |  |
| `AutosportRacingSystem.cs:1835` | `RefreshPowerControls` | Pace span  | 1 | `if (_powerTargetValues.Count == 0)` |
| `AutosportRacingSystem.cs:2660` | `StartRace` | Starting race | 1 |  |
| `AutosportRacingSystem.cs:2687` | `StartRace` | Adding player to grid | 1 |  |
| `AutosportRacingSystem.cs:2697` | `StartRace` | Setting up | 1 |  |
| `AutosportRacingSystem.cs:2699` | `StartRace` | Set up | 1 |  |
| `AutosportRacingSystem.cs:2708` | `StartRace` | Started race | 1 |  |
| `AutosportRacingSystem.cs:2762` | `SetupRace` | Initializing racers | 1 |  |
| `AutosportRacingSystem.cs:2792` | `SetupRace` | Placing cars | 1 | `if (placecars)` |
| `AutosportRacingSystem.cs:2801` | `SetupRace` | Smart tuning queued for  | 1 | `if (tunecars && SmartTuning)` |
| `AutosportRacingSystem.cs:3293` | `GetWheelsPtr` | [MEMORY] Learned the handling offset:  | 1 | `if (addr != null)` |
| `AutosportRacingSystem.cs:3407` | `WheelPowers` | [MEMORY] Learned wheel power offset: 0x | 1 |  |
| `AutosportRacingSystem.cs:3853` | `LoadSettings` | Checked the Settings folder. | 1 |  |
| `AutosportRacingSystem.cs:4201` | `LoadGrid` | Loading vehicle models | 1 |  |
| `AutosportRacingSystem.cs:4374` | `SpawnRacer` | Grid:  | 1 |  |
| `AutosportRacingSystem.cs:4375` | `SpawnRacer` | Force-Fill: repeated the pool's models  | 1 | `if (Racers.Count < maxcars) Log(Lo` |
| `TrackLoader.cs:28` | `LoadTrack` | Loading track | 1 |  |
| `TrackLoader.cs:192` | `GenerateRouteInfo` | Generating route info... | 1 |  |
| `TrackLoader.cs:206` | `GenerateRouteInfo` | Route generated | 1 |  |
| `TrackLoader.cs:463` | `BuildApexTable` | Apex table:  | 1 |  |
| `TrackLoader.cs:557` | `DensifyRoute` | Route densified:  | 1 |  |
| `TrackLoader.cs:612` | `AssignBumpNodes` | Bumps:  | 1 | `if (!seamOk) ARS.Log(ARS.LogImport` |
| `TrackLoader.cs:668` | `MoveEntrancesForFeatures` | Entrance moved for  | 1 |  |
| `TrackLoader.cs:801` | `AssignCrests` | Crest | 1 | `if (c.Type == CrestType.Crest) cre` |
| `TrackLoader.cs:803` | `AssignCrests` | Features:  | 1 | `if (c.Type == CrestType.Crest) cre` |
| `TrackLoader.cs:887` | `AddCornerRegion` | Corner add CAP: apex= | 1 |  |
| `TrackLoader.cs:893` | `AddCornerRegion` | Corner add CAP: apex= | 1 |  |
| `TrackLoader.cs:924` | `AddCornerRegion` | Corner blur: apex= | 1 | `if (nodesToDouble > SpanBlurNodes)` |

## Not exercised in this session (25)

Zero lines, which means either a branch that this run did not take (a failed load, a missing pattern, a rare manoeuvre) or a message whose text starts with a variable. Not evidence of dead code.

| Site | Method | Message |
|---|---|---|
| `AutosportRacingSystem.cs:125` | `IsElectricModel` | GET_IS_VEHICLE_ELECTRIC is unavailable on this game build -  |
| `AutosportRacingSystem.cs:501` | `FillKnownTracks` |  - [ |
| `AutosportRacingSystem.cs:551` | `BuildPowerCache` | (dinamico) |
| `AutosportRacingSystem.cs:1428` | `Add` | Corner ceiling  |
| `AutosportRacingSystem.cs:1598` | `InstanceGrid` | Grid: the track only yielded  |
| `AutosportRacingSystem.cs:3412` | `WheelPowers` | [MEMORY] Wheel power pattern not found, using fallback 0x1D8 |
| `AutosportRacingSystem.cs:3726` | `LoadSettings` | Checking the Settings folder ... |
| `AutosportRacingSystem.cs:3754` | `LoadSettings` | Loading per-menu settings (Menu-*.ini) ... |
| `AutosportRacingSystem.cs:3833` | `LoadSettings` | Loaded per-menu settings. |
| `AutosportRacingSystem.cs:4142` | `DisplayHelpText` | (dinamico) |
| `AutosportRacingSystem.cs:4149` | `DisplayHelpText` | Pace target  |
| `AutosportRacingSystem.cs:4151` | `DisplayHelpText` | (dinamico) |
| `SettingsRepair.cs:201` | `CreateMissingFiles` | Settings repair: renamed DevSettings.ini to DevConfig.ini. |
| `SettingsRepair.cs:240` | `PruneOwnedFiles` | Settings repair:  |
| `SettingsRepair.cs:241` | `PruneOwnedFiles` | Settings repair:  |
| `SettingsRepair.cs:274` | `CompleteOwnedKeys` | Settings repair:  |
| `SettingsRepair.cs:353` | `DeleteLegacyFiles` | Settings repair: removed retired  |
| `TrackLoader.cs:346` | `LogRebuildAndVerify` | Route rebuild:  |
| `TrackLoader.cs:609` | `AssignBumpNodes` | Bump: lip= |
| `TrackLoader.cs:611` | `AssignBumpNodes` | Bumps: the route seam steps at grade  |
| `TrackLoader.cs:855` | `AddCornerRegion` | Corner skipped: apex= |
| `TrackLoader.cs:983` | `AddCornerRegion` | Corners merged ( |
| `UpdateChecker.cs:47` | `CheckLatestRelease` | UpdateChecker: no release title or tag in the GitHub respons |
| `UpdateChecker.cs:52` | `CheckLatestRelease` | UpdateChecker failed:  |
| `VehicleMemory.cs:108` | `FindPattern` | [MEMORY] Pattern not found on this build:  |

## Recommendations

1. **Steering trace**: leave it off by default (it already is) and, when it is on, limit it to the player's car (`ControlledByPlayer`, `Racer.cs:28`) and throttle to ~10 Hz. That is 394 lines/s -> ~10, and the trace was built for the player's steering anyway. Nothing else needs touching to remove the lag spikes.

2. **File I/O**: `Log()` calls `File.AppendAllText`, which opens and closes the file per line - 394 opens per second at the peak above, inside `OnTick`. Harmless once the trace is bounded; it becomes worth a buffered writer only if a hot site is ever added again.

3. **Nothing else to cut on volume grounds.** After the trace the whole level costs about 1,000 lines per session, and those lines are the ones that name a track, a roster, a pace target and a grid - the diagnostic value per byte is high.
