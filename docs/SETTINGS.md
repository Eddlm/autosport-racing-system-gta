# ARS settings — every menu key, its default, and the live value on this build

**Every key, compared: v0.9.0 (build 1381) → now** — fresh-install defaults only. `yes` = present in both, `**NEW**` = new since 0.9.0, `**gone**` = removed. Defaults that changed are bolded in the NEW column.

`CornerRadiusBlendPct` existed briefly after 0.9.0 and was deleted before any release — it never shipped, so it has no row below.

| Setting | Ini | Already existed? | NEW default | OLD default (0.9.0) |
|---|---|---|---|---|
| `BrakeLearningNotify` | Menu-Development.ini | yes | False | False |
| `LaneErrorTelemetry` | Menu-Development.ini | **NEW** | False | — |
| `ShapeAngleScale` | Menu-Development.ini | **NEW** | 0.75 | — |
| `SlipFromMemory` | Menu-Development.ini | yes | **False** | True |
| `SteerTrace` | Menu-Development.ini | yes | False | False |
| `GridSize` | Menu-Race.ini | yes | 8 | 8 |
| `Laps` | Menu-Race.ini | yes | 6 | 6 |
| `NearbyCars` | Menu-Race.ini | **NEW** | Ignore | — |
| `PaceDeviation` | Menu-Race.ini | yes | Off | Off |
| `PaceMode` | Menu-Race.ini | yes | RelativeToMine | RelativeToMine |
| `PaceOffset` | Menu-Race.ini | yes | 0 | 0 |
| `PaceTarget` | Menu-Race.ini | yes | — | — |
| `ReverseRoute` | Menu-Race.ini | yes | False | False |
| `Track` | Menu-Race.ini | yes | — | — |
| `AIRacerAutofix` | Menu-Settings.ini | yes | AutoRepair | AutoRepair |
| `AbsEnabled` | Menu-Settings.ini | yes | False | False |
| `AiNitroPct` | Menu-Settings.ini | yes | 100 | 100 |
| `BrakeLearning` | Menu-Settings.ini | yes | True | True |
| `CornerOffset` | Menu-Settings.ini | yes | **0** | 4 |
| `DNFUnderTrack` | Menu-Settings.ini | yes | True | True |
| `DamperCountersteerMaxDeg` | Menu-Settings.ini | yes | 2.00 | 2.00 |
| `ForceFillGrid` | Menu-Settings.ini | yes | True | True |
| `GridSorting` | Menu-Settings.ini | yes | Random | Random |
| `GsLaneThrottleFloor` | Menu-Settings.ini | yes | True | True |
| `HillGripEffect` | Menu-Settings.ini | yes | 100 | 100 |
| `Hotkeys` | Menu-Settings.ini | yes | True | True |
| `LogLevel` | Menu-Settings.ini | yes | Error | Error |
| `MaxCrestLift` | Menu-Settings.ini | yes | 0.4 | 0.4 |
| `MenuKey` | Menu-Settings.ini | yes | Keys.E (69) | Keys.E (69) |
| `MenuModifierKey` | Menu-Settings.ini | yes | Keys.LShiftKey (160) | Keys.LShiftKey (160) |
| `NitrousEnabled` | Menu-Settings.ini | yes | True | True |
| `NoCollision` | Menu-Settings.ini | yes | False | False |
| `OpenUpOnThrottleFloorPct` | Menu-Settings.ini | **gone** | — removed — | 60 |
| `OverspeedCut` | Menu-Settings.ini | yes | 0.2 | 0.2 |
| `PeakSlipToleranceDeg` | Menu-Settings.ini | **NEW** | 0.0 | — |
| `RouteOffset` | Menu-Settings.ini | yes | **0** | 2 |
| `RouteRadiusFar` | Menu-Settings.ini | **NEW** | False | — |
| `RubberbandMode` | Menu-Settings.ini | yes | Natural | Natural |
| `Rubberbanding` | Menu-Settings.ini | yes | 100 | 100 |
| `SlidRadiusLeadSeconds` | Menu-Settings.ini | **NEW** | 1.0 | — |
| `SlideBrakeCap` | Menu-Settings.ini | **NEW** | True | — |
| `SlipCutDepthPct` | Menu-Settings.ini | **NEW** | 60 | — |
| `SlipCutFloor` | Menu-Settings.ini | **gone** | — removed — | 0.40 |
| `SmartTuning` | Menu-Settings.ini | yes | True | True |
| `StagedSpawns` | Menu-Settings.ini | yes | False | False |
| `SteerCeilingFactor` | Menu-Settings.ini | yes | 0.90 | 0.90 |
| `SteerDampingGain` | Menu-Settings.ini | yes | 0.50 | 0.50 |
| `TcsEnabled` | Menu-Settings.ini | yes | True | True |
| `TcsSlipLimitPct` | Menu-Settings.ini | yes | 100 | 100 |
| `TcsTaperSpanPct` | Menu-Settings.ini | yes | **40** | 50 |
| `TimeoutSeconds` | Menu-Settings.ini | yes | 30 | 30 |
| `TipRate` | Menu-Settings.ini | yes | High | High |
| `UseMenyooSkins` | Menu-Settings.ini | yes | True | True |
| `VehiclePool` | Menu-Settings.ini | yes | cars.txt | cars.txt |

Debug overlays (not the ini schema — the `DebugToggles` dictionary, live values in `Menu-Debug.ini`):

| Setting | Already existed? | NEW default | OLD default |
|---|---|---|---|
| `ShowSlideAngle` | **NEW** | False | — |
| `ShowThrottleReason` | **NEW** | False | — |
| `ShowTrLateral` | **NEW** | False | — |
| `ShowCheckpoints` | yes | True | True |
| `ShowLeaderboard` | yes | True | True |
| `ShowInputs` | yes | False | False |
| `ShowTrackAnalysis` | yes | False | False |
| `ShowRivalry` | yes | False | False |
| `ReverseRoute` | yes | False | False |
| `HighDownforceOnline` | yes | False | False |
| `ShowEdgeChevrons` | yes | False | False |
| `ShowDriverNames` | yes | False | False |
| `ShowAiLapTimes` | yes | False | False |
| `LockLaneCentre` | yes | False | False |
| `ShowBumps` | yes | False | False |
| `ShowWheelSlip` | yes | False | False |
| `ShowFrontSlip` | yes | False | False |
| `ShowGsProjection` | yes | False | False |
| `ShowGLine` | yes | False | False |
| `ShowRouteRadius` | yes | False | False |

Generated from `src/SettingsRepair.cs` (the authoritative schema — every key, its fresh-install
default, its domain and the ini file it lives in) merged with the live `Settings\Menu-*.ini` on this
machine. A dash (—) means the schema declares no default (unset until the menu sets it).

Change a row, and the value lands in the named ini. The `check-settings-keys` build gate refuses a
build where code reads or writes a key the schema does not declare.

## `Menu-Race.ini`

| Key | Default | Live here | Allowed |
|---|---|---|---|
| `Track` | null | scripts\AutosportRacingSystem\Tracks\Figureight.xml |  |
| `Laps` | 6 | 4 | 2, 4, 6, 8, 10 |
| `GridSize` | 8 | 6 | 0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 14, 16, 18, 22, 26, 30, 40, 50, 60, 70, 80, 90, 100 |
| `PaceMode` | RelativeToMine | Absolute | Absolute, RelativeToMine |
| `PaceOffset` | 0 | 0 | -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 |
| `PaceTarget` | null | 240 |  |
| `PaceDeviation` | Off | Off | Off, 2, 4, 6, 8 |
| `ReverseRoute` | False | False |  |
| `NearbyCars` | Ignore.ToString | Ignore | Ignore, JoinAsIs, JoinAndTune |

## `Menu-Settings.ini`

| Key | Default | Live here | Allowed |
|---|---|---|---|
| `GridSorting` | Random | Random | Power, PowerDescendent, TopSpeed, TopSpeedDescendent, Random |
| `VehiclePool` | cars.txt | cars.txt |  |
| `TimeoutSeconds` | 30 | 30 | 15, 30, 45, 60 |
| `AIRacerAutofix` | AutoRepair | Invincible | Off, AutoRepair, Invincible |
| `CornerOffset` | 0 | 0 | -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10 |
| `RouteOffset` | 0 | 4 | -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10 |
| `SmartTuning` | True | True |  |
| `NitrousEnabled` | True | True |  |
| `TipRate` | High | High | High, Medium, Low |
| `UseMenyooSkins` | True | True |  |
| `ForceFillGrid` | True | True |  |
| `OverspeedCut` | 0.2 | 0.2 | Off, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0 |
| `BrakeLearning` | True | True |  |
| `SlideBrakeCap` | True | True |  |
| `TcsEnabled` | True | True |  |
| `AbsEnabled` | False | False |  |
| `TcsSlipLimitPct` | 100 | 100 | 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240 … 1250 (61 steps) |
| `TcsTaperSpanPct` | 40 | 100 | 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240 … 1250 (61 steps) |
| `DNFUnderTrack` | True | True |  |
| `NoCollision` | False | False |  |
| `SteerDampingGain` | 0.50 | 0.50 | 0.00, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50, 1.60, 1.70, 1.80, 1.90, 2.00 |
| `DamperCountersteerMaxDeg` | 2.00 | 4.00 | 0.00, 2.00, 4.00, 6.00, 8.00, 10.00 |
| `SteerCeilingFactor` | 0.90 | 1.00 | 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60 … 2.00 (40 steps) |
| `SlipCutDepthPct` | 60 | 50 | 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 |
| `PeakSlipToleranceDeg` | 0.0 | 2.0 | 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0 |
| `MaxCrestLift` | 0.4 | 0.5 | 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0 |
| `HillGripEffect` | 100 | 100 | 0, 25, 50, 75, 100, 150, 200 |
| `GsLaneThrottleFloor` | True | True |  |
| `RouteRadiusFar` | False | True |  |
| `SlidRadiusLeadSeconds` | 1.0 | 1.5 | 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5 |
| `Rubberbanding` | 100 | 100 | 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 |
| `RubberbandMode` | Natural | Natural | Natural, Artificial |
| `StagedSpawns` | False | False |  |
| `Hotkeys` | True | True |  |
| `MenuModifierKey` | 160 | 160 | `Keys.LShiftKey` — the number is the enum value |
| `MenuKey` | 69 | 69 | `Keys.E` — the number is the enum value |
| `LogLevel` | Error | Info | Info, Error, Fatal, None |
| `AiNitroPct` | 100 | 100 | 50, 100, 150, 200 |

## `Menu-Development.ini`

| Key | Default | Live here | Allowed |
|---|---|---|---|
| `BrakeLearningNotify` | False | False |  |
| `LaneErrorTelemetry` | False | False |  |
| `SteerTrace` | False | False |  |
| `SlipFromMemory` | False | False |  |
| `ShapeAngleScale` | 0.75 | 0.75 | 0.50 … 1.00 (step 0.05) |

## `Menu-Debug.ini` — debug toggles

Not in the repair schema — written by the Debug menu itself. Defaults from `DebugToggles` in `AutosportRacingSystem.cs`.

| Key | Default | Live here |
|---|---|---|
| `ShowInputs` | False | True |
| `ShowTrackAnalysis` | False | False |
| `ShowRivalry` | False | False |
| `ReverseRoute` | False | False |
| `HighDownforceOnline` | False | False |
| `ShowCheckpoints` | True | True |
| `ShowEdgeChevrons` | False | False |
| `ShowLeaderboard` | True | True |
| `ShowDriverNames` | False | False |
| `ShowAiLapTimes` | False | False |
| `LockLaneCentre` | False | False |
| `ShowBumps` | False | False |
| `ShowWheelSlip` | False | False |
| `ShowFrontSlip` | False | False |
| `ShowSlideAngle` | False | False |
| `ShowThrottleReason` | False | False |
| `ShowGsProjection` | False | False |
| `ShowGLine` | False | False |
| `ShowRouteRadius` | False | False |
| `ShowTrLateral` | False | False |
