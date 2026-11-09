# 412-T3 — manoeuvre cards and the timers that own them

- Date: 2026-10-30
- Build: 1375
- Reviewer scope: read-only. No source edit, no build, no run, no git.
- Severity: **blocker** = can cost a race or crash; **release-note** = a decision or a line needed before the tag; **file** = hardening, the next reader's problem.

## Scope read (today's line numbers)

The brief's ranges were stale by ~380–420 lines. What exists, and where:

| File | Read | Contains |
|---|---|---|
| `src/Racer.cs` | 396–500 | `EffectiveBrakeFactor` 407–412, card constants 430–449, `RivalsWithinDistance` 451–455, `ActiveManeuver` 478, `_cardCooldownUntil` 479, `CardCooled`/`CardMayEnd`/`EndCard` 481–489, nitrous fields 492–495 |
| `src/Racer.cs` | 608–687 | `Initialize` (per-race reset; `_nitrousLapUsed = -1` at 645) |
| `src/Racer.cs` | 1749–1822 | `Launch` (card clear 1753–1757) |
| `src/Racer.cs` | 2207–2210 | `CornerEntranceNode` |
| `src/Racer.cs` | 2240 | `TickScale` |
| `src/Racer.cs` | 2488–2527 | `UpdateThrottleReasonCaps` — `FromOverspeed` 2502, `FromChillOut` 2514, `FromYield` 2515 |
| `src/Racer.cs` | 2650–2750 | `ConsiderManeuvers` + `ConsiderCrowdCard` (the exits) |
| `src/Racer.cs` | 2769–2770 | `OutsideGateMeters` |
| `src/Racer.cs` | 2788–2792 | `UpdateNitrous` |
| `src/Racer.cs` | 2799–2986 | the four `TryPlay*Card` entries + `RemainingRaceDistanceMeters` |
| `src/Racer.cs` | 3006–3058 | `StartNitrous` / `TryFireNitrous` / `StopNitrous` / `UpdateYield` |
| `src/Racer.cs` | 3375–3405 | `RunTimedCore` (the only caller of `ProcessAI`) |
| `src/Racer.cs` | 3474–3528 | `ApplyPowerMultiplier` (burn read 3499, 3513) |
| `src/Racer.cs` | 3553–3566 | `UpdateRivalInfo` |
| `src/Racer.cs` | 4038–4044 | `HasPassedApex` |
| `src/Racer.cs` | 4213–4294 | `ProcessAI` (call order), `UpdatePressure` |
| `src/Racer.cs` | 4647–4688 | `UpdateRivals`, `Delete` |
| `src/DataStructures.cs` | 140–258 | `Rival` + `Update`/`UpdateRouteGap`/`ComputeTimeToReach` |
| `src/DataStructures.cs` | 420–427 | `ManeuverType`, `Maneuver` |
| `src/AutosportRacingSystem.cs` | 815–828 | "Restart Race" → `SetupRace` |
| `src/AutosportRacingSystem.cs` | 1926–1935 | the only read of `ActiveManeuver` outside `Racer.cs` (debug label) |
| `src/AutosportRacingSystem.cs` | 2174–2191 | the round-robin that paces `RunTimedCore` |
| `src/AutosportRacingSystem.cs` | 2200–2215 | finish detection → `BaseBehavior = FinishedRace` |
| `src/AutosportRacingSystem.cs` | 2470–2497 | `CleanRacers`, `AddPlayerToGrid` |
| `src/AutosportRacingSystem.cs` | 2509–2556 | `SetupRace`, `StartCountdown` (Aggression 2548) |
| `src/AutosportRacingSystem.cs` | 3588–3599 | `CardLabel` |

**No card state lives in `AutosportRacingSystem.cs`.** `ActiveManeuver` is read there twice, both for drawing (`:1931`, `:3590`); there is no static card timer, and `CleanRacers`/`SetupRace`/`StartCountdown` clear nothing card-related. The only card resets are `Racer.Launch` (`:1753–1757`) and object teardown (`Delete` `:4685–4688`). That is the correct shape **because** `CleanRacers` discards the `Racer` objects — except for the "Restart Race" path, which does not (finding 5).

## Five cards, entry vs exit

Notation: `P` = entry predicate, `X` = exit predicate. "Paired" means every entry predicate that can be violated by the world appears in the exit, or is deliberately owned by a timer; thresholds may differ, predicates may not.

### 1. Nitrous — `TryPlayNitrousCard` 2799–2840, burn 3006–3046, exit 2791
| # | Predicate | Line |
|---|---|---|
| P1 | `!ARS.NitrousEnabled \|\| Lap <= _nitrousLapUsed` | 2801 |
| P2 | arm delay `NitrousArmDelayMs` after `RaceStartTime` | 2802 |
| P3 | `Control.Brake == 0` | 2803 |
| P4 | not off track | 2804 |
| P5 | `\|Steer\| < 5°` | 2805 |
| P6 | `Throttle >= 0.9` | 2806 |
| P7 | gear/AWD check | 2807 |
| P8 | `NextApexNode >= 0` | 2808 |
| P9 | not already inside the corner (when `!finishSpender`) | 2819 |
| P10 | corner entrance ≥ 8 s away (when `!finishSpender`) | 2823 |
| P11 | `rivalNearbyFaster \|\| rivalBehindIncoming \|\| lonelyClear \|\| finishSpender` | 2836 |

Exit: X1 burn deadline `_nitrousActiveUntil`; X2 `!ARS.NitrousEnabled`. **11 / 2 — paired, timer-owned.** The burn is a fixed 3 s actuator (`NitrousDurationMs = 3000`), so P3–P6, P9–P10 are entry-only by design and the exit is the deadline alone. But the branch of P11 that skips P9/P10 (`finishSpender`) is unreachable → finding 2.

### 2. DiveBomb — `TryPlayDivebombCard` 2848–2890, exit 2660–2679
| # | Entry predicate | Line |
|---|---|---|
| P1 | `CardCooled(DiveBomb)` | 2850 |
| P2 | `Brain.Corner != null` | 2851 |
| P3 | `CornerAsksForBraking()` | 2852 |
| P4 | `Control.Throttle >= 0.99` | 2853 |
| P5 | `_activeRivalWallCount <= 1` | 2854 |
| P6 | `entranceNode >= 0` | 2858 |
| P7 | `timeToEntrance ∈ [2, 6]` | 2860 |
| P8 | `rivalTimeToEntrance < timeToEntrance` | 2869 |
| P9 | `timeToEntrance − rivalTimeToEntrance <= 0.75` | 2870 |
| P10 | `RouteGapAhead ∈ (0, 30]` | 2872 |
| P11 | `ForwardSpeedGap > 0` | 2873 |
| P12 | `\|DirectionDiff\| <= 20` | 2875 |
| P13 | `CardFree(target, DiveBomb)` | 2876 |

Exit: X1 target null / dead / not in `Brain.Rivals`; X2 `\|RouteGapAhead\| > 40`; X3 `\|DirectionDiff\| > 25`; X4 `HasPassedApex(_divebombApexNode)`; X5 all of those gated on `CardMayEnd`.

**13 / 5 — missing the sign of `RouteGapAhead`** (P10 requires `> 0`, X2 tests `Math.Abs`), and missing P11 (`ForwardSpeedGap > 0`) and P4 (full throttle). Thresholds 30→40 and 20→25 are the allowed kind of difference (the exit band contains the entry band). The absent predicate is the *sign*, which is the one that can flip while a card is live → finding 3.

### 3. DefendLane — `TryPlayDefendLaneCard` 2892–2962, exit 2681–2718
| # | Entry predicate | Line |
|---|---|---|
| P1 | `CardCooled(DefendLane)` | 2894 |
| P2 | `Brain.Corner != null` | 2895 |
| P3 | `CornerAsksForBraking()` | 2896 |
| P4 | `entranceNode >= 0` | 2900 |
| P5 | `myTimeToEntrance ∈ [2, 6]` | 2902 |
| P6 | `MetresToNode(apex) > OutsideGateMeters(CurrentTrackPoint)` | 2905 |
| P7 | rival alive, not playing DiveBomb/DefendLane, `RelativePosition == Behind` | 2911–2913 |
| P8 | `closesSoon(TimeToReach ∈ [3,6]) \|\| closeBehind(Distance < 20+len)` | 2926–2928 |
| P9 | `hisTimeToEntrance <= myTimeToEntrance + 1` | 2932 |
| P10 | lane room: `\|hisLane + inward*clearance\| <= reachable` | 2951 |
| P11 | `defendTurn != 0` | 2939 |

Exit: X1 target null / dead / `!targetTrails`; X2 `threatGone` = `!stillClosing && !stillClose`; X3 `outsideOver` = `MetresToNode(apex) <= OutsideGateMeters(CurrentTrackPoint)`; X4 `HasPassedApex`; X5 all gated on `CardMayEnd`.

**11 / 5 — paired.** X3 is the same expression as P6 read at the same reference point (2705 comment says so and the code agrees); X2's `stillClose` is P8's `closeBehind` verbatim (2702 vs 2927); X2's `stillClosing` is "any finite `TimeToReach`", a strict superset of P8's `[3,6]` band — exit band contains entry band, which is the correct direction. P9/P10/P11 have no exit counterpart and need none: they are one-shot admission tests, not world-state predicates. The only defect here is ownership of `stillClosing` (finding 9).

### 4. Yield — `TryPlayYieldCard` 2964–2986, exit `UpdateYield` 3048–3058
| # | Entry predicate | Line |
|---|---|---|
| P1 | `CardCooled(Yield)` | 2966 |
| P2 | `Brain.Corner != null` | 2967 |
| P3 | `CornerAsksForBraking()` | 2968 |
| P4 | `NearestRival() != null` | 2970–2971 |
| P5 | `pressureDiff > 30` | 2977 |
| P6 | `inOverlap` (Left or Right) | 2977 |
| P7 | `timeToEntrance ∈ [0.5, 2]` | 2977 |
| P8 | `closestRival.Distance <= 20` | 2978 |
| P9 | `closestRival.Speed > my speed` | 2979 |
| P10 | `CardFree(target, Yield)` | 2980 |

Exit: X1 target null or dead; X2 `dist > 10`.

**10 / 2 — missing the dwell, missing the cooldown, and missing the band.** X2's threshold (10 m) is *inside* P8's admission band (≤ 20 m), so every card entered between 10 and 20 m is born past its own exit; X1/X2 are not gated on `CardMayEnd` where all three other exits are; and `EndCard` (`:486`) writes no cooldown for `Yield` while P1 reads that cooldown. Findings 1 and 1b.

### 5. ChillOut — `ConsiderCrowdCard` 2729–2750
| # | Entry predicate | Line |
|---|---|---|
| P1 | `const bool ChillOutEnabled` — **false** | 2738 |
| P2 | `MpsToMph(vel) >= 30` | 2740 |
| P3 | `RivalsWithinDistance(40) >= 5` | 2740 |
| P4 | `NearestRival() != null` | 2742 |
| P5 | `CardFree(target, ChillOut)` | 2743 |

Exit: X1 `RivalsWithinDistance(50) < 4`, gated on `CardMayEnd` (2733).

**5 / 1 — unreachable.** P1 is a compile-time `false` and it is the first term of the `&&` chain, so P2–P5 and X1 cannot execute, `ManeuverType.ChillOut` is never assigned anywhere, and the 1 s `ConsiderCrowdCard` pass at `:4171` runs for every AI racer every second to evaluate it. Finding 6.

## Findings

| # | Site | Severity | Claim |
|---|---|---|---|
| 1 | `Racer.cs:3054–3055` (with `:2978`, `:486`, `:482`) | blocker | The Yield card is admitted out to 20 m but its only exit ends it past 10 m with no `CardMayEnd` and no cooldown, so a card played in the 10–20 m band is ended on the same tick it is played and can be replayed on the next one, chopping the 50 % throttle cap at core-tick rate. |
| 1b | `Racer.cs:486` / `:2966` | release-note | `EndCard` deliberately skips the cooldown write for `Yield`, and nothing else writes `_cardCooldownUntil[(int)Yield]`, so `CardCooled(ManeuverType.Yield)` is a gate that can never be false. |
| 2 | `Racer.cs:2988–2993` + `:2814` | release-note | `RemainingRaceDistanceMeters` returns a **node count**, and the caller compares it against `speed * 3 s + 100` **metres**; in a multi-lap race `(laps + 1 − Lap) * nodeCount` can never be under ~300, so `finishSpender` is permanently false and the corner-lookahead guard it was meant to bypass never yields. |
| 3 | `Racer.cs:2669` (with `:2872`, `DataStructures.cs:234–237`) | release-note | The DiveBomb exit tests `Math.Abs(divePartner.RouteGapAhead) > 40` where the entry demanded `RouteGapAhead > 0`, so a partner that drops behind the player keeps the card alive; and `DirectionDiff` is forced to `0f` when either car is slower than 1 m/s, so a stopped partner always passes the 25° release. |
| 4 | `Racer.cs:2652` + `:2654` | release-note | Every card exit and the 8 s backstop sit behind `if (… BaseBehavior != Race …) return;`, so a card live when the racer finishes is never ended and its effect (`:2515` throttle cap, `:723` lane commit, `:407–412` brake bonus) keeps applying after the flag. |
| 5 | `Racer.cs:645` (`Initialize`) + `AutosportRacingSystem.cs:826` | release-note | "Restart Race" re-runs `Initialize` on the *same* `Racer` objects; it resets `_nitrousLapUsed` but not `_nitrousActiveUntil`, so a burn that was live at the restart stays live (power multiplier at `:3499`, overspeed cap at `:2502`) and the freshly-cleared lap latch lets it be re-fired. |
| 6 | `Racer.cs:2738` | file | `const bool ChillOutEnabled = false;` makes the whole `ChillOut` card unreachable: its exit `:2733`, its throttle cap `:2514`, its standoff `:2357–2368` and its label `AutosportRacingSystem.cs:3596` are dead, and `ManeuverType.ChillOut` is never assigned. |
| 7 | `Racer.cs:4289–4291` + `:4221` | file | `UpdatePressure` integrates `PressureRisePerSecond * TickScale` but is called behind a 500 ms wall-clock gate, so `Pressure` advances one core tick per half second; the only consumer in scope, `pressureDiff > 30` at `:2977`, is therefore paced by the `Aggression` spread rather than by proximity. |
| 8 | `Racer.cs:2868–2870` + `:4157–4160` | file | DiveBomb's 0.75 s reach window divides a live `ForwardNodeDistance` by `r.Speed` from the 250 ms `Rival` snapshot (`RivalRefreshMs = 250`, `:93`); on a 60 m approach a 2 m/s error in that stale speed moves the estimate 0.3 s — 40 % of the whole window. |
| 9 | `Racer.cs:2701` (`stillClosing`) + `:2915–2921` | file | DefendLane's exit predicate is read from the *target's* mirrored `Rival` record (`r.RivalRacer.Brain.Rivals`), i.e. a value written by another racer's brain on that racer's own round-robin cadence; the card cannot end until the rival's core pass has run. Entry and exit read the same foreign record, so the pair is consistent — the defect is the ownership, not the pairing. |

---

### 1 — Yield: admitted at 20 m, ended at 10 m, with no dwell and no cooldown

`src/Racer.cs:3052–3055`
```csharp
            if (ActiveManeuver.Target != null && ActiveManeuver.Target.Car.Exists())
            {
                float dist = Car.Position.DistanceTo(ActiveManeuver.Target.Car.Position);
                if (dist > 10f) EndCard();
```
`src/Racer.cs:2978`
```csharp
                || closestRival.Distance > 20f
```

`UpdateYield` is called at `:4253`, eighteen lines *after* `ConsiderManeuvers` at `:4235`, so a card played in this tick is ended in this tick. `CardMayEnd` (`:482`, 1000 ms) is checked by all three sibling exits (`:2671`, `:2713`, `:2733`) and not here. `EndCard` (`:486`) then writes no cooldown for `Yield`, so the next tick's `TryPlayYieldCard` can re-take the slot: `EndCard` → `Type = None` (`:487`) → `:2726` → `:2966` `CardCooled(Yield)` is always true → the other nine gates are slow-moving and still hold → replay.

The cap is applied in the window between play and end, at `:2515`, so what the car sees is 0.5 throttle for one core tick, 1.0 for the next, repeating while the rival is in the 10–20 m overlap band.

**Falsify**: in a race with `ShowRivalry` on, log `Game.GameTime`, `dist`, and `ActiveManeuver.Type` in `UpdateYield`; a card should never show `LastEnabled == Game.GameTime`. Static alternative: read `:2652–2726` and confirm no path from `TryPlayYieldCard` sets `_cardCooldownUntil[3]`.

### 1b — the Yield cooldown is a gate that can never fail

`src/Racer.cs:483–488`
```csharp
        void EndCard()
        {

            if (ActiveManeuver.Type != ManeuverType.Yield) _cardCooldownUntil[(int)ActiveManeuver.Type] = Game.GameTime + CardCooldownMs;
            ActiveManeuver.Type = ManeuverType.None;
            ActiveManeuver.Target = null;
```
`src/Racer.cs:2966`
```csharp
            if (!CardCooled(ManeuverType.Yield)) return false;
```

`_cardCooldownUntil` is written in exactly one statement (`:486`) and that statement excludes `Yield`; `TryPlayYieldCard` is the only writer of the Yield slot and never writes it. So `_cardCooldownUntil[(int)ManeuverType.Yield]` stays `0` for the life of the racer and `CardCooled(Yield)` is `Game.GameTime >= 0`. There is no comment giving a reason for the exclusion. House rule 8 (a value written and never read — here an array element never written but read as a gate) plus rule 2 (one bound per value).

**Falsify**: `grep -n "_cardCooldownUntil" src/Racer.cs` — six hits, one write, and it is the `!= Yield` branch.

### 2 — `RemainingRaceDistanceMeters` is a node count compared against metres

`src/Racer.cs:2988–2993`
```csharp
        float RemainingRaceDistanceMeters()
        {
            int nodeCount = ARS.TrackPoints.Count;
            if (ARS.IsPointToPoint) return nodeCount - CurrentTrackPoint.Node;
            float totalLaps = ARS.RaceMenuStore.GetInt("Laps", 6);
            return Math.Max(0f, (totalLaps + 1f - Lap) * nodeCount - CurrentTrackPoint.Node);
```
`src/Racer.cs:2813–2814`
```csharp
            bool finishSpender = closestRival != null && closestRival.Distance <= NitrousNearbyRivalDistance
                && RemainingRaceDistanceMeters() <= speed * (NitrousDurationMs / 1000f) + NitrousFinishExtraDistance;
```

Nodes are not metres and not on a fixed grid: `TrackLoader.RecomputeCumulativeDistances` (`:242–252`) builds `CumulativeDistance` by summing real node-to-node distances, which is what the neighbouring `MetresToNode` (`:2760`) uses. The function name says metres, its sibling in the same class measures metres, and it returns a count. At `speed = 60 m/s` the budget is 280; a 2-lap race on a 1000-node route starts at `2 × 1000 = 2000` and only decreases, so `finishSpender` is false for the entire race — the `if (!finishSpender)` guard at `:2815` always takes the true branch, and the eight-second corner lookahead at `:2823` can never be skipped. In the point-to-point branch the value under-reads true metres by the node spacing, so where it does fire it fires early by that factor.

This is the only consumer (`:2814`), so nothing else depends on the units. Rule 5 (names must be honest) and rule 2.

**Falsify**: log `ARS.TrackPoints.Count`, `RemainingRaceDistanceMeters()` and `speed*3+100` once per lap on a 2-lap race; the first number is ~2–3 orders of magnitude above the third.

### 3 — DiveBomb exit lost the sign of `RouteGapAhead`

`src/Racer.cs:2669`
```csharp
                bool partnerLost = ActiveManeuver.Target == null || !ActiveManeuver.Target.Car.Exists() || divePartner == null || Math.Abs(divePartner.RouteGapAhead) > DivebombReleaseGapMeters || Math.Abs(divePartner.DirectionDiff) > DivebombReleaseAngleDegrees;
```
`src/Racer.cs:2872`
```csharp
                if (r.RouteGapAhead <= 0f || r.RouteGapAhead > SameSectionMaxGapMeters) continue;
```

Entry: a partner from 1 to 30 m **ahead** along the route. Exit: `|gap| <= 40`, which admits −40 to 0 as well — a partner who has dropped behind stays "found", so `partnerLost` is false and the card survives to the apex (or the 8 s cap) carrying its 2–8 % brake bonus into `EffectiveBrakeFactor` (`:407–412`, consumed at `:4085`). The log message at `:2674` says "partner lost" and can never fire for this case.

Second, quieter term of the same shape: `DirectionDiff` is forced to `0f` when either car is under `SlowRivalMps = 1f` (`DataStructures.cs:234–237`), so `|DirectionDiff| > 25` is false for a partner who has stopped — the angle half of the exit cannot fire on a stalled target. The 40 m and 25° thresholds themselves are allowed to differ from the entry's 30 m and 20° (the exit band contains the entry band); the missing predicate is the sign.

**Falsify**: in `Divebomb`, log `divePartner.RouteGapAhead` each exit evaluation; a negative value with the card still live confirms it. Or set `DivebombReleaseGapMeters` to a value and observe that a partner who brakes past the player does not release the card.

### 4 — the exits and the 8 s backstop are behind `BaseBehavior == Race`

`src/Racer.cs:2652`
```csharp
            if (ControlledByPlayer || BaseBehavior != RacerBaseBehavior.Race || ARS.Racers.Count < 1) return;
```
`src/Racer.cs:2654–2657`
```csharp
            if (ActiveManeuver.Type != ManeuverType.None && Game.GameTime - ActiveManeuver.LastEnabled > 8000)
            {
                EndCard();
                _divebombApexNode = -1;
```

`AutosportRacingSystem.cs:2211` sets `BaseBehavior = FinishedRace` the tick a racer takes the flag; `ProcessTick` → `RunTimedCore` → `ProcessAI` keep running (`:3402`), so `ComputeSteering` (`:723` lane commit), `UpdateThrottleReasonCaps` (`:2515` cap) and the brake path (`:4085` bonus) keep reading the frozen card, while the method that owns every exit returns at its first line. The card is only freed by `Delete` at teardown, which can be up to `TimeoutSeconds` (30 s) later.

This is the "reset behind a branch that stops running" shape: the 8 s backstop is *inside* the method the guard protects. `UpdateYield` (`:3048`) is called unconditionally at `:4253` and therefore does not share the bug — which is the same asymmetry as finding 1.

**Falsify**: `ShowRivalry` on, watch a car take the flag with a card showing; the label (`:1933`) persists past the finish line.

### 5 — Restart Race inherits a live nitrous burn

`src/Racer.cs:641–645` (`Initialize`)
```csharp
            LapTimes.Clear();
            LapStartTime = 0;
            VehicleData.ResetLapPeaks();
            Lap = 0;
            _nitrousLapUsed = -1;
```
`src/AutosportRacingSystem.cs:824–826`
```csharp
                if (!PlayerParticipating) { StartRaceFromMenu(); return; }
                _raceTimedFinishMs = 0;
                SetupRace(true, true);
```

`SetupRace` (`:2515–2518`) calls `r.Initialize()` on the existing objects — `CleanRacers`, the only thing that would run `Delete` → `StopNitrous` (`:4688`), is not called on this path. `Initialize` resets `_nitrousLapUsed` and `Lap`, and does not touch `_nitrousActiveUntil`. So after a restart inside a burn window: `Game.GameTime < _nitrousActiveUntil` still holds → `ApplyPowerMultiplier` (`:3499`) keeps the nitrous multiplier, `:2502` keeps `MaxThrottleFromOverspeed` pinned at 1, the native override from `StartNitrous` was never released — and because `_nitrousLapUsed` is now `-1` while `Lap` becomes 1, `TryPlayNitrousCard`'s `Lap <= _nitrousLapUsed` gate (`:2801`) passes and the bottle can be re-fired for a second full 3 s.

**Falsify**: start a race, let an AI fire nitrous, hit Restart Race within the 3 s window, read the "Burn …" log line (`:3517`) and `AI nitrous: … lap 1` (`:3017`) in the same second.

### 6 — `ChillOut` is unreachable and its machinery still runs

`src/Racer.cs:2738`
```csharp
            const bool ChillOutEnabled = false;
```
`src/Racer.cs:2740`
```csharp
            if (ChillOutEnabled && ActiveManeuver.Type == ManeuverType.None && ARS.MpsToMph(Car.Velocity.Length()) >= ChillMinSpeedMph && RivalsWithinDistance(ChillRivalCrowdDistance) >= ChillCrowdCount)
```

Const `false`, first term: the entry cannot be reached, therefore `ManeuverType.ChillOut` is never assigned (its only writer is `:2745`), therefore the exit at `:2733` and the cap at `:2514` are dead conditions, and `ChillStandoffDistance` is read only by a label (`AutosportRacingSystem.cs:3597`) for a card that cannot exist. `ConsiderCrowdCard` is still called for every AI racer once per second at `:4171`, and it is the only card whose exit lives in a *different* method from its entry and on a 1 s beat — which is why its dwell (`CardMayEnd`, 1000 ms) could never bind either.

Whether the card is meant to ship or to stay off is the driver's call, and no comment says which; the defect is that the disabled state costs a per-second pass and leaves five fields and four branches reading as live.

**Falsify**: `grep -n "ManeuverType.ChillOut" src/*.cs` — one write at `:2745`, behind the const.

### 7 — `Pressure` advances one core tick per 500 ms

`src/Racer.cs:4288–4291`
```csharp
            if (targetPressure > Pressure)
                Pressure = Math.Min(Pressure + PressureRisePerSecond * TickScale, targetPressure);
            else
                Pressure = Math.Max(Pressure - PressureFallPerSecond * TickScale, targetPressure);
```
`src/Racer.cs:4218–4222`
```csharp
            if (_pressureTick + _phaseOffsetMs < now)
            {
                _pressureTick = now + 500;
                UpdatePressure();
            }
```

`TickScale` (`:2240`) is the interval since this racer's last `RunTimedCore` — one core pass, not the 500 ms gate. So each call integrates one tick's worth of a per-second rate, 500 ms late. With a 20-car field the core pass runs roughly every 3 ticks (~50 ms) so `Pressure` climbs ~0.11 per call, ~0.22/s against the documented 2/s; with a small field it is worse. `docs/audit/04-lane-avoidance.md:71–72` records both constants as "1/s — correct", which is the doc contradicting the code (rule 6).

In scope because the only card gate that reads `Pressure` is Yield's `pressureDiff = closestRival.RivalRacer.Pressure - Pressure; if (pressureDiff <= 30f …) return false;` (`:2973`, `:2977`): a 30-point gap between two cars in the same traffic is reachable only through the `Aggression` spread assigned at `StartCountdown` (`AutosportRacingSystem.cs:2548`, in ten-point bands), not through the pressure ramp the constant was written for.

**Falsify**: log `Pressure` once a second in `UpdatePressure` and compare the slope to `PressureRisePerSecond`; or read `:4289` against the 500 ms gate at `:4220`.

### 8 — a 0.75 s window measured with a 250 ms-old speed

`src/Racer.cs:2868–2870`
```csharp
                float rivalTimeToEntrance = r.RivalRacer.ForwardNodeDistance(entranceNode) / Math.Max(r.Speed, 1f);
                if (rivalTimeToEntrance >= timeToEntrance) continue;
                if (timeToEntrance - rivalTimeToEntrance > DivebombReachSeconds) continue;
```
`src/Racer.cs:4157–4160`
```csharp
            if (_rivalTick + _phaseOffsetMs < now)
            {
                _rivalTick = now + RivalRefreshMs;
                if (!ControlledByPlayer && BaseBehavior == RacerBaseBehavior.Race && ARS.Racers.Count >= 1) UpdateRivals();
```

`ForwardNodeDistance` is recomputed from live track positions inside this call, while `r.Speed` comes from `Rival.Update` (`DataStructures.cs:170`) which last ran up to `RivalRefreshMs = 250` (`:93`) ago — and up to 500 ms on the phase offset. `ConsiderManeuvers` runs per core tick (`:4235`). `d(estimate)/d(speed) = −distance/speed²`; at 60 m and 20 m/s a 2 m/s error in the stale speed shifts the estimate by 0.3 s against a 0.75 s acceptance window (`DivebombReachSeconds`, `:2845`).

This is the filed shape — a per-tick gate reading a value refreshed on a slower beat — and it is present in every card gate that touches `Rival`; it is reported at its narrowest, where the window is small enough for the staleness to dominate. Falsify: log `r.Speed`, `Speed - Car.Velocity.Length()`, and `timeToEntrance - rivalTimeToEntrance` at the moment the card plays; or raise `RivalRefreshMs` to 1000 and count how many more divebombs play.

### 9 — DefendLane's exit is written by the target's brain

`src/Racer.cs:2691–2694`
```csharp
                    foreach (Rival back in r.RivalRacer.Brain.Rivals)
                    {
                        if (back.RivalRacer != this) continue;
                        hisTimeToReachMe = back.TimeToReach;
```
`src/Racer.cs:2701`
```csharp
                    bool stillClosing = !float.IsPositiveInfinity(hisTimeToReachMe);
```

`TimeToReach` here is my closing time as computed from *his* snapshot of me, refreshed when *his* `RunTimedCore` last ran, and the whole predicate is `!IsPositiveInfinity` — a two-valued read of a float that his `ComputeTimeToReach` (`DataStructures.cs:245–256`) returns as infinity for four different reasons (gap ≤ 0, lateral alignment, non-positive closing, or a slot that was cleared to `null` and never re-`Update`d). If his brain stops running (finished, DNF, `ARS.NoCollision` at `:4649–4653` which nulls every slot), `TimeToReach` keeps its last value or goes stale-infinite, and my card ends on a value he owns.

Entry and exit read the same foreign record (`:2915–2921` and `:2691–2696` are the same loop), so the pair is consistent — that is why this is `file` and not a pairing defect. But it is a value with one writer in another object and a reader here, on a different cadence, and it is the only card exit whose truth is not decided by the card's own owner.

**Falsify**: freeze one rival (`Car.FreezePosition`) mid-defence and watch whether the defending car ever leaves the card before the apex gate at `:2711` releases it.

## What I could not decide

1. **Yield's 10 m exit against its 20 m entry** — inverted hysteresis (a typo) or intended "stay only while he is truly alongside"? No comment says, and the code is the only evidence. The same question covers the missing `CardMayEnd` there: an omission or a deliberate immediate-release card? Filed as a decision, not a fix.
2. **Whether the 8 000 ms cap in `:2654` is a backstop or the real maximum card age.** `CardMinDwellMs` and `CardCooldownMs` are both 1 000 and named; 8 000 is a bare literal.
3. **Whether `Math.Abs` in the DiveBomb exit was meant as "either side"** — the entry only ever admits a partner ahead, and the log string says "partner lost", which argues it was not.
4. **Node spacing on shipped tracks.** `RemainingRaceDistanceMeters`'s unit error is spacing-independent, but the size of the point-to-point error (`speed*3+100` against nodes) depends on it; I did not load a track file to measure it.
5. **Whether `ChillOut` is meant to ship.** The const is the only statement of intent; I have not treated it as a defect of behaviour, only of dead weight.
6. **`_cardCooldownUntil` sized 8 for a 5-value enum** (`:479`). Harmless today; I could not tell whether it is future-proofing or a leftover.
7. **Not investigated, out of the card scope**: why `Pressure` is a 0–100 field with `Aggression`-derived ceiling rather than a direct proximity term, and whether the yield card is meant to fire at all in this build. Both are design, not defects.
