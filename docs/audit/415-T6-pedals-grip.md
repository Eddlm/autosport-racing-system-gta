# 415-T6 — Pedals and the grip laws they read

- Date: 2026-10-30
- Build: 1378
- Mode: read-only. No source edit, no build, no git, no game. This file is the only write.
- Read: `src/Racer.cs` (4709 lines, md5 at read time `2687d18580dcc911b936d4ea2942ae6e`, **now** `658d2d14173cb342a9d61a5ef4a33717`), `src/DataStructures.cs` (427, `ca7e163a38efafb2e5016426433b8ffc`), `src/VehicleMemory.cs` (138, `8813082a016a4c3fb77ec6183bbb31e7`), `src/AutosportRacingSystem.cs` (3944 lines / `704217b4d2a043f59a7ee84c535d8c48` at read time, **now** 3946 / `b60f1d7c986061dcf7b640ebdf64fa3e` — cited only for the native readers the pedal path trusts, `MaxWheelSlip` 3054, `BrakeLockSlip` 3042, `WheelsOffGround` 3133, `WheelSideSlipAngles` 3144).

**Baseline note.** The tree moved under this review. `Racer.cs` changed md5 with its length fixed at 4709 and **no movement at any line this report cites** — all 42 anchors (1589 … 4581) were re-read one by one against `658d2d14…` and are byte-identical, so every Racer quote and line number below is current; the delta is somewhere I do not quote. `AutosportRacingSystem.cs` grew by 2 lines above line 1006, which shifted every cited ARS line by +2; the numbers below are the **post-shift** ones, re-resolved against `b60f1d7c…`. Re-anchor against those two md5s, not against a later tree.
- Every `Racer.cs` line and quote below was read from `658d2d14…`; every `AutosportRacingSystem.cs` one from `b60f1d7c…`.

## The pedal path, in tick order

`ProcessTick` `Racer.cs:3141` (per frame) → `ApplyInputs` `:3456` writes the pedals reached here → `RunTimedCore` `:3400` (one core tick per racer roughly every 10 ms, `ARS:2186`; `TickScale` `:2245` is `0.001 * TimeSince_lastCoreTick`, i.e. **seconds**, and is honest) → `ProcessAI` `:4218`:

| # | step | line |
|---|---|---|
| 1 | `ComputeTargetSpeed` → `IntendedSpeed`, and the route brake cap | 4241 |
| 2 | `ComputeSteering` | 4242 |
| 3 | `UpdateThrottleReasonCaps` (8 throttle caps) | 4244 |
| 4 | `UpdateBrakeReasonCaps` (3 brake caps) | 4245 |
| 5 | `UpdateThrottleKick` (returns immediately, see table note) | 4246 |
| 6 | `ConvertSpeedToPedals` (plan → slew → caps → floor) | 4247 |
| 7 | `ApplyStuckRecoveryOverride` (pedal written again) | 4252 |
| 8 | `ApplySteerLimits` → writes `_slip` and the measured-slip clamp | 4254 |

The last line of that table is the root of F12 below: step 6 reads what step 8 writes.

## Cap table — every field in `VehicleControl` that bounds a pedal

`MaxThrottle*` are multiplied/`min`'d in `ComposeThrottleCap` `:2591-2624`; `MaxBrake*` in `ComposeBrakeCap` `:2626-2642`. `Launch()` `:1753` is the per-race reset; `Initialize()` `:609` sets the pedals to 0 once (`:635-636`).

| field (DataStructures) | tick writer | law / shape | reset in `Launch` | reaches the pedal? |
|---|---|---|---|---|
| `MaxThrottleFromTCS` 323 | `Racer.cs:2499` | `GlideCap` 3.5/s down, 1.75/s up (`:2470-2480`) toward `TcsCapLevel()` 0.2..1 | `:1795` | yes, `min` `:2597` |
| `MaxThrottleFromStability` 332 | `:2501-2505` | hand-integrated: `-0.5 × wheelsOffGround /s`, `+0.5/s`, clamp 0..1, **no glide** | `:1796` | yes, `:2598` |
| `MaxThrottleFromOverspeed` 328 | `:2507-2509` | hard `1f` under nitrous, else `GlideCap` | `:1797` | yes, `:2598` |
| `MaxThrottleFromRival` 329 | `:2517` | `GlideCap`, level from `RivalLiftMeters` remap | `:1798` | yes, `:2599` |
| `MaxThrottleFromChillOut` 330 | `:2519` | `GlideCap` toward `ChillThrottleCap` | `:1799` | yes, `:2600` |
| `MaxThrottleFromYield` 331 | `:2520` | `GlideCap` toward `YieldThrottleLevel` | `:1800` | yes, `:2601` |
| `MaxThrottleFromOffTrack` 333 | `:2529` | `GlideCap`; level from `Remap(mph, 20, 15, 0, 1)` `:2525` | `:1801` | yes, `:2602` |
| `MaxThrottleFromRecovery` 334 | `:2533` | `GlideCap` toward a speed remap | `:1802` | yes, `:2603` |
| `MaxThrottle` 321 | `:2604` | `min` of the eight above | `:1790` | yes — but its only reader is `:1942` (bounds the Gs floor) |
| `MaxBrakeFromABS` 324 | `:2564` | **hard assign** `AbsCapLevel()`; 0 on a locked wheel, 1 below 15 mph | `:1792` | yes `:2628`/`:2633`, and again at `:1925` |
| `MaxBrakeFromCountersteer` 327 | `:2565` | `GlideCap` toward `CountersteerPedalCap` 0.5 | `:1794` | nominally `:2628`/`:2635` — **can never bind**, F6 |
| `MaxBrakeFromRoute` 326 | `:2360` | hard assign `0.2` or `1f`, on the deep path of `ComputeTargetSpeed` only | `:1793` | yes `:2629`/`:2634`; **not** for the projection brake, F3 |
| `MaxBrake` 322 | `:2628` then `:2629` (two writes) | `min(ABS, Countersteer)` then `min(…, Route)` | `:1791` | **no reader anywhere**, F10 |

Two notes on the table:

- `UpdateThrottleKick` `:4492` returns on its first line because `const bool KickCardEnabled = false;` `:4490`. So `_kickPlaying` is permanently false, and both `_kickPlaying ? 1f :` at `:2499` (a hard TCS bypass that skips `GlideCap`) and `kickRaises` `:2594-2595` (which lifts the *plan* to `KickPedalLevel`) are unreachable today. Dead, but they are two more laws sitting on the throttle actuator, ready to switch on: if the card is re-enabled, TCS reads 1.00 for the whole window and then has to glide back down at 3.5/s.
- Pedal bounds that live **outside** this table, applied around it in `ConvertSpeedToPedals`: front-slip cut `:1895-1899`; countersteer 0.5 on both pedals `:1906-1907`; projection throttle `:1933`; projection brake `:1925`; Gs lane floor `:1944`; slew `:1883`, `:1936-1937`. And `ApplyStuckRecoveryOverride` `:4576-4581` writes the pedals directly after the whole pipeline.

## Findings

### F1 — blocker — the "Open Up on Throttle" law is multiplied into a field nobody steers by
`src/Racer.cs:1694-1697`

```
                _clampTargetDeg = clampTarget;

                _clampTargetDeg *= OpensUpClampScale();
                float chase = float.IsNaN(_chasingClampDeg) ? clampTarget : _chasingClampDeg;
```

`OpensUpClampScale()` has exactly one call site in the tree (`:1696`) and `_clampTargetDeg` has exactly one reader, the HUD at `:3256`. The limiter itself chases the **local** `clampTarget`, unscaled, at `:1697`, `:1699`, `:1701` and `:1708`, and `speedCeiling` is `min`'d with `_chasingClampDeg` at `:1710`. So the setting documented at `ARS:1008` ("Share of the normal steering clamp left at full pedal … 40 % opens the wheel to two fifths of its allowance under power") does nothing to the wheel at all, and the readout at `:3269` divides the applied (unscaled) clamp by the scaled target:

```
                UI.ShowSubtitle("Slip vs Allowed~n~x" + (applied / target).ToString("0.0")
                    + " of " + target.ToString("0.0") + "°", 50);
```

which prints a ratio above 1.0 whenever `OpenUpOnThrottleFloorPct < 100` — i.e. the debug subtitle says the limiter granted more steer than it asked for.

Falsify: set `OpenUpOnThrottleFloorPct = 40`, hold the same steer request through the same corner at full pedal with and without the setting, and log `_chasingClampDeg` against `clampTarget`. Expect identical clamp traces (proving the law is inert) and `applied/target > 1`.

### F2 — blocker — `WheelsOffGround` counts "no side slip" as "no contact"
`src/AutosportRacingSystem.cs:3139`, consumed at `src/Racer.cs:2500-2502`

```
                if (*((float*)(wheel + 0x1C4)) == 0f) off++;
```

```
            int wheelsOffGround = ARS.WheelsOffGround(Car);
            if (wheelsOffGround > 0)
                Control.MaxThrottleFromStability -= StabilityFallPerWheelPerSecond * wheelsOffGround * TickScale;
```

`0x1C4` is the field the same file reads as the **wheel side-slip angle**: `MaxWheelSideSlip` `ARS:3127`, `WheelSideSlipAngles` `ARS:3150`, and `DrawWheelSlip` `Racer.cs:3288-3298` colours each wheel's sphere by it. Both readings cannot be right. If it is the slip angle, then the predicate "airborne" is "slip angle is exactly zero", which is the *best*-grip case: on flat tarmac, driving straight, all four wheels sit at 0 and `MaxThrottleFromStability` drains at `4 × 0.5 = 2.0/s` from 1.0 to 0 in half a second, on the one part of the track where full throttle is wanted. If it is not the slip angle, then the slip-angle laws (F7, and the whole measured-slip limiter) read a field that is not slip. Either way one of the two names is lying about what it measures.

Note this is the only cap in the table with a hand-written integrator instead of `GlideCap`, and it is also the only one that can be driven to 0 and back purely by a predicate that reads a *dynamic* value — so it is the most exposed to this.

Falsify: log `ARS.WheelsOffGround(Car)` and `Control.MaxThrottleFromStability` while driving straight at 50 mph on flat road. Expect 0 and 1.0; the bug gives 4 and a decay to 0.

### F3 — release-note — the projection brake is a second writer bounded by ABS only
`src/Racer.cs:1923-1928`

```
                if (projectionInput < 0f)
                {
                    float projectionBrake = Math.Min(-projectionInput, Control.MaxBrakeFromABS);
                    if (projectionBrake > newBrake)
                    {
                        newBrake = projectionBrake;
```

(`:1923-1928`; the bound is `:1925`.)

`newBrake` was already composed against all three brake caps at `:1919`, four lines above (`newBrake = ComposeBrakeCap(newBrake, countersteering);`). This block then *overwrites* it with a value bounded only by `MaxBrakeFromABS` — `MaxBrakeFromRoute` (`RouteBrakeCap = 0.2`, `:222`) is not applied, and neither is `MaxBrakeFromCountersteer` (moot here only because of the `!countersteering` guard on the enclosing `if`). One bound per value, broken: the route's "this corner is curvature-limited, feather the brake" law is silently overridden by the off-track projection's brake, which is allowed to go to 1.0. The 1854 comment says the projection owns the *throttle floor*; nothing in the code says it owns the brake ceiling.

Falsify: on a route-limited section (`MaxBrakeFromRoute == 0.2`) put the car wide enough that `projectionInput < 0`, log `Control.Brake` and `Control.MaxBrakeFromRoute` per tick: brake > 0.2 with the cap at 0.2.

### F4 — release-note — the projection throttle cap lands after the caps, so the reason level reports a pedal that never existed
`src/Racer.cs:1916` and `:1933`, read back at `:2604`, `:2622`

```
            newThrottle = ComposeThrottleCap(newThrottle, projectionBinds, countersteering);
```

```
                else if (newThrottle > projectionInput) newThrottle = projectionInput;
```

`ComposeThrottleCap` returns `min(baseThrottle, all eight caps)` and stores the same number in `Control.MaxThrottle` `:2604` and `Control.ThrottleReasonLevel` `:2622`. The projection then lowers `newThrottle` to `projectionInput` afterwards, so under an off-track projection the log and the HUD report `Control.ThrottleReasonLevel` — advertised as the level the throttle is held at — at a value the pedal never reaches. The `offtrackLimited` parameter is passed `projectionBinds` (`:1915-1916`) and only ever relabels the reason at `:2616`; it does not participate in the `min` at all, despite being named as if it were a bound. Same class for `MaxThrottle` `:2604`: it is the composed ceiling, not the applied one, and its single reader `:1942` uses it to clamp the Gs floor.

Falsify: drive off the outer edge while wider-than-was and over target speed, log `ThrottleReason`, `ThrottleReasonLevel`, `Control.MaxThrottle`, `Control.Throttle`: level and MaxThrottle stay above the pedal for as long as the projection ramp is running.

### F5 — release-note — `MaxBrakeFromRoute` is skipped by three early returns and keeps its last value
`src/Racer.cs:2360`, with `:2267-2281`

```
            Control.MaxBrakeFromRoute = routeTerm <= cornerTerm && !cornerPlansABrake ? RouteBrakeCap : 1f;
```

```
            if (BaseBehavior == RacerBaseBehavior.GridWait)
            {
                Brain.CurrentIntention.Speed = 200f;
                return;
            }
```

`:2360` is the only writer of this cap, and it sits below three `return`s (`GridWait` `:2267`, `FinishedRace` `:2272`, `FinishedStandStill` `:2277`). The cap is therefore not recomputed on those paths and holds whatever the last racing tick left in it. A racer that crosses the line on a curvature-limited section enters `FinishedRace` with `MaxBrakeFromRoute = 0.2` and can then only brake at 20 % while it coasts down from the 20 m/s that `:2274` asks for — for the rest of the session, until `Launch()` sets it back to 1 at `:1793`. It is also not a glide, so on the racing path the cap steps 1.0 ↔ 0.2 between ticks with no ramp, while every other cap in the table is slewed.

Falsify: finish a race on a route-limited section (`MaxBrakeFromRoute` == 0.2 on the last racing tick), then log `Control.MaxBrakeFromRoute` and `Control.Brake` in `FinishedRace`: cap stuck at 0.20, brake clamped with it.

### F6 — release-note — the countersteer brake cap is written twice, and the glided one can never bind
`src/Racer.cs:1907` versus `:2565` / `:2635`

```
                newBrake = Math.Min(newBrake, CountersteerPedalCap);
```

```
            Control.MaxBrakeFromCountersteer = GlideCap(Control.MaxBrakeFromCountersteer, countersteering ? CountersteerPedalCap : 1f);
```

Two writers encode one law. The hard `min` at `:1907` already guarantees `newBrake ≤ 0.5` *before* `ComposeBrakeCap` runs; `MaxBrakeFromCountersteer` glides from 1.0 **down towards** 0.5 and never below it, so `if (Control.MaxBrakeFromCountersteer < binding)` at `:2635` cannot fire while the hard cap is in place. The glided cap is a floor under a floor that cannot bind. The intent of the `GlideCap` — ease the brake chop in rather than snap it — is defeated by the snap at `:1907`, which is exactly what happens: a car braking at 1.0 under countersteer drops to 0.5 in one tick, and a `GlideCap` at 3.5/s would have taken ~0.14 s. The throttle has the mirror asymmetry: `CountersteerPedalCap` at `:1906` with no `MaxThrottleFromCountersteer` behind it, so the same law is table-backed on one pedal and table-free on the other.

Falsify: brake at 1.0, engage countersteer, log `Control.MaxBrakeFromCountersteer` and `Control.Brake` together. The pedal steps to 0.50 in one tick while the cap is still easing down through 0.8; `MaxBrakeFromCountersteer` is never the minimum.

### F7 — release-note — one normaliser feeds the steering limiter and the throttle cut
`src/Racer.cs:1589-1590` (memory path), `:1608` (theory path), consumed at `:1688` and `:1895`

```
            float capShare = Math.Max(0.05f, ARS.SteerCeilingFactor) * SlowCornerSlipShare(fwdSpeed);
            s.InnerPct = 100f * rawInner / capShare;
```

```
            if (_slip.Readable && _slip.InnerPct > 100f)
            {
                float over = ARS.Clamp((_slip.InnerPct - 100f) / 150f, 0f, 1f);
                combinedInput = Math.Min(combinedInput, Math.Max(0f, 1f - over));
            }
```

`_slip.InnerPct` is the shared normaliser: `100 / InnerPct` is the steering cut factor `:1688`, and `InnerPct > 100` is the throttle cut threshold `:1895`. Its divisor carries `ARS.SteerCeilingFactor`, which the settings menu documents purely as a steering-limiter knob (`ARS:1094`, "How much of the tyre's peak slip the limiter allows before it cuts"). Moving that slider therefore moves the speed at which the throttle starts being cut as well: at 0.5 the cut begins at half the slip, at 2.0 at twice it. The same expression is re-typed a third time for the HUD at `:3277` (`float share = Math.Max(0.05f, ARS.SteerCeilingFactor) * SlowCornerSlipShare(...)`), so the readout is a copy of the law, not a reader of it — three places to edit, two of which are not the law.

Falsify: hold a fixed steer angle and speed in a long corner (`SlipFromMemory` on), switch Peak Slip Share between its extremes, log `_slip.InnerPct` and `Control.Throttle`: both the steering clamp and the throttle cut boundary move with the steering setting.

### F8 — release-note — the stuck-recovery override writes both pedals outside the pipeline, and reports a level that is not the pedal
`src/Racer.cs:4576-4581`

```
            Control.Brake = 0f;
            Control.BrakeReason = BrakeReason.StuckRecovery;
            Control.BrakeReasonLevel = 0f;
            Control.ThrottleReason = ThrottleReason.StuckRecovery;
            Control.ThrottleReasonLevel = 0f;
            Control.Throttle = -0.5f;
```

Called at `:4252`, after `ConvertSpeedToPedals`. It bypasses the slew (`:1883`) and every cap in the table — that is arguably what an override is for, and the state machine owns reverse — but two things are defects regardless: (a) `Control.ThrottleReasonLevel = 0f` while `Control.Throttle = -0.5f`, so the reason level is not the pedal and is not even in the pedal's sign domain; (b) the reverse value is a magic `-0.5` that no setting or constant names, in a pipeline whose every other number is a named constant, and `ApplyInputs:3466` clamps the throttle to `[-1, 1]` so it passes through untouched.

Falsify: force `_stuckPhase == StuckPhase.Reverse`, log `Control.Throttle`, `Control.ThrottleReason`, `Control.ThrottleReasonLevel`: `-0.50` with reason level `0.00`.

### F9 — release-note — the Gs lane floor raises the pedal after the slew, so the slew is not one bound
`src/Racer.cs:1942-1944`, against `:1883` and `:1936-1937`

```
            gsFloor = Math.Min(gsFloor, Control.MaxThrottle);
            float throttleBeforeFloor = Control.Throttle;
            if (gsFloor > 0f) Control.Throttle = Math.Max(Control.Throttle, gsFloor);
```

`GsLaneThrottleMaxFloor` is `1f` (`:224`), so when the inside-lane regime arms, this line can take `Control.Throttle` from whatever the plan asked for to full in a single core tick, ignoring the `inputChange` budget that `:1937` enforces for every other move. The `min` with `Control.MaxThrottle` does keep it under the eight caps, so this is a slew violation, not a cap violation — and `_gsFloorActiveThisTick` `:1945` exists only to tell the brake learner `:2139` that it happened, which reads like the floor was meant to be an exception the rest of the pipeline knows about. It also interacts with F4: `Control.MaxThrottle` is the pre-projection ceiling, so the bound this line is clamped by is not the applied one.

Falsify: log `Control.Throttle` per core tick through an apex where lateral Gs cross `0.525 × theoreticalGs` (`:1860`) with the plan throttle near 0. Expect a step larger than `Math.Min(6 × TickScale, 0.5)`.

### F10 — file — `Control.MaxBrake` is composed and never read; `_appliedThrottleLastFrame` is written and never read
`src/DataStructures.cs:322`, written `src/Racer.cs:2628-2629`, `:1791`

```
            Control.MaxBrake = Math.Min(Control.MaxBrakeFromABS, Control.MaxBrakeFromCountersteer);
            Control.MaxBrake = Math.Min(Control.MaxBrake, Control.MaxBrakeFromRoute);
```

Three references to `Control.MaxBrake` exist in the tree (`:1791`, `:2628`, `:2629`) and all three are writes. Nothing enforces it and nothing displays it: `ApplyInputs` `:3467` clamps the brake to `[0,1]`, not to `MaxBrake`; the pedal-limit HUD `:3360-3361` reads the two component fields, not the composite. So the brake's composed ceiling is decorative, and the row exists in the table only because it is written. The two successive writes are also a second `min` on the same value (one bound per value) — harmless, since the first is unreadable in between, but it is the shape the house rules call out.

Same defect, different field: `_appliedThrottleLastFrame` (`:509`, `:638`, `:3148`) has three writes and no reads, and the surviving line still carries the hole where its reader was:

```
            _appliedThrottleLastFrame = VehicleMemory.GetThrottle(Car);            if (!ControlledByPlayer)
```

Falsify: `grep -rn "Control.MaxBrake[^F]" src/` → three hits, all assignments; `grep -rn "_appliedThrottleLastFrame" src/` → three hits, all assignments.

### F11 — file — one pedal rate, two bounds, one of them per tick
`src/Racer.cs:1883`, with `:1829-1830`

```
            float inputChange = Math.Min(PedalSlewRate * TickScale, PedalSlewMaxPerTick);
```

`PedalSlewRate = 6f` is a per-second rate and `PedalSlewMaxPerTick = 0.5f` is an absolute per-tick step; `min` of the two means the bound that applies changes with the frame time. Below 83 ms per core tick the rate governs (6/s); above it the per-tick clamp governs and the effective rate becomes `0.5 / TickScale`, so at 10 Hz the pedal moves at 5/s and at 5 Hz at 2.5/s. Latent today — the core ticks at roughly 100 Hz (`ARS:2186`), so `6 × 0.01 = 0.06` and the per-tick clamp never binds — but the second bound is not frame-rate independent and would take over on a hitch (`TimeSince_lastCoreTick` is clamped to 9999 ms at `:99`, so a 1 s stall asks for `inputChange = 0.5`).

Falsify: log `inputChange` with `_lastCoreTick` forced 200 ms back: expect `0.5`, i.e. 2.5 units/s, not 6/s.

### F12 — file — `_slip` is read by the throttle before `ApplySteerLimits` writes it, and no launch clears the slip latches
`src/Racer.cs:1895` (read) against `:1681` (write), reached from `:4247` versus `:4254`; `:269`, `:1697`

```
            _slip = MeasureFrontSlip(fwdSpeed);
```

`_slip`, `_clampTargetDeg` and `_chasingClampDeg` are written only inside `ApplySteerLimits` (`:1681`, `:1694`, `:1696`, `:1701`, `:1708`), and `ProcessAI` calls `ConvertSpeedToPedals` (`:4247`) *before* `ApplySteerLimits` (`:4254`). So the F7 throttle cut at `:1895` reads the previous core tick's slip state on every tick — one tick stale by construction, ~10 ms at the current cadence, and not stale at all only in the degenerate case where the two happen to agree. None of the three fields is reset by `Launch()` (`:1753-1827`): `_chasingClampDeg` in particular survives a race, so a car that ended the last race clamped to 8° starts the next one chasing from 8° instead of from `clampTarget`, and `:1697` (`float chase = float.IsNaN(_chasingClampDeg) ? clampTarget : _chasingClampDeg;`) deliberately skips the rate limit on the very first slip-readable tick of the session — the rate limiter is born at its own limit.

Falsify: crash mid-corner with the limiter clamped (`_chasingClampDeg ≈ 8`), relaunch, log `_chasingClampDeg` on the first core tick where `_slip.Readable`: 8.0 (stale) rather than NaN/`clampTarget`. And with a fresh start, log `_clampTargetDeg` and `_chasingClampDeg` on the first readable tick: equal, i.e. the clamp snapped in with no ramp.

## Prior findings, re-checked against build 1378

**A — "the pedal law is proportional on speed error with no deceleration term, so a car 1 m/s over at 200 m out and one 1 m/s over at 10 m out get the same pedal."** — **TRUE today at `Racer.cs:1954-1955`:**

```
            float divisor = intendedSpeedChange < 0f ? FullPedalSpeedErrorMps * BrakeErrorMultiplier : FullPedalSpeedErrorMps;
            return ARS.Clamp(intendedSpeedChange / divisor, -1f, 1f);
```

`ComputeCombinedInput` takes only `IntendedSpeedChange` (`:1891`), and nothing between `:1880` and `:1937` reads a distance, a time-to-go or a deceleration. `UpdateBrakeLearning` (`:2030`) adjusts the *apex* speed it learns, not the pedal. The approach is carried entirely by the target (`ApexBrakingSpeed` `:4051` is distance-aware), which is why the law survives: the pedal stays a P law on error by design. Still a real sensitivity — a stepped target (B) becomes a stepped pedal with no damping.

**B — "the curvature term has no time dimension while the corner does, so its target can step ~13 km/h in one node."** — **TRUE in shape at `Racer.cs:2358`** (`Brain.CurrentIntention.Speed = Math.Min(cornerTerm, routeTerm);`), with the curvature term born at `:2295` from `RouteRadiusSampled()` (`:3745`, sampled per tick through four nodes whose spacing is `Car.Velocity.Length() * 2f / traction`), and no smoothing or lead anywhere on the `routeTerm` path (`:2356-2358`). The corner term has distance in it (`ApexBrakingSpeed` → `ForwardNodeDistance`, `:4061`); the curvature term is the instantaneous radius of a node-sampled circle, so it changes in steps as the sampled nodes advance, and the target inherits the step. I did **not** reproduce the ~13 km/h magnitude — that needs track data and a driven lap (see "Could not decide").

**C — "the 10-100 % wheel-opening helper must scale the clamp target only, never the shared `capShare`/`available` normaliser, because that number also feeds the throttle cap."** — **TRUE (wiring satisfies it) at `Racer.cs:1696`**: `OpensUpClampScale()` multiplies `_clampTargetDeg` and nothing else; `capShare` `:1589` and `available` `:1608` are untouched by it. C is not violated. But the property C was protecting is now moot *and* worse: the target it does scale is read only by the HUD, so the helper's output never reaches a law at all (F1). Note also that the thing C was worried about is done by a different knob: `ARS.SteerCeilingFactor` *does* scale `capShare`/`available` and those do feed the throttle cut (F7).

**D — "`capShare` has a second writer somewhere that is looser than the first."** — **CLOSED.** `capShare` is a function-local in `MeasureSlipFromMemory` with exactly one assignment, `:1589`; `MeasureSlipFromTheory` has its own local `available` at `:1608`. There is no second writer to either. The nearest thing to what D describes is `ARS.SteerCeilingFactor`, the public static *inside* that expression, which has two writers — the menu `ARS:1097` and the settings load `ARS:3405` — and both write the same field from the same value; the menu offers values above 1.0 by design ("2.00 barely cuts at all", `ARS:1094`). No defect.

## Could not decide

- **F2's blast radius.** Whether `WheelsOffGround` is a live catastrophe or a mislabelled predicate depends on whether GTA's tyre model returns exactly `0.0f` for the side-slip angle of a wheel rolling straight. If it returns `0.0` exactly, every straight kills throttle in 0.5 s, which would be unmissable in play, so I suspect the field is *not* the slip angle on this build — but then the slip laws that read it are also wrong, and I cannot settle which from the source. Needs one log line, not a read.
- **B's magnitude.** The ~13 km/h per node figure needs a real track's node spacing and curvature deltas; `RouteRadiusSampled` `:3745` is in the source but the radius it returns is data-dependent. Shape verified, number not.
- **Whether `Control.Throttle = -0.5f` (`:4581`) actually reverses on this build.** `VehicleMemory.SetThrottle` writes the float at the control struct's throttle offset with no sign handling (`VehicleMemory.cs:28, 64`); whether the game reads a negative throttle as reverse is a runtime fact I cannot check read-only. If it does not, the entire reverse-recovery phase drives forward at half throttle — which would make F8 a blocker rather than a release-note.
- **`OffTrackSafeSpeedMph = 20f` vs `OffTrackFullThrottleMph = 15f`** (`:2452-2453`), used as `ARS.Remap(mph, 20, 15, 0, 1)` at `:2525`: the argument named "safe speed" is the one that yields **zero** throttle and the one named "full throttle" is the lower speed. Either the names are inverted or the remap direction is. I did not rank it because the behaviour (cut above 20 mph off-track, full below 15) is coherent and only the naming is suspect — it belongs in the naming pass, not here.
- **Whether `CountersteerPedalCap` applying to both pedals at once (`:1906-1907`) is intended.** No comment, and the two pedals then carry different glides; I reported the dead glided cap (F6) but not the symmetric cap itself.
