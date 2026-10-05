# Audit — where compute-once is not obeyed

**Status: analysis only. Nothing implemented, nothing changed in code.**

The principle: **compute a value once, publish it, and let consumers take it under an explicit staleness
budget** — rather than each consumer re-deriving it from raw inputs whose freshness nobody tracks. This is
the inventory of where the codebase still violates it and could stop.

Method: three read-only passes over the tree, on disjoint slices — duplicate native reads, re-derivation and
unpublished state, and per-tick rebuilding (allocations and LINQ). Findings below are merged and de-duplicated
across slices, and a spot-check pass confirmed the counts where the passes disagreed. Cadence model for all
arithmetic: a 30-car grid at 60 fps, where `RunTimedCore` runs six racers a frame (`AutosportRacingSystem.cs:1882`)
— 360 car-cores/s, ~12 Hz per car — while `ProcessTick` runs every racer every frame and does no steering or
speed work.

## Tier A — largest, and safely reversible

| # | What | Where | Cadence | Cost today | Fix |
|---|---|---|---|---|---|
| 1 | The leaderboard re-sorts the field and rebuilds ~180 strings | `AutosportRacingSystem.cs:2056`, `:2047-2049`, `:2070-2077` | per frame | ~11,000 strings/s and 60 sort buffers/s at 30 cars | `RacePosition` is already published every 200 ms (`:1924-1936`) — iterate that order under its staleness budget; reuse a scratch list and string builder |
| 2 | Node→corner lookup walks the table with a boxed enumerator and a delegate | `Racer.cs:1524`, `:2044`, `:3089`, `:3134`, `:3183` (five sites) and the `Any` at `:2901` | per car-core, several times | ~2,500 calls/s → ~5,000-7,500 gen0 objects/s | publish a `Dictionary<int, CornerPoint>` keyed to `CornersRevision` — the revision already exists (`Racer.cs:252-260`, `TrackLoader.cs:351`) and *is* the staleness budget |
| 3 | `GetWheelPtrs` builds a fresh list, three times per car-core | `AutosportRacingSystem.cs:2764` via `:2782`, `:2797`, `:2847` | 3 × per car-core | ~1,080 lists/s, ~86 KB/s | a reusable per-car buffer — wheel pointers and count are stable per vehicle |
| 4 | `ComputeRubberBandFactor` resolved twice in one core tick, each call looping every racer | `Racer.cs:1670` and `:1337` | 2 × per car-core **when enabled** | ~10,400-41,800 natives/s **when on; exactly zero by default** (`RubberbandingPct` starts 0, early return `:1703`) | hoist the player lookup, or return the factor from the first call and hand it to the second; `PlayerRacer` is already cached (`AutosportRacingSystem.cs:1684`) |
| 5 | The car's own pose and every rival's pose re-read per use | `Racer.cs:508-669` (own velocity ×8, forward ×2, position ×4), `:558-576` and `:650-682` (≈13 rival reads where ~4 serve) | per car-core, per rival | ~11,500 natives/s | three locals at the head of `ComputeSteering`, a per-rival snapshot, and the existing `EntityRelativeOffset(Entity, Vector3)` overload (`AutosportRacingSystem.cs:555`) |
| 6 | `GetForwardSpeed` computed independently across six sites | `Racer.cs:589`, `:1131`, `:2305`, `:2605`, `:2633`, `:3645` | per frame and per car-core | ~5,400/s from the two proven same-frame sites alone; more if the others share a tick | publish the forward speed once per tick and pass it down |
| 7 | A ~200-character line is built before the log level is consulted | `Racer.cs:698`, reached from `:708-750` via `ComputeSteering:548` | per corner crossing, per car, per lap | ~2,700 strings/lap at 30 cars, ~0.5 MB per race, plus an always-on O(corner table) scan | the call is self-labelled temporary; delete it or gate on the level first — the default level makes the logging itself free but not the concatenation |
| 8 | `UpdateApexLeapfrog` allocates two 3-element arrays and `Any` closures | `Racer.cs:2898-2901` | per car-core, every tick | ~2,500 objects/s, ~100-125 KB/s | two reused fields (both are recomputed wholesale anyway) plus a manual loop over the node→corner map |
| 9 | `UpdatePressure` re-reads its own position once per candidate inside the loop | `Racer.cs:3397` | 2 Hz per car | ~3,480 natives/s at 30 cars, half of it redundant | one local above the loop — already recorded in `PLAN-RATE-PRECISION.md` |
| 10 | `Rival.UpdateOffsets` reads the rival's position twice | `DataStructures.cs:190` then `:192` | up to 3 Hz per car | ~260 natives/s | the held `rivalPosition` with the overload at `AutosportRacingSystem.cs:555` |
| 11 | The model top speed is re-read from a native every core tick | `AutosportRacingSystem.cs:3267` via `Racer.cs:3650` | per car-core | one native per downforce car per core tick, while `ModelTopSpeedMphCache` (`AutosportRacingSystem.cs:71`) holds the same value | read the cache — mind the mph→mps round trip |
| 12 | `AverageAcceleration` re-sums its whole window on every read | `DataStructures.cs:33-42`, readers `Racer.cs:541`, `:931-932`, `:1388`, `:2942`, `:3675` | ~5 × per car-core | ~50 vector adds per car-core | publish once per tick — the samples only move every 20 ms (`Racer.cs:2238`) |
| 13 | Debug drawing re-fetches model dimensions the constructor already cached | `Racer.cs:2271`, `:2439` against the cache at `:354` | per debug frame | one native per debug frame | read `VehicleData.ModelDimensions` |
| 14 | `ProjectAhead` at the same preview time recomputed for each consumer | `Racer.cs:541`, `:931-932`, `:1388` | per car-core | small | publish once per tick |

## Tier B — small, or conditional

- `CommitApexQueue` allocates a `Corner` per call (`Racer.cs:3097`), ~360 objects/s — a reusable field.
- `RefillApexQueue` builds three lists, a sort closure and two array triples per refill
  (`Racer.cs:2991-3050`), ~500 objects/s at 2 Hz — reusable scratch fields.
- The card chains re-project the three rival slots seven ways (`Racer.cs:1657-1660`, `:1671`, `:1969`,
  `:2033-2036`, `:2084-2090`, `:2113-2123`, `:2138-2141`) — small, because the whole block runs at **1 Hz per
  car**, not per tick. A single manual scan would serve them all, and ties must keep the first-match rule.
- `TryPlayNitrousCard` reads the same rival velocity twice inside one predicate (`Racer.cs:2055-2056`).
- `RivalsWithinDistance` reads its own position once per element inside a LINQ `Count` (`Racer.cs:299-303`).
- `IsUnstable` and `IsFullCountersteer` are each evaluated twice per core tick (`:1781`/`:1219`, `:1324`/`:1853`).
- Adjacent-line duplicates of `Car.Velocity.Length()`: `:1612-1613`, `:2866-2867`, `:1783`/`:1802`.
- Debug-only strings and a per-frame `DebugFocusRacer` re-projection (`:2362-2371`, `:2514-2517`,
  `AutosportRacingSystem.cs:1900`), and `Racers.Count(r => !r.IsDNF)` twice per frame (`:1940-1941`).

## Tier C — needs care, do not treat as a free swap

- **`UpdateCornerRequirements` walks the whole corner table every car-core** (`Racer.cs:2945`) with flags that
  latch and are never reset. It is the largest per-frame cost that scales with track length, but bounding the
  scan can miss a flag on approach, so the fix has to preserve the in-range test exactly.
- **`VehicleData.SpeedVectorLocal.Y` is not a drop-in for `GetForwardSpeed`** — the local Y carries pitch while
  the helper projects horizontally, so substituting them changes behaviour on grades.
- **`UpdatePerceivedGrip` re-reads `GET_ENTITY_ROTATION_VELOCITY` per car-core** (`Racer.cs:3689`). Belongs to the native-read caching stage rather than a local fix.
- **`Rival.RelativeOffset` and `LateralGap` are published, but the side-by-side correction re-derives them**
  (`Racer.cs:657` against `DataStructures.cs:192-194`) — reuse would hand it a value up to 500 ms old, and a
  rival that just pulled alongside would be missed.
- **`CombinedSize` is not a clean merge with the repulsion gates or the passenger-seat length**
  (`Racer.cs:564-565`, `:2544-2552`) — the buffers differ (1.0/0.25 against +3) and the repulsion loop's frame
  is velocity, not the car's forward.
- **One car's forward speed has three homes with three reference frames** — `VehicleData.SpeedVectorLocal.Y`
  (`Racer.cs:1305`, `:2238`), the horizontal projection in `GetForwardSpeed` (`AutosportRacingSystem.cs:3311`),
  and a raw 3D dot (`Racer.cs:1175`, `:1238`, `:3581`). Unifying them changes what the steer ceiling and the
  pedal plan read on a slope.
- **`TryPlayNitrousCard` re-derives a closure time the route frame already publishes** (`Racer.cs:2056`
  against `Rival.TimeToReach`, `DataStructures.cs:283`) — world scalars against a route-frame gap with a lane
  gate, so not equivalent.
- **`UpdatePressure` rescans every racer while the three nearest already publish `Distance`**
  (`Racer.cs:3394-3400` against `DataStructures.cs:201`) — a different set, so not equivalent.

## Tier D — the codebase disagreeing with itself

Not cost. These are cases where a value or a law exists twice, or where one of the copies is dead. The first
is the highest-value item in the whole audit precisely because it is not about speed.

- **`PowerScale` is re-derived at `Racer.Initialize`** (`Racer.cs:481-487`) from three live model natives, while
  `ARS.ModelPaceIndexCache` (`AutosportRacingSystem.cs:84`, filled `:493`) already holds the value the grid was
  selected on, and `TryComputePlayerCarPaceIndex` (`:3186`) already implements cache-first with a native
  fallback. **The comment above the line states the intent — "Cache first so the spawn-time PI matches the metric
  the grid was selected on; live probe only for an unscored car" — and the code does not do it**: the only cache
  consulted there is `ModelElectricCache`, for the electric flag, so the pace index itself is always probed live.
  The player and the AI can therefore derive the same number by different routes. **Highest-value item in the
  audit, and it is a consistency bug rather than a speed one.**
- **Two methods are byte-identical**: `RouteIdealSpeedForRadius` (`Racer.cs:3239`) and `ApexSpeedWithDownforce`
  (`:3246`). A third copy, `ARS.CornerApexSpeed` (`AutosportRacingSystem.cs:2675`), carries different speed
  clamps — merge the first pair, leave the third alone.
- **An unreachable branch makes a whole method dead**: the `else if (Brain.Corner != null)` at `Racer.cs:1589`
  cannot be taken, because `Brain.Corner` is only assigned non-null where `NextApexNode >= 0` (`:3097`, nulled
  at `:3101` and `:1248`) — which makes `ARS.MaxSpeedForBrakingDistance` (`AutosportRacingSystem.cs:2917`) dead.
  It also re-implements the solve `ApexBrakingSpeed` (`Racer.cs:3129`) already does with different terms, so
  delete rather than repoint.
- **`VehicleData.PerformanceIndex` is write-only** (`DataStructures.cs:96`), fed by two natives (`Racer.cs:478-480`)
  and read nowhere.
- **`Rival.LateralGap` is written every update and read nowhere** (`DataStructures.cs:158`, `:194`) while the
  one consumer that wants it re-derives it (`Racer.cs:657-671`).
- **`TrackProgress` is write-only** (`Racer.cs:61`, `:2701-2702`); `RaceProgress` is what every consumer reads.
- **Four dead hill helpers** (`AutosportRacingSystem.cs:2562`, `:2591`, `:2614`, `:2644`) duplicate an `atan2`
  climb angle that the live `Racer.GetFollowPointSlopeAngle` (`Racer.cs:1716`) computes.
- **`GetPreciseRadius` has no callers** (`AutosportRacingSystem.cs:2499`) and re-derives `TrackPoint.PreciseCurveRadius`,
  already stored per node (`TrackLoader.cs:213`).
- `OccupiedLaneWidth = CombinedSize.X` (`DataStructures.cs:199`) is two fields for one number — noted only so
  the pair is not mistaken for independent values.

## Already obeying the principle — do not "fix" these

`OccupiedLane` is the rival's own `DeviationFromCenter` (`DataStructures.cs:200`); `CombinedSize` is built from
both cars' own cached dimensions (`:197-198`); the route gap reads the rival's own `CumulativeDistance` (`:209`)
and the closure its own `AlongTrackSpeed`; `Rival.Update` reads each entity vector once and hands it down
(`:175-183`). `Entity.Handle` is a stored value in SHVDN2, not a native, so handle comparisons cost nothing.

## Method notes

Each of the three passes **under**counted where it was checked: four corner-lookup sites where there are five,
two `GetForwardSpeed` calls where there are six. The searches are sound; the counts need re-verification at
implementation time, which is why every row carries its `file:line`. The re-derivation pass worked from the
current working tree, whose lines have drifted from `AGENTS.md`'s anchors, so expect the same drift here.

Nothing here has been built or driven. Anything in Tier A that touches steering or rival data is a behaviour
change by definition and needs its own drive, even when the arithmetic is identical.
