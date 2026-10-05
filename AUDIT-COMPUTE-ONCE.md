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
- **The model top speed is read from a native per car-core** for a value fixed per vehicle model
  (`AutosportRacingSystem.cs:3267`, reached from `UpdatePerceivedGrip`), and `UpdatePerceivedGrip` re-reads
  `GET_ENTITY_ROTATION_VELOCITY` per car-core (`:3689`). Both belong to the native-read caching stage rather
  than to a local fix.

## Already obeying the principle — do not "fix" these

`OccupiedLane` is the rival's own `DeviationFromCenter` (`DataStructures.cs:200`); `CombinedSize` is built from
both cars' own cached dimensions (`:197-198`); the route gap reads the rival's own `CumulativeDistance` (`:209`)
and the closure its own `AlongTrackSpeed`; `Rival.Update` reads each entity vector once and hands it down
(`:175-183`). `Entity.Handle` is a stored value in SHVDN2, not a native, so handle comparisons cost nothing.

## Method notes

Both the native-read and allocation passes **under**counted where they were checked: four corner-lookup sites
where there are five, two `GetForwardSpeed` calls where there are six. The searches are sound; the counts need
re-verification at implementation time, which is why every row above carries its `file:line`.

Nothing here has been built or driven. Anything in Tier A that touches steering or rival data is a behaviour
change by definition and needs its own drive, even when the arithmetic is identical.
