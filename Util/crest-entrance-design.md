# Crest-before-corner entrance — Council design note (merged)

**Position: AGREED.** One `CornerPoint` field carries the crest — not new schema: `CrestGs` is declared
(`DataStructures.cs:396`) and never written, like `RequiresEarlyBrake`/`RampEndNode`. Decel term first
(the physics fix), bounded entrance move second; both read the one field.

## Sign, probe speed, window

`HillGripDeltaGs` (`AutosportRacingSystem.cs:2458`) takes a 2D cross in the (horizontal, Z) plane
(`:2483`), negative for convex-up: **negative = crest** (the lightest point, not the elevation
maximum), matching `CrestGs`; `CrestGripSpeedFactor` already discards positive values
(`Racer.cs:1609`).

`deltaGs` scales with v² (`:2488`) and no car exists at generation, so the scan needs a **fixed probe
speed, 40 m/s**: `CrestGs` is written at that probe and rescaled `(v/40)²` at read — the curvature stored
earlier, reviving the declared field. Window **±6 nodes (~13 m)**: `SaveRoute`'s 2-decimal Z noise is
~0.02 G there, against ~1 G per 5 cm at ±3.

## The lever

The plan demands apex speed at the entrance less `BrakeTargetLeadMeters` (`Racer.cs:1429`, `:3054`)
while its decel is grip·g·factor + grade only (`:3061-3074`) — it brakes to a plan that believes in
full grip. Moving the anchor buys distance, not correctness, and a decel term has no cap to lift, so the
oscillation objection misses. Consumers: `BrakingDecel` via `CrestDecelFactor`, and an approach
multiplier centred on `CrestNode`.

`StartNode` is not free, and "never cross the previous `EndNode`" is too loose: the binding conflict is
the previous corner's **apex**, because an entrance inside it demands this apex's speed while the car is
still in that one. Clamp at least 10 nodes forward of the previous apex.

## Placement, move, side effects

Post-pass after the chicane loop (`TrackLoader.cs:325-347`), before `CornersRevision++` (`:349`);
`LengthStart` synced. Cap **60 m** — beyond it route speed's own crest check (`Racer.cs:1533`) owns the
crest. Cluster ≥3 contiguous nodes, nearest cluster wins, one crest to one corner.

Three side effects: positioning/queue windows read the entrance (`Racer.cs:2843-2856`, `:485`, `:1988`,
`:2017`), so a long move puts the outside hold on the straight or crest — it reads as a lane bug; the
live crest check (`Racer.cs:1543`) spans entrance..exit, so a moved entrance turns it into a 100 m fit
averaging to zero that **hides the apex crest** — keep a short apex window; and the brake-learning slide
gate (`Racer.cs:1354-1360`) spans the straight, blaming crest slides on the corner — accepted.

## Pseudocode

```
// pass 1 — geometry only, after merging and the chicane loop, before CornersRevision++
for each corner c in Corners:
    c.CrestNode = -1 ; c.CrestGs = 0
    entrance = CornerEntranceNode(c)
    cluster = 0
    for n = entrance-1 down to entrance-CrestScanBackNodes:        // wrap on circuits
        g = CrestGsAtProbe(n-CrestWindowNodes, n, n+CrestWindowNodes)   // v = CrestProbeSpeed
        if g >= +CrestGsThreshold: break                          // a dip: load returns, stop
        if g <= -CrestGsThreshold: cluster++; if cluster == 1: near = n; continue
        if cluster >= CrestClusterMinNodes: c.CrestNode = near; c.CrestGs = -g; break
        cluster = 0

// pass 2 — bounded entrance move, after pass 1
for each corner c with c.CrestNode >= 0:
    if wrapDist(entrance - c.CrestNode) > CrestMoveCap: skip
    if wrapDist(c.CrestNode -> c.Node) < SpanMinimumMeters: skip
    if wrapDist(previousApex -> c.CrestNode) < CrestPrevApexClearanceNodes: skip
    if crossed(previous.EndNode) or crest claimed: skip
    c.StartNode = c.CrestNode; c.LengthStart = wrapDist(c.Node - c.CrestNode); crestClaimed = true

// consumers of the single field
unloadGs = c.CrestGs * (v / CrestProbeSpeed)^2        // v = live arrival speed at CrestNode
decel *= CrestDecelFactor(unloadGs)                   // BrakingDecel, Racer.cs:3061
approachSpeed *= CrestGripSpeedFactor(unloadGs)       // centred on CrestNode
```

## Named constants

| Constant | Value | Reason |
|---|---|---|
| `CrestProbeSpeed` | 40 m/s | fixed scan speed; deltaGs scales with v² |
| `CrestWindowNodes` | 6 | ~13 m chord; Z noise ~0.02 G |
| `CrestGsThreshold` | 0.15 g | above flat-road noise |
| `CrestScanBackNodes` | 100 | user's bound; live bound is a dip |
| `CrestClusterMinNodes` | 3 | a lone node is a spike |
| `CrestMoveCap` | 60 m | past it route speed owns it |
| `CrestPrevApexClearanceNodes` | 10 | keep out of the previous apex |
| `CrestDecelFactor` | `1 + unloadGs/grip` | decel must fall with unload |

## Falsification in game

Log per corner at 10 Hz: `CrestNode`, `CrestGs`, `unloadGs`, `StartNode`, brake-onset node, crest speed,
`cornerSpd`, `HasPassedBrakingTarget`, `BrakeFactor`, `RequiresPositioning`, peak `SlideAngle`.

1. **Mechanism** — brake onset moves earlier by the added span; else route speed or the release gate
   (`Racer.cs:1519`) governs, not the anchor.
2. **Sign and physics** — crest speed falls, and measured decel under the plan's assumption ⇒
   `CrestDecelFactor`, not the entrance.
3. **Noise** — deltaGs at ±3 vs ±6 on flat ground (LSIA); ±6 over 0.15 g there falsifies the threshold.
4. **Lane** — `RequiresPositioning` must not engage before the straight, nor hold on the crest.
5. **Apex crest and learner** — the live factor at `Racer.cs:1543` is unchanged by a moved entrance;
   `BrakeFactor` does not drift on an unchanged track.
