# Code clarity review

The user's exercise: pick apart sections of the code together and decide whether they can be
simplified or made clearer to a human reader **without changing behaviour**. The Lead picks one, the
crest-designer picks the next, alternating, until both are satisfied the agreed scope is covered.
Each round ends in an agreement, recorded here **before** either side writes code.

## Rules agreed up front

- **One method per round.** `Racer.cs` alone is 3,600 lines; anything wider stops being a clarity
  pass and becomes a codebase review.
- **This is not a vehicle for behaviour fixes already on file** — the live crest window at
  `Racer.cs:1543`, the stale `runPeak` at `TrackLoader.cs:385`, the repeated probe constants. If one
  of those lands in a round it lands as its own labelled change, never disguised as a clarity edit.
- **Write scopes stay disjoint**: this file is the Lead's, the design note is the crest-designer's.
- **Applying an agreed change is a separate decision** from agreeing it. These are proposals until
  someone asks for the code.

Scope order: newest first — the crest code written this session — then outward through the files
that work touched, then beyond.

## Round 1 — `MoveEntrancesToCrests`, `TrackLoader.cs:516` (Lead's pick)

**Defect.** A string carries control flow:

    string verdict = move < 1 ? "no move" : move > moveCap ? "over cap" : "applied";
    ...
    if (verdict != "applied") { ARS.Log(..., " (" + verdict + ")"); continue; }

`"applied"` appears three times (`:433`, `:436`, `:444`) and each typo fails **silently, differently**:

- in the first, every move becomes a skip;
- in the second, the previous-corner guard is dropped entirely, and a moved entrance lands inside the
  previous corner — the exact bug an hour of work went into preventing;
- in the third, `StartNode` is written for a move that was rejected.

None of the three is compile-visible. On top of that the nested ternary classifies three outcomes
with no name, and the guard fuses comparison with prose.

**Agreed shape.** Extract a pure classification, with two corrections to the Lead's first signature:

    bool CanMoveEntrance(CornerPoint corner, CornerPoint previous, int move, int count, out string skipReason)

- `move` is **passed in**, not recomputed: the log prints it, and duplicating the
  `IsPointToPoint ? … : Wrap(…)` expression is duplication that drifts.
- `previous` is **passed in**, not the loop index: resolving index 0 to the last corner on a circuit
  is a table detail and belongs in the loop, where the geometry already is.
- `moveMargin` and `moveCap` move to class scope as constants. They belong there anyway; no value
  change rides along.
- `skipReason` stays a **string**, not an enum. It is only ever concatenated, never compared, and
  `null` means allowed in exactly one place. An enum plus a formatter earns its keep only when a
  second consumer branches on the *kind* of skip — the moment anything compares the reason is the
  signal to convert.

**No-behaviour-change checklist** — each of these is easy to lose in the rewrite:

- `move < 1` must still **log**. It is a skip outcome today, so a bare `continue` would be a silent
  behaviour change.
- The previous-corner check runs **only when the move is otherwise allowed**, in the same order with
  the same two comparisons.
- The log keeps its exact `"Crest move skipped: node=… move=…m (…)"` format, and applied moves stay
  silent.

## Round 2 — `CrestDecelFactor`, `Racer.cs:3077` (crest-designer's pick)

**Defect.** `rising` and `falling` are neither sides nor areas: they are the raw integral *before* its
`1/(2h)` normalisation, and the meaning only appears at the division, so the reader has to
reverse-engineer an integral the surrounding comment already claims is there. Both expressions share a
shape with a different origin — `start` for the rising branch, `end` for the falling one.

**Agreed.** Extract the shared expression:

    static float RampIntegralUnscaled(float zeroAt, float from, float to)
    {
        return (to - zeroAt) * (to - zeroAt) - (from - zeroAt) * (from - zeroAt);
    }

`risingUnscaled` / `fallingUnscaled` at the call site, and the caller keeps
`/ (2f * halfExtent * spanMeters)`, which now visibly supplies the `1/(2h)` turning the unscaled
integral into the span mean. Same operations, same order: **bit-identical**.

**Corrected in drafting.** The Lead proposed `RampAreaDoubled`, and it is false. The ramp's slope is
`1/h`, so the raw expression is `2h ×` the area — at `h = 6` that is `12×`, not `2×`. Naming the value
for what it actually is, unscaled, is honest at every extent.

**On the bar itself.** Bit-identity is stronger than "no behaviour change" needs. A helper returning
the clipped mean directly would re-associate the final multiply and divide, shifting `unload` by a
last ulp — not observable downstream. Bit-identity is kept because it is free here, not because one
rounding would be a behaviour change.

## Round 3 — `CrestNode(int, int)` → `TryResolveNode`, `TrackLoader.cs` (Lead's pick)

**Defect, in two layers.** The helper shares an identifier with the `CornerPoint.CrestNode` field, so
the same word means two things a few lines apart. Sharper, and the crest-designer's catch: the
**sentinel** collides too — the helper returns `-1` for "past the end on point-to-point" while the
field uses `-1` for "no crest", and `AssignCrestNodes` reads both within twenty lines (`:382`,
`:399`). Renaming around that would leave the ambiguity intact under a better label.

**Agreed.** Kill the sentinel rather than rename it:

    static bool TryResolveNode(int node, int count, out int resolved)

Call sites become `if (!TryResolveNode(node, count, out int resolved)) break;`. The field keeps its
`-1`, because "no crest" is a genuinely different concept from "this node does not exist" — killing
one of the two beats renaming around both. It also matches the existing
`TryGetCornerContext(..., out ...)` convention rather than inventing a second one.

**The name was argued twice and the second attempt won.** The Lead proposed `TryWrapNode`; the
crest-designer pointed out it describes only the circuit branch — on point-to-point the method
deliberately does NOT wrap, it fails — while `ARS.IsPointToPoint` appears nowhere in the signature,
so a reader at the call site cannot tell which half they are getting. `TryResolveNode` states a
contract true in both modes: produce an index into `TrackPoints`, or false.

**Clamping ruled out.** Out-of-range currently ENDS the scan (`:382`, `:476`), so clamping would
continue it — a behaviour change, not clarity.

## Round 4 — `CrestCurvature` → `GsAtProbeSpeed`, `TrackLoader.cs:469` (crest-designer's pick)

**Defect.** It returns Gs, not curvature, and the field it feeds is documented in Gs
(`DataStructures.cs:395`).

**Agreed.** Rename only, body untouched. `CrestGs` was deliberately avoided: putting the field's own
name on the helper would rebuild the collision Round 3 removes, which is a neat demonstration that
these rounds interact.

**Recorded as a candidate, not folded in** — see below: its only caller tests `< 0f`, so the entire
probe-speed scaling exists to produce a sign there.

## Round 5 — `CrestBaseWidth`'s mirrored loops → `NodesToInflection` (Lead's pick)

**Defect.** Two anonymous `while`s (`:462-465`) that differ only in direction, so the walk's purpose
has to be inferred from the comment above it.

**Agreed.** Extract:

    static int NodesToInflection(int from, int step, int maxNodes, int count)

`CrestBaseWidth` then reads `NodesToInflection(peak, -1, run, count) + NodesToInflection(peak, 1, run, count) + 1`,
where the `+ 1` is visibly the peak itself. Behaviour-identical — the operand translation was checked
rather than assumed: `peak - left - 1` with `left` counting from 0 is exactly `from + step * (walked + 1)`
at step −1, with the same `limit` cap and the same `count` guard on both sides.

**The name was countered and mine lost again.** The Lead proposed `WalkToInflection`; the
crest-designer pointed out it returns a **count**, not a node and not a walk — the same class of lie as
Round 2's `RampAreaDoubled`. `NodesToInflection` puts the return value in the name and `maxNodes`
says what the cap is a cap on.

## Round 6 — `TryResolveNode` re-implements `Wrap`, `TrackLoader.cs:482` (crest-designer's pick)

**Defect.** The helper inlines `((node % count) + count) % count` while `Wrap` already exists at
`:712`, so the file carries two implementations of the same index arithmetic.

**Agreed.** Replace the inline form with `Wrap(node, count)`. Behaviour-identical — verified for every
sign, including `index = -count` — one line, and the helper then reads as what it is: a point-to-point
guard wrapped around `Wrap`.

## Round 7 — `MoveEntrancesToCrests`'s previous-corner lookup (Lead's pick)

**Defect.** A doubled ternary inline in the loop —
`i > 0 ? ARS.Corners[i - 1] : ARS.IsPointToPoint ? null : ARS.Corners[ARS.Corners.Count - 1]` — the
last unnamed piece of the method Round 1 covers.

**Agreed, and not by extracting.** The Lead proposed `PreviousCorner(int index)`; the crest-designer
countered that such a helper hides `ARS.Corners` and `ARS.IsPointToPoint`, has exactly one caller, and
would be the only impure private helper in a file whose others are all pure (`Wrap`,
`NodesToInflection`, `AimsSameWay`) — a parameterless form reading static state is the exact shape
Round 3 removed. With nothing to de-duplicate, extraction buys a name at the cost of hidden state.
Replaced instead with a chain visible at the point of use:

    CornerPoint previous = null;
    if (i > 0) previous = ARS.Corners[i - 1];
    else if (!ARS.IsPointToPoint) previous = ARS.Corners[ARS.Corners.Count - 1];

If a name is ever wanted, the pure form is
`PreviousCorner(IList<CornerPoint> corners, int index, bool wraps)` — the mode in the call, per Round 3.

## Round 8 — a dead condition in `CrestDecelFactor`'s overlap guard, `Racer.cs:3089` (crest-designer's pick)

**Defect.** `if (end <= 0f || start >= spanMeters || hi <= lo) return 1f;` — the third condition is
unreachable. **Proof, verified:** assume the first two false, so `end > 0` and `start < spanMeters`. If
`start >= 0` then `lo = start` and `hi = min(end, spanMeters) > start = lo`, since `halfExtent >= 1`
makes `end > start` always and `spanMeters > start` by assumption. If `start < 0` then `lo = 0` and
`hi > 0 = lo`. Either way `hi > lo`. The only inputs that could reach it — `halfExtent < 0`,
`spanMeters < 1` — the method does not admit.

**Agreed.** Delete it, silently. A comment explaining why an impossible thing does not happen is worse
than the deletion.

**Agreed in the same round**, being the same method: replace the trailing `Math.Min(unload, 1f)` with an
early `if (corner.CrestGs >= 0f) return 1f;`. With a non-negative `CrestGs` excluded,
`unload = 1 + CrestGs * ratio * ratio` can only fall below 1, so that `Min` was already a no-op for
every value the method accepts — the guard now *names* the dip case instead of defending against it at
the bottom. Observably identical.

**Not touched, deliberately:** `peak = ARS.Clamp(toCrest, lo, hi)` earns its place. On point-to-point
`toCrest` goes negative once the car is inside the crest, and the clamp moves the peak to 0.

## Round 9 — the run state in `AssignCrestNodes`, `TrackLoader.cs:377` (Lead's pick)

**The Lead's framing was wrong and the correction is the round's value.** The three exits are not the
defect. A dip (load returned, scan over), an unmarked node (run ended, scan continues) and out-of-range
(track ended, scan over) are three genuinely different outcomes, and naming or merging them would hide
the difference that stops the next bug being written.

**The real defect: `run`, `runPeak` and `runPeakGs` are three variables that must be reset together,
and Round 2 of this night's work forgot one.** One value makes the reset atomic and the bug class
structurally impossible:

    (int peakNode, float peakGs, int length) run = (-1, 0f, 0);
    ...
    if (g > -threshold)
    {
        if (run.length >= minRun) break;
        run = (-1, 0f, 0);
        continue;
    }
    run.length++;
    if (run.peakNode < 0 || g < run.peakGs) run = (node, g, run.length);

Same operations, same order, same stored crests; the loop loses two locals and the reader stops holding
three coupled values. A private `CrestRun` struct with a `None` value is the identical fix if a value
tuple is against house style.

**Also deliberately not done:** the dip `break` and the out-of-range `break` stay separate even though
both end the scan — on a circuit the out-of-range case can never fire, and a merged exit would conceal
that.

## Cluster closed — what "done" means here

Both sides are satisfied the crest cluster is covered. **Done means the note is complete, not the code.**
None of these agreements is in the tree; the apply pass is its own step and is exactly where a recorded
clarity edit drifts from what was agreed — the failure this exercise exists to catch. Apply in round
order, then the normal gate: compile, drive, commit. These are code changes and do not qualify for the
documentation exemption.

**Stopping here, by agreement.** The crest cluster was the newest and least reviewed, so it earned the
exercise. `Racer.cs` is 3,600 lines and "outward" has no natural end, so another round would sprawl or
need an arbitrary cap. A future clarity round should be a one-off aimed at a seam that has already
confused a reader — clarity passes are cheapest where a bug has already passed through.

**Left as labelled items, not rounds:**

1. `ForwardDistance(from, to, count)` — the ~20 `ARS.IsPointToPoint ? a - b : Wrap(a - b, count)` sites.
   The biggest single clarity win remaining, and the one that wants tests.
2. `window = 6` single-homed (`TrackLoader.cs:363`, `:472`) — same treatment the probe constant got.
3. The sign-only consumer of `GsAtProbeSpeed`, which would remove the probe from `CrestBaseWidth`.
4. The live-window behaviour fix at `Racer.cs:1543` — still open, and not clarity.
5. The bump-angle and launch detectors — designed, unbuilt; both need the unthresholded distribution log
   before any threshold is chosen.

## Candidates — real simplifications, deliberately not taken in a clarity round

- **`ForwardDistance(from, to, count)` — the real prize.** The pattern
  `ARS.IsPointToPoint ? a - b : Wrap(a - b, count)` appears roughly twenty times (`:432`, `:549`,
  `:652`, `:659` and on). They are all one operation: a signed distance on point-to-point, a wrapped
  one on a circuit. Cross-method by nature, so it fails the one-method rule and wants a labelled
  change or several rounds.
- **The probe-speed scaling may exist only to produce a sign.** `GsAtProbeSpeed` has exactly one
  caller, `CrestBaseWidth` (`:463`, `:465`), and both calls test `< 0f` — the magnitude is never used.
  Reducing the walk to a sign test changes that method's body and its return unit, so it is a
  labelled change, not a rename. The cheapest real simplification left in this cluster.
- **`window = 6` is still duplicated** (`TrackLoader.cs:363`, `:472`), and the scan and the base walk
  must agree on it for the two to stay consistent. Same class as the probe constant, which is now
  single-homed via `ARS.CrestProbeSpeed` — so it lands labelled or not at all.

**Closed candidates**

- **A `TryGetSampleWindow(centre, window, count, out before, out middle, out after)` for the
  three-sample pattern** (`:379-383`, `:473-477`) — dropped, and Round 3 is why. Once the Try
  conversion lands, the two sites are three early exits that each name their failure. A single window
  helper must assign all three `out` parameters on every path, so the failed ones need placeholder
  values — reintroducing exactly the dummy-sentinel smell Round 3 removed — to buy a shorter spelling
  of a three-line idiom used twice. The honest alternative, a nullable tuple, trades the sentinel for
  a null and does not match `TryGetCornerContext`.
