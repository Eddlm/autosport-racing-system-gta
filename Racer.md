# Racer Refactor TODO

Goal: simplify `Racer.cs`, reduce hidden state, and make behavior readable top-to-bottom.
Scope in this file only for now. No behavior rewrite in this list unless explicitly noted.

## 0) Immediate Cleanups (before structural refactor)

1. Remove/disable dead debug paths currently left in active flow.
- `SteerTrack()` currently hard-sets `targetLane = 0` and comments out rival logic.
- Decide if this is temporary test mode or intended baseline.
- If temporary, move to a feature flag (`DEV`) to avoid accidental commit of broken flow.

2. Remove stale fields no longer used in active path.
- `TargetLane` field is legacy from `OldSteerTrack()` and not needed as class state.
- Keep lane targets as method locals unless another method truly consumes them.

3. Remove unused constants/state.
- `StuckNoProgressTimeMs`, `LastProgressNode`, `LastProgressTime` are mostly vestigial with current stuck condition.
- Either reintroduce no-progress logic fully, or delete these.

## 1) Top of File: Data Layout and State Ownership

4. Group fields by domain and make intent explicit.
- Identity/runtime refs (`Car`, `Driver`, `Name`, team)
- Navigation/control (`CurrentTrackPoint`, `LookAheads`, lane bounds)
- Dynamics (`vData`, `Handling`)
- Recovery/stuck state
- Debug draw caches (`trailSamples`, lane trails)

5. Move `TrailSample` to `DataStructures.cs` (or keep private but at end of class fields).
- If private here: keep near debug-trail fields only.
- If shared later: promote to common data file.

6. Normalize naming style.
- Fix mixed casing in locals: `AngleOneSec`, `KeepInside`, `DistToRear` (legacy style).
- Use one style consistently (`angleOneSec`, `keepInside`, `distToRear`).

## 2) Constructor + Initialize + Launch

7. Constructor does too much setup and resilience policy.
- Split into private helpers called from constructor:
  - `SetupDriverDefaults()`
  - `SetupVehicleSafety()`
  - `SetupBlip()`
  - `SetupBehaviorVariance()`

8. `Initialize()` + `Launch()` overlap in reset responsibilities.
- Create one reset helper for runtime control state:
  - `ResetControlStateForRaceStart()`
- Keep mechanical/static init (`handling` reads) separate from race-state reset.

## 3) Steering Section (main readability win)

9. Delete `OldSteerTrack()` after confidence period.
- It doubles cognitive load and still carries many stale branches.
- If needed for rollback, keep in branch history, not in active file.

10. Keep `SteerTrack()` as a single readable pipeline.
- Context gather
- Base lane source (track/corner)
- Rival constraints
- Final lane clamp
- Steer target build
- Steering angle output

11. Local helper functions inside `SteerTrack()` are good; keep only used ones.
- Remove local functions not called in current mode.
- If helpers get reused elsewhere, extract to class-level private methods.

12. Use one reference frame for lane planning.
- Decide and document one planning node (`HalfSec` or `SteerRef`) for profile lookup.
- Keep trail visualization and lane lookup using the same node basis when comparing overlays.

13. Remove silent behavior toggles in-line.
- Commented code inside `SteerTrack()` should become explicit `if (devFlag)` blocks or be removed.

## 4) Steer post-processing and input mapping

14. `SteerCorrections()` currently has an early `return`.
- If intentionally disabled, remove body and keep stub with clear comment.
- Or re-enable fully and remove dead code warnings/branches.

15. Keep rate limiting in one place only.
- Current limiter in `TranslateSteer()` is good; document it as "final actuator limiter".
- Avoid adding additional hidden steer rate limits elsewhere.

16. `TranslateSteer()` should be pure “degrees -> input”.
- It should only sanitize, rate-limit, map, and set `SteerInput`.
- No navigation policy should leak in.

## 5) Speed control block

17. `SpeedTrack()` mixes corner speed, projected path confidence, and anti-outward limiter.
- Split into private compute helpers:
  - `ComputeCornerSpeedLimit()`
  - `ComputePathSpeedLimit()`
  - `ApplyOutwardCornerSpeedLimiter()`
- Keep final `mem.intention.Speed = min(...)` clearly visible.

18. Remove unused locals in speed math.
- `latGs`, `speedAdjust`, and any placeholder calculations not feeding outputs.

## 6) Tick loop organization

19. Make one authoritative call order document block in `ProcessAI()`.
- Mark each stage with numbered comments:
  1) perceive
  2) decide steer
  3) decide speed
  4) corrections
  5) translate inputs
  6) stuck/recovery overrides

20. Ensure recovery override ordering is intentional and singular.
- `ApplyStuckRecoveryOverride()` is called in both `ProcessAI()` and `ProcessTick()`.
- Decide one location only to avoid confusion and conflicting writes.

## 7) Draw/Debug region

21. `DrawStuff()` remains too large.
- Split into gated helpers:
  - `DrawPhysicsDebug()`
  - `DrawAggroDebug()`
  - `DrawInputDebug()`
  - `DrawTrackAnalysisDebug()`
  - `DrawPlayerTrackLimitsDebug()`

22. Keep all trail sampling outside draw if possible.
- Update trail data in tick/update stage, not inside draw path.
- Draw methods should consume immutable snapshots.

23. Unify lane debug arrow/trails reference node.
- Decide one source node for comparison mode.
- If comparing different references intentionally, label each color in comments.

## 8) Follow-track and rival info

24. `UpdateFollowTrack()` does too many concerns.
- Split into:
  - `ResolveCurrentTrackPoint()`
  - `BuildLookAheads()`
  - `UpdateLapState()`
  - `UpdateProjectedCurveRadius()`

25. `UpdateRivalInfo()` and `UpdateRivals()` naming overlap is confusing.
- Rename for clarity:
  - `RefreshRivalSlots()` (selection)
  - `RefreshRivalMetrics()` (per-rival update)

## 9) Stuck/recovery module

26. Formalize stuck state machine.
- States: `Idle`, `Tracking`, `Recovering`, `EscalationKick`.
- Current logic is split across 4 methods with duplicated transitions.

27. Consolidate recovery-attempt escalation trigger.
- `HandleRecoveryAttemptEscalation()` should be called from one transition point only.
- Keep attempt increment/reset rules in one method.

28. Move magic numbers to named constants.
- 10 mph reset, 15 mph kick, +1.5 Z boost, 2 attempts threshold.

## 10) Lifecycle and deletion

29. `Delete()` should null-check and clear blips/tasks consistently.
- Ensure no lingering references in ARS collections after delete.

30. Keep risk-factor methods near speed policy section.
- `RiskFactorForGrip/Brake` currently sit at bottom; relocate near `SpeedTrack()`.

## 11) Merge-only-once guidance (requested)

31. Methods currently called once and good candidates to inline/merge into caller sections:
- `UpdateDynamicBoundingBox()` -> inline into `RunTimedCore()` perception block.
- `UpdateCornerInfo()` -> merge into timed update block (or keep as module if continuing to grow).
- `HandleRecoveryAttemptEscalation()` -> merge into stuck transition method if it stays tiny.

32. Methods to keep separate even if single-caller (for clarity):
- `TranslateSteer()`
- `SpeedToThrottleBrake()`
- `TractionControl()`
- `UpdateFollowTrack()`
- `DrawInputTrails()`

## 12) Suggested refactor order (safe and incremental)

1. Remove test/debug hardcoded branches in `SteerTrack()`.
2. Delete `OldSteerTrack()` and `TargetLane` field.
3. Split `DrawStuff()` into helper methods with same gating.
4. Split `UpdateFollowTrack()` concerns.
5. Consolidate stuck/recovery transitions.
6. Final pass: naming cleanup + dead code removal.

## Notes

- Keep behavior checkpoints after each step (ingame quick lap + stuck test).
- Prefer "move-only" refactors first, then logic changes.
