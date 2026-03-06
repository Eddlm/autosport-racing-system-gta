# Racer Refactor TODO

Goal: simplify `Racer.cs`, reduce hidden state, and make behavior readable top-to-bottom.
Scope in this file only for now. No behavior rewrite in this list unless explicitly noted.

## 1) Top of File: Data Layout and State Ownership

6. Naming pass follow-up only.
- Main local-casing cleanup is already done.
- Keep enforcing camelCase for new locals during future edits.

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

9. Keep `SteerTrack()` as the only steering planner (already done).
- `OldSteerTrack()` was removed from active code.

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

13. Keep `SteerTrack()` free of commented behavior toggles (already true right now).

## 4) Steer post-processing and input mapping

14. `SteerCorrections()` has no early return now; keep it intentional and trimmed.
- Remove placeholder/unused calculations (`isUndersteering`, physics temp vars) if they stay unused.

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
- `HandleRecoveryAttemptEscalation()` is currently called from two places, so treat it as shared unless transitions are consolidated first.

32. Methods to keep separate even if single-caller (for clarity):
- `TranslateSteer()`
- `SpeedToThrottleBrake()`
- `TractionControl()`
- `UpdateFollowTrack()`
- `DrawInputTrails()`

## 12) Suggested refactor order (safe and incremental)

1. Split `DrawStuff()` into helper methods with same gating.
2. Split `UpdateFollowTrack()` concerns.
3. Consolidate stuck/recovery transitions.
4. Final pass: naming cleanup + dead code removal.

## Notes

- Keep behavior checkpoints after each step (ingame quick lap + stuck test).
- Prefer "move-only" refactors first, then logic changes.
