# VERDICT 31-32 — steering hot path, settings, docs and dead code

**Date**: 2026-10-27.
**Reports adjudicated**: `docs/audit/31-steering-hotpath.md`, `docs/audit/32-settings-docs-deadcode.md` (both written 04:38-04:42 against `HEAD 85ba39b`, build 1277).
**State after this pass**: build 1285 (`0.9.0.1285`), 19 commits on top of `85ba39b`, tree clean apart from the untracked audit folder and `docs/VERIFY-NOW.md`.
**Method**: every row was opened in the code at the cited lines before anything was touched. Verdicts are `real`, `partial`, `false`, `by design` or `not verifiable`; only rows that are real, local and behaviour-neutral were fixed.

**Totals**: 40 rows (31: 15; 32: 25). **No row in either report is a blocker.** 40 read, 1 false, 2 by design, 37 real (three of those probable because the mechanism needs a runtime trace). 19 rows acted on — 18 fixed, 1 partial — in 19 commits; 21 left as proposals.

**One correction to report 31's header**: the live install does exist at `D:/SteamLibrary/steamapps/common/Grand Theft Auto V/Scripts/AutosportRacingSystem/ARS.dll`. It was byte-identical to `bin/Release/ARS.dll` (sha256 `b56d79b6…`) at 1277 and matches after every build below; the report's four-level walk missed it.

## Report 31 — steering / lane / pedal hot path

| Row | Verdict | Action |
|---|---|---|
| R1 blend finishes early under braking | real (arithmetic; entry speed not recorded) | **Not applied.** The clock is time, the geometry is distance; fixing it changes the line law and is a feel decision. |
| R2 Gs floor writes the pedal outside the slew | real | **Not applied.** Two-line move that changes an experimental feature's behaviour; ships off. |
| R3 Open Up copy names the wrong quantity | real | **Fixed**, build 1284 / `b1cb2b2` (with 32-F11). |
| R4 air and floor flags recorded, never read | real | **Not applied.** The choice is printing them (a notice change) or deleting the sampler; that is a decision. |
| R5 Brake Learning toggle parks the verdict queue | **false** | The first statement of `UpdateBrakeLearning` clears `_brakeCommitApexNode` when `BrakeLearning` is off, so nothing can be parked. |
| F1 two guards after the schedule | real | **Fixed**, build 1279 / `446c90b`. |
| F2 decel clock only feeds the debug line | real | **Partial**, build 1282 / `a33204f`: `_lineAlongPrev` is reset in `Launch`. The dead clock stays because removing it changes the debug readout. |
| F3 ordering comment on `_gsFloorActiveThisTick` | real | **Not applied**: the comment sits in the block R4 would delete; re-commenting dead code is noise. |
| F4 four stale comments | real | **Fixed**, build 1280 / `8f7ff02` (follow-up `3bf390e`, build 1285). |
| F5 TECHNOTES brake-learner laws | real | **Fixed**, `cd77f25` (also 32-F1). |
| F6 movers NaN guard downstream of the clamp | real position; NaN source unproven | **Not applied.** Adds a guard, changes a path with no proven input. |
| F7 `apexCommitted` has no lower bound | real, not reachable today | **Not applied.** Needs a symmetric test, not a one-line edit. |
| F8 entrance mark re-captured on a flicker | probable | **Not applied.** Needs a speed trace before deciding the hysteresis. |
| F9 fallback discards the per-driver and per-corner draws | real | **Not applied.** Changes every car's braking point when learning is off — driver's call. |
| F10 `BoundingBox` is both sweep width and lane bound | real | **Not applied.** Separating the two is a design change to the line. |

## Report 32 — settings, menus, docs, dead code

| Row | Verdict | Action |
|---|---|---|
| R1 `CrestEffect` migration cannot run | real | **Not applied.** Migration semantics; needs a decision on what a stored `0` means. |
| R2 stored `AiNitro = Never` pruned unread | real | **Not applied.** Same migration family; the legacy file is deleted in the same load. |
| R3 repair reaches the file and menu, not the session | real | **Not applied.** The fix is a write-back block for twelve statics, not a one-liner. |
| R4 Open Up row seeds from the static | real | **Not applied.** Fixing the reader alone leaves the session on the unrepaired value; it belongs with R3. |
| R5 Steer Trace opens off | real | **Fixed**, build 1278 / `0cd3fee`. |
| R6 `ReverseRoute` stored twice | real, deliberate | **No change**: the two-homes shape keeps the seeding getter from fighting the repair; already documented. |
| F1 AGENTS-STEERING brake learner | real | **Fixed**, `488d1d3`. |
| F2 ABS glide | real | **Fixed**, `4daaaa5` (also closes the OPEN-ITEMS filing and the greyed-switch sentence). |
| F3 Damper Countersteer default 4 | real | **Fixed**, `3644783`. |
| F4 Steer Limit At Speed / 1.25 top | real | **Fixed**, `19b0083`. |
| F5 `OverspeedEnabled` / staged-row restore | real | **Fixed**, `2e4d858`. |
| F6 Development inventory | real | **Fixed**, `f9f3662`. |
| F7 Race Rules omits AI Nitrous Power | real | **Fixed**, `6e9df8d`. |
| F8 WIP `MaxCrestLift` default 1.0 | real | **Fixed**, `70dc6bb`. |
| F9 ABS analogy in the Staged Spawns comment | real | **Fixed**, `dcddb6e`. |
| F10 `BrakeMaxFactor` comment says 1.0 seed | real | **Fixed**, `b1a9581`. |
| F11 Open Up menu vs code, dead symbol | real | **Fixed**, build 1284 / `b1cb2b2` (method renamed `OpensUpClampScale`). |
| F12 nine dead fields | real (1 occurrence each) | **Not applied.** Deletion batch, one decision. |
| F13 surface-material subsystem dead | real | **Not applied.** Same deletion batch. |
| F14 thirteen uncalled methods | real (all 13 confirmed 1 occurrence) | **Not applied.** Same deletion batch. |
| F15 throttle-kick card off for ever | real | **Not applied.** The build does emit CS0162 for it (`Racer.cs:5521`); removing it is a decision. |
| F16 speed-derived steer ceiling parked | real, stated decision | **No change**: documented at the declaration. |
| F17 Staged Spawns handler unreachable | real | **Not applied.** Goes with F5's decision on the row. |
| F18 FEATURES rubberbanding off by default | real | **Fixed**, `680e16c`. |
| F19 PACKAGE-README Settings -> Hotkeys | real | **Fixed**, `9c97b17`. |

## What was not verifiable from this repo

- Every runtime claim: R1's real entry speed, F6's NaN input, F8's flicker, F9's effect on a field, and the repaired-value symptoms in 32-R3/R5. Each has a repro in its report; none was driven.
- Whether the 2.5x skid multiple and the 0.25 brake share fire at the right moment on track.
- The dead-code lists were confirmed by symbol sweep only; nothing here exercises the build's unreachable-code paths beyond the CS0162 warnings it already prints.
