# audit — frozen records of past audits

Every file here describes a tree that no longer exists. **Never edit one to match today's code** — a report is evidence
of what was true on the day it was written, and a "fix" to a report destroys the only record that the defect was once
found. A finding that is still true gets a new report, or a row in `docs/REGISTER.md`.

The method that produced these is `skill: audit-swarm`: hunt cards written open-ended, one per attack surface, each
finding citing `file:line`, a log line or a commit hash, graded VERIFIED or PROBABLE. A worker's blocker is a claim
until somebody re-opens the cited lines.

**Series**, in the order they were run:

| Series | What it is |
|---|---|
| `01`–`13` | the first 0.9.0 hunt, one card per surface: build/toolchain, race cycle, speed governors, lane avoidance, physics/tyres, menu settings, track data, runtime robustness, performance, release readiness, docs vs code, data integrity, crash environment |
| `200`–`212` | the verifier passes: `200` adjudicates the release, `201`/`202` re-check the two halves of the hunt, `203`–`212` take one surface each and give every inherited row its own verdict |
| `31`–`33` | an earlier pair — steering hot path, settings/docs/dead code — and the verdict that closed them |
| `301`–`305` | the adjudication of the next batch, plus appearance/vehicles, grid utilities, the grid-fill failure and a verify pass over the fixes |
| `410`–`417` | the T-series hunt, one card per surface: startup/teardown, settings, cards and timers, state lifetime, lane geometry, pedals and grip, menus, docs vs code |
| `500`, `501` | the session audits of the lane/corner range (`fc8dd13..HEAD`), one per model, run independently and adjudicated in `docs/working/VERIFY-NOW.md` §L |

**Reading one**: the header names the model, the date, the range and the HEAD it saw. The table's `evidence` column is
the only part worth trusting on its own; the verdicts were the auditor's, and the ones that mattered were re-checked.
