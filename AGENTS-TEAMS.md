# ARS — Agent Teams: which route, and the discipline a teammate keeps

Companion to `AGENTS.md`. The routing rule, the pinned pools and the dispatch rules are in the global `~\.dsh\AGENTS.md`; this file is what those instructions mean **on this repo**.

## Which route
- **`subagent` — one deliverable, no discussion.** Fetch a slice, extract an inventory, audit one file against a doc, answer one question whose answer is in the tree. Cheap, parallel, always available — no permission gate. One subagent, one deliverable; the extraction prompt shape is in the global file.
- **`spawn_teammate` — discussing and planning a system.** The brief is a design conversation that runs propose → argue → settle → apply across many turns, or two writers on files that must not touch. Durable, addressable, shares this filesystem. **The user must ask for it** — never spawn one because the work merely feels big, and never for a fetch a `subagent` could hand back in one turn.

**Evidence it earns the round trip**: the Agent Teams variant once caught a sign error that would have *raised* braking decel on a crest, an invented unit conversion and a point-to-point guard bug, and overturned two of the Lead's own "corrections". Two agents with file access beat one — provided the Lead arbitrates.

## The task board
A teammate's work starts as a shared task, never as a prompt alone.
1. **Create the task before the work** with its subject, its acceptance criteria and its **write scopes** — the file or directory prefixes it may modify.
2. **Get, then claim** with the current revision. A claim against a stale revision is rejected rather than merged.
3. **Work, then complete.** `complete` means the edits are on disk, not that they are right.
4. **Readiness never wakes anybody.** A task that becomes unblocked while its owner is inactive stays pending until `send_message` starts that owner.

## Writing
- **One writer per file.** Disjoint scopes are the only safe shape; a write-scope overlap warning is advisory, not a lock.
- The Lead owns the merge and the final acceptance test, and arbitrates a named disagreement instead of silently picking a side — a teammate's assertion is a claim to check, not an authority.
- A teammate may **compile** but may never call a change **verified**: the human drives, and a successful build is not evidence.
- **Every commit goes to a teammate for review** — superseded as a *cadence* by "Audit cadence" below, which keeps the review-task mechanics but reserves the teammate pass for the circling signal, because the driver reloads about twice per five minutes and a reload is therefore not a review boundary. When a review does run: put the range on the shared board as a review task and send it to the teammate with the hashes, what each commit claims, and the acceptance criteria — and a finding needs `file:line`, evidence, and a note of what was verified against the code rather than inferred.

## Audit cadence — the standing procedure

Two tiers, because the failure classes are different. **Mechanical invariants get a checker, not an audit** — auditing is for judgement, and spending a teammate on a symmetry a command can verify is how a frequent audit turns into a rubber stamp.

- **Tier A — every batch, the Lead, mechanical, before the batch is driven.** The symmetry sweep: every key the menu writes is declared in `SettingsRepair`'s schema (the governor key was written, undeclared, pruned on every load, and silently defaulted); every field written has a reader; every constant has a consumer; every method has a caller; every commit body's claim is supported by its own diff. No teammate and no judgement — it is greps and a diff read, and it is what would have caught the governor key the moment it was added. **Standing todo:** turn the first of these into a load-time self-check that logs a warning, so the class becomes a log line instead of a habit.
- **Tier B — on the circling signal, a teammate.** Call one when the work shows any of: **a long run of commits landing without a push** (the objective form of everything below, since a push only lands on a resolved batch); the same law or constant being re-tuned for a third time; a change reverting something added within the last few commits; a fix whose diagnosis supersedes the previous fix's own diagnosis; the driver reporting the same symptom twice in different words; batches shrinking toward one-line surgical edits with no end in view; or a drive requested to **distinguish** two hypotheses rather than confirm one. Scope it to the batch's blast radius with the Tier A checklist — a full-system inventory is for when the structure changed, not for every pass.
- **Backstop** — if no signal fires, audit every six driver reloads anyway, so nothing goes unaudited forever. Six is a floor, not the trigger.

## Talking
- `send_message` is durable — a running teammate takes it at its next step boundary, an inactive one is started or resumed by it.
- `interrupt_agent` stops a turn and keeps the pending inbox; never reach for process termination, because a teammate runs on this same harness.
- `wait_agent` observes only changes that happen *after* the call and returns `noProgress` when nobody else is running; re-list agents and tasks after every wakeup instead of assuming.
- **Only a teammate's closing message reaches the Lead** — its intermediate steps do not — so a design touching several questions must be restated whole in the final message, or the Lead reopens the task and asks for it; an answer split across the work arrives as one section of itself.

## Non-negotiable on this repo
- **A teammate never commits.** The Lead stages explicit paths and writes the commit body, which carries no attribution of any kind; `Dist\` mirrors the live install, so a blanket add sweeps session data into an unrelated commit.
- **Code is edited with the direct file tools, never a script**, and `AGENTS.md`'s Coding style binds a teammate exactly as it binds the Lead — the C# 7.3 ceiling and the comment policy included.
- **Anything touching the build → reload → drive cycle stops at the user**: a teammate can change the DLL, only the driver can say it works.
