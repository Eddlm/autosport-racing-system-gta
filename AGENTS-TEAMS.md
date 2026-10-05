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

## Talking
- `send_message` is durable — a running teammate takes it at its next step boundary, an inactive one is started or resumed by it.
- `interrupt_agent` stops a turn and keeps the pending inbox; never reach for process termination, because a teammate runs on this same harness.
- `wait_agent` observes only changes that happen *after* the call and returns `noProgress` when nobody else is running; re-list agents and tasks after every wakeup instead of assuming.
- **Only a teammate's closing message reaches the Lead** — its intermediate steps do not — so a design touching several questions must be restated whole in the final message, or the Lead reopens the task and asks for it; an answer split across the work arrives as one section of itself.

## Non-negotiable on this repo
- **A teammate never commits.** The Lead stages explicit paths and writes the commit body, which carries no attribution of any kind; `Dist\` mirrors the live install, so a blanket add sweeps session data into an unrelated commit.
- **Code is edited with the direct file tools, never a script**, and `AGENTS.md`'s Coding style binds a teammate exactly as it binds the Lead — the C# 7.3 ceiling and the comment policy included.
- **Anything touching the build → reload → drive cycle stops at the user**: a teammate can change the DLL, only the driver can say it works.
