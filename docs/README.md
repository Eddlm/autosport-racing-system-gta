# ARS documentation

Five live documents, one folder of reference material and two of records. Each file answers **one question**;
if a file has to answer two, the depth moves to a companion and the entry keeps the pointer.

**The rule this set is built on: a document defines concepts and the relationships between them, and leaves the
numbers to the source.** A constant copied into prose is wrong within a session — this repository has carried a stale
pedal scale, a stale lift rate and stale line pointers in the same week, each of them quoted from a doc that looked
authoritative. Values live in the code, where a rename or a retune cannot leave them behind.

| File | The question it answers | Who reads it |
|---|---|---|
| `DESIGN.md` | What is each system *for*, and what shape serves that goal — including the shapes that were tried and rejected, and why | anyone changing behaviour |
| `steering-and-pedals.md` | How the front-slip guard and the pedal limits are built today: the four pieces, the actuator rates, the evidence and what is still open | anyone touching the steering limiter or a pedal law |
| `FEATURES.md` | What does the player get, and what does it feel like | players; the store page and the README |
| `RELEASE-NOTES.md` | What ships in this release, what it needs, and what it does not do yet | players installing it |
| `REGISTER.md` | What is still open: decided and not built, waiting on a decision, understood and not fixed | whoever picks the next piece of work |

Then, by folder:

- **`reference/`** — material that is *not* about this project and must not be edited to fit it: the source-cited
  survey of how racing-game and sim-racing AI build a lateral controller. Read it when you need to know what the field
  does before deciding what ARS should do.
- **`working/`** — session papers, not documentation: the drives still owed (`TEST-PLAN.md`), in-flight plans, audit
  notes, and the archives of files this set absorbed. Nothing here is a statement about how the system works; it is a
  record of what a session was doing.
- **`audit/`** — frozen records of past audits, one file per theme, plus the verifier passes that adjudicated them.
  They describe a tree that no longer exists. Never edit one to match today's code; write a new finding instead.

**Where to look for a given thing**

- *"Why does the car do this?"* → `DESIGN.md`.
- *"What is this setting, and what does it trade?"* → `DESIGN.md`, then the row in the menu, then the constant.
- *"Is this already known?"* → `REGISTER.md` for open work, `working/TEST-PLAN.md` for an owed drive.
- *"What does this do?"* → `FEATURES.md` if the answer is for a player, `DESIGN.md` if it is for a reader of the code.
- *"How does anything else do it?"* → `reference/`.

**The agent-facing notes are not here.** `AGENTS/` holds the working logs, the ranked backlog, the flag references and
the session handoffs — material for the next session, deliberately kept out of the shipped folder.
