# working — session papers, not documentation

Nothing in this folder is a statement about how the system works today. Each file is the record of what a session
was doing: a plan, an audit, a drive still owed, a state dump written at the context guard.

**The rules here**

- **Frozen.** A paper is not edited to match later code. If today's tree contradicts it, write a new paper or a line in
  the paper's own head saying it is superseded — never rewrite the record until it agrees.
- **Numbers are allowed** — a paper quotes the code and the logs it read, which makes it evidence rather than
  documentation. `AGENTS.md`'s no-numbers rule does not reach into this folder.
- **The live answer is elsewhere**: open work is `docs/REGISTER.md`, the drives owed are `TEST-PLAN.md`, and how the
  systems work is `docs/DESIGN.md`.

| File | What it records | Status |
|---|---|---|
| `VERIFY-NOW.md` | the in-game verification ledger: what is deployed, what is compile-only, what the driver has signed off | **live — updated every session** |
| `TEST-PLAN.md` | the drives still owed, one section per feature, with what to do and what would count as passing | live |
| `OPEN-ITEMS-LOG.md` | a dated section per run of a 16:00 collection cron that no longer exists | frozen |
| `KNOWN-BUGS-ARCHIVE.md` | defects understood well enough to describe and not worth fixing yet | frozen |
| `AUDIT.md` | the clarity / method-size / CPU / comment audit of the 22-file `src/` tree | record + status table |
| `AUDIT-COMPUTE-ONCE.md` | every place the publish-once rule is still broken | analysis only |
| `LOG-AUDIT.md` | one session's `LogImportance.Info` inventory, from a 30,980-message log | record |
| `PLAN-RATE-PRECISION.md` | the rate and precision stages, the deferred perception manager, the route-frame migration | plan |
| `PLAN-STEERING-SIMPLIFICATION.md` | the contract and the rulings for the steering simplification | plan |
| `PLAN-NOS-PR11.md` | N.O.S. compatibility and the PR #11 adoption triage | plan |
| `STEERING-SIMPLIFICATION-FINDINGS.md` | the findings and ranked candidates behind that plan — superseded where it touches the lane | findings |
| `RENAME-LANES.md` | the lane vocabulary map, annotated in place before any rename | record |
| `STATE-2026-11-02.md` | a context-guard handoff: the tree at build 1602 and the lane law as it stood | frozen |
| `RELEASE-0.9.0-DRAFT-ARCHIVE.md` | the curated release body for the `v0.9.0` tag | archive |
| `chicane-inflection-experiment.patch` | an experiment kept as a patch, never applied | archive |

Reports written by an audit belong in `docs/audit/`, not here.
