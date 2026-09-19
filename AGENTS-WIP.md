# ARS — First WIP release gate (companion to AGENTS.md)

**Read this when**: the request touches the **first WIP, release, release blocker, test plan, packaging, clean install, artifact, release notes, or pre-release verification**.

> This is the human acceptance and packaging gate for the first public WIP. Code and documentation can show that a path exists; only a clean build plus the appropriate in-game check closes a gameplay item. Docs drift; the code wins.

## WIP scope

The WIP ships the playable race loop, bundled `Tracks\`, the flat `Vehicles\cars.txt` roster, ARS menu settings, Smart Tuning, optional Menyoo appearance application, and LemonUI.SHVDN2.

It deliberately does not ship a track creator or any track-file mutation path. The retired per-vehicle XML, saved-driver, personality, and discipline systems are also outside this WIP.

## Race-loop gate

- [ ] From the menu, start, finish, abandon, and restart a race without a crash or orphaned vehicles/blips.
- [ ] Start both a circuit and a point-to-point track from the supplied `Tracks\` folder.
- [ ] Start at a world join chevron with Context, and open its menu with Sprint + Context.
- [ ] Confirm the selected track, laps, grid size, PI mode, and route direction are the settings the race actually uses.
- [ ] Join the grid in the current player vehicle without it being replaced or teleported unexpectedly.
- [ ] Run a full field through the race without start-line pile-ups or persistent mid-race contact.

## AI-driving gate

- [ ] Test low-, medium-, and high-speed tracks: cars remain on track and brake for tight corners without crawling through sweepers.
- [ ] Test a genuinely low-grip car at speed. Watch specifically for excessive understeer or refusal to turn in.
- [ ] Test hills and crests: no sudden inappropriate braking, zeroed speed target, repeated spins, or lock-ups.
- [ ] Test off-track, inverted, and blocked-car cases: stuck recovery returns the car to racing without looping forever.
- [ ] Test side-by-side traffic and a slower rival: avoidance and overtaking leave physical space and do not drive through rivals or walls.
- [ ] Observe DiveBomb, DefendLane, Yield, and AI nitrous where their conditions arise; none may remain active or corrupt driving after their situation ends.

## Settings and diagnostics gate

- [ ] Verify Race, General Settings, AI Settings, Advanced Settings, and Debug menu changes persist across a script reload.
- [ ] Verify a fresh settings folder creates usable defaults and a second load does not rewrite it.
- [ ] Verify a legacy install migrates old per-menu files without losing user-facing values.
- [ ] Check Show Inputs, Projection, Input Trail, Corner Checkpoints, Edge Chevrons, and Leaderboard against the closest AI car or race state they describe.
- [ ] Confirm `Log.log` records the session banner and actionable load failures. With **Log Level `None`** (the shipped default) that is *all* it records: the banner, plus forced error lines — one of which is the bridge probe's, which is why the release README's "the log names the reason" still holds.
- [ ] Confirm `Options.ini` advanced catch-up and reverse-route settings remain readable after the menu-settings migration.

## Package gate

- [ ] Build Release successfully and confirm `ARS.dll` and `LemonUI.SHVDN2.dll` deploy to the game Scripts folder.
- [ ] Run the generated GitHub artifact from a clean install using a current, matched SHVDN asi/API-dll pair — the verified pair is nightly **`v3.7.0-nightly.188`**: asi `239104` B + `ScriptHookVDotNet2.dll` `984576` B, both from that one zip (`AGENTS-SHVDN.md`).
- [ ] Confirm the package includes the LemonUI credit — **JustALemon**, its nickname, stating the dll is used under the MIT License — plus tracks, `cars.txt`, `sillynames.txt`, and an **empty `Settings\`** — no settings ship at all; the mod writes its three inis on first run.
  - **No `LemonUI-LICENSE.txt` and no verbatim MIT notice ship. That is a deliberate, informed decision, not an oversight — do not "fix" it.** It sits outside the MIT condition that the notice accompany copies, and the copyright holder's real name is present in the dll's own assembly metadata in any case, so the file bought no privacy.
- [ ] Confirm it excludes `ScriptHookVDotNet2.dll`, logs, crash artifacts, stale legacy settings files, and untracked `Options.ini`.
- [ ] Read the staged release `README.txt` for installation, matched-SHVDN, menu, and log-troubleshooting instructions.

## Limitations to disclose

- No in-game track creation, editing, update, or deletion.
- No per-car XML saves, saved drivers, personalities, or discipline filtering.
- The grid uses model-level pace, not the installed upgrades of a particular vehicle instance.
- Low-grip steering behavior still needs the dedicated release-gate drive above.
- The beater/rust matte paint rule is implemented and shipped **unverified in game** — drive a `rusty`/`rat look`/`junkyard`/`primed` livery before trusting it.
