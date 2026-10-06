# ARS — First WIP release gate (companion to AGENTS.md)

**Read this when** the request touches the **first WIP, release, release blocker, test plan, packaging, clean install, artifact, release notes, or pre-release verification**.

> The human acceptance and packaging gate for the first public WIP. Code and documentation can show that a path exists; only a clean build plus the matching in-game check closes a gameplay item. Docs drift; the code wins.

## WIP scope

The WIP ships the playable race loop, bundled `Tracks\`, the flat `Vehicles\cars.txt` roster, ARS menu settings, Smart Tuning, optional Menyoo appearance, and LemonUI.SHVDN2.

The track creator is live — a root Track Creator submenu records a route and Save Track writes `Tracks\*.xml` (revived and driver-verified, per `AGENTS-BACKLOG.md`). There is no track-update path: the dead `UpdateRoute` was cut (`c2ad2de`). Retired per-vehicle XML, saved-driver, personality and discipline systems are outside this WIP.

## Race-loop gate

- [ ] From the menu, start, finish, abandon and restart a race without a crash or orphaned vehicles/blips.
- [ ] Start a circuit and a point-to-point track from the supplied `Tracks\` folder.
- [ ] Start at a world join chevron with Context, and open its menu with Sprint + Context.
- [ ] Confirm the selected track, laps, grid size, PI mode and route direction are the settings the race actually uses.
- [ ] Join the grid in the current player vehicle without it being replaced or teleported unexpectedly.
- [ ] Run a full field through the race without start-line pile-ups or persistent mid-race contact.

## AI-driving gate

- [ ] Low-, medium- and high-speed tracks: cars remain on track and brake for tight corners without crawling through sweepers.
- [ ] A genuinely low-grip car at speed — watch specifically for excessive understeer or refusal to turn in.
- [ ] Hills and crests: no sudden inappropriate braking, zeroed speed target, repeated spins or lock-ups.
- [ ] Off-track, inverted and blocked-car cases: stuck recovery returns the car to racing without looping forever.
- [ ] Side-by-side traffic and a slower rival: avoidance and overtaking leave physical space and do not drive through rivals or walls.
- [ ] DiveBomb, DefendLane, Yield and AI nitrous where their conditions arise; none may remain active or corrupt driving after its situation ends.

## Settings and diagnostics gate

- [ ] Race, General Settings, AI Settings, Advanced Settings and Debug menu changes persist across a script reload.
- [ ] A fresh settings folder creates usable defaults, and a second load does not rewrite it.
- [ ] A legacy install migrates old per-menu files without losing user-facing values.
- [ ] Show Inputs, Track Analysis, Corner Checkpoints, Edge Chevrons and Leaderboard match the closest AI car or race state they describe.
- [ ] `Log.log` records the session banner and actionable load failures. With **Log Level `None`** (the shipped default) that is *all* it records: the banner plus forced error lines — one of which is the bridge probe's, which is why the release README's "the log names the reason" still holds.
- [ ] Every retired settings file is removed on load (`Settings.ini`, `Options.ini`, `DevSettings.ini`, `DevConfig.ini`, `MemoryOffsets.ini`, `Menu-Racers.ini`, `Menu-DevSettings.ini`). **`Options.ini` has no reader at all** — nothing left in it to verify.

## Package gate

- [ ] Build Release, and confirm `ARS.dll` and `LemonUI.SHVDN2.dll` deploy to the game Scripts folder.
- [ ] Run the generated GitHub artifact from a clean install on a current, matched SHVDN asi/API-dll pair — the verified pair is nightly **`v3.7.0-nightly.188`**: asi `239104` B + `ScriptHookVDotNet2.dll` `984576` B, both from that one zip (`AGENTS-SHVDN.md`).
- [ ] Confirm the package includes the LemonUI credit — **JustALemon**, its nickname, stating the dll is used under the MIT License — plus tracks, `cars.txt`, `sillynames.txt` and an **empty `Settings\`**: no settings ship at all, the mod writes its three inis on first run.
  - **No `LemonUI-LICENSE.txt` and no verbatim MIT notice ship. That is deliberate and informed, not an oversight — do not "fix" it.** It sits outside the MIT condition that the notice accompany copies, and the copyright holder's real name is in the dll's own assembly metadata in any case, so the file bought no privacy.
- [ ] Confirm it excludes `ScriptHookVDotNet2.dll`, logs, crash artifacts and stale legacy settings files.
- [ ] Read the staged release `README.txt` for installation, matched-SHVDN, menu and log-troubleshooting instructions.

## Limitations to disclose

- In-game creation is live (the creator's Save Track); no track update, editing or deletion — the `UpdateRoute` path was cut (`c2ad2de`).
- No per-car XML saves, saved drivers, personalities or discipline filtering.
- The grid uses model-level pace, not the installed upgrades of a particular vehicle instance.
- Low-grip steering behavior still needs the dedicated release-gate drive above.
- The beater/rust matte paint rule is implemented and ships **unverified in game** — drive a `rusty`/`rat look`/`junkyard`/`primed` livery before trusting it.
