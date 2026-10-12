# ARS — First WIP release gate (companion to AGENTS.md)

**Read this when** the request touches the **first WIP, release, release blocker, test plan, packaging, clean install, artifact, release notes, or pre-release verification**.

> The human acceptance and packaging gate for the first public WIP. Code and documentation can show that a path exists; only a clean build plus the matching in-game check closes a gameplay item. Docs drift; the code wins.

## WIP scope

The WIP ships the playable race loop, bundled `Tracks\`, the flat `Vehicles\cars.txt` roster, ARS menu settings, Auto Styling, optional Menyoo appearance, and LemonUI.SHVDN2. **Bikes are out of scope**: every bike line is dropped by the loader's `IsCar` gate and the Motorcycles/Cycles/OffRoad blacklist, so no bikes pool ships and the work is parked post-0.9.0 (`raceable-bikes` in `AGENTS-BACKLOG.md`).

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

- [ ] Race, Rivals (and its three behaviour submenus), Race Rules, Interface, Controls and Debug menu changes persist across a script reload.
- [ ] A fresh settings folder creates usable defaults, and a second load does not rewrite it.
- [ ] A legacy install migrates old per-menu files without losing user-facing values.
- [ ] Show Pedal Trail, Track Analysis, Corner Checkpoints, Edge Chevrons and Leaderboard match the closest AI car or race state they describe.
- [ ] `Log.log` records the session banner and actionable load failures. With **Log Level `None`** (the shipped default) that is *all* it records: the banner plus forced error lines — one of which is the bridge probe's, which is why the release README's "the log names the reason" still holds.
- [ ] Every retired settings file is removed on load (`Settings.ini`, `Options.ini`, `DevSettings.ini`, `DevConfig.ini`, `MemoryOffsets.ini`, `Menu-Racers.ini`, `Menu-DevSettings.ini`). **`Options.ini` has no reader at all** — nothing left in it to verify.

## Versioning and releases — the convention

**SemVer as written, in the `0.y.z` reading the spec itself gives** (user-accepted): the minor is the release dial while major is zero — the FAQ says to increment it for each initial-development release — and `1.0.0` is declared when the formats and the menu are settled, never counted towards. The minor digit is unbounded: §2's own example runs `1.9.0 -> 1.10.0 -> 1.11.0`, so no `0.9 -> 1.0` step is waiting to be tripped, and the ecosystem treats a `0.y` minor as breaking-capable (npm's caret widens only from 1.0) — which is what "anything MAY change" means in practice.

- **The syntax still holds**: three numeric fields, no leading zeroes, compared numerically — so `0.8.10` correctly follows `0.8.9` (`semver.org` §2 and §11), and the mod never compares versions anyway: `UpdateChecker` only prints the latest release title, falling back to its tag.
- **The release identity is hand-moved.** `src\Properties\AssemblyInfo.cs`'s `AssemblyVersion`/`AssemblyFileVersion` are bumped at the cut so they match the tag; nothing derives either from the other, the zip is named by short SHA (`deploy.yml:65`), and the build number beside the dll is the "which binary" axis.
- **Only a `v*` tag attaches a release** (`deploy.yml:7,76,80`), which is why the older `0.8` and `0.8.5` — no `v` — were uploaded by hand. Keep the prefix consistent, and fix the dead changelog link in `v0.8.6`'s body, which compares against `v0.8.5DONTRY`, a tag that does not exist on the remote.
- **A CI release starts as a prerelease on purpose** — a guard, flipped off by hand once the build is blessed. While it is on, GitHub's `releases/latest` **skips it**, so the in-game update notice stays quiet: the flip is what announces the release, and forgetting it is how a release goes unannounced.
- **The double-digit minor is settled — stay on SemVer; a mid-stream switch costs more than it buys.** The fraction no longer reaches a player at all: the release title is the build number (`deploy.yml:78`) and `UpdateChecker` prefers the title over the tag, so the number they compare is monotonic while the tag keeps the semantic version. CalVer's surviving advantage here is the one thing a SemVer string cannot say — *when* this was built — and GitHub already dates every release, so switching would buy that alone at the price of the fix-versus-feature signal, which `calver.org`'s own FAQ gives up deliberately.

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
