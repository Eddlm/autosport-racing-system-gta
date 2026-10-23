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
- [ ] `Log.log` records the session banner and actionable load failures. With **Log Level `Error`** (the shipped default) that is the banner, every ARS error and fatal line, and the forced diagnostics — one of which is the bridge probe's, which is why the release README's "the log names the reason" still holds. Info is opt-in.
- [ ] Every retired settings file is removed on load (`Settings.ini`, `Options.ini`, `DevSettings.ini`, `DevConfig.ini`, `MemoryOffsets.ini`, `Menu-Racers.ini`, `Menu-DevSettings.ini`). **`Options.ini` has no reader at all** — nothing left in it to verify.

## Versioning and releases — the convention

**SemVer as written, in the `0.y.z` reading the spec itself gives** (user-accepted): the minor is the release dial while major is zero — the FAQ says to increment it for each initial-development release — and `1.0.0` is declared when the formats and the menu are settled, never counted towards. The minor digit is unbounded: §2's own example runs `1.9.0 -> 1.10.0 -> 1.11.0`, so no `0.9 -> 1.0` step is waiting to be tripped, and the ecosystem treats a `0.y` minor as breaking-capable (npm's caret widens only from 1.0) — which is what "anything MAY change" means in practice.

- **The syntax still holds**: three numeric fields, no leading zeroes, compared numerically — so `0.8.10` correctly follows `0.8.9` (`semver.org` §2 and §11), and the mod never compares versions anyway: `UpdateChecker` only prints the latest release title, falling back to its tag.
- **The release identity is hand-moved.** `src\Properties\AssemblyInfo.cs`'s `AssemblyVersion`/`AssemblyFileVersion` are bumped at the cut so they match the tag; nothing derives either from the other, the zip is named by short SHA (`deploy.yml:65`), and the build number beside the dll is the "which binary" axis.
- **Only a `v*` tag attaches a release** (`deploy.yml:7,76,80`), which is why the older `0.8` and `0.8.5` — no `v` — were uploaded by hand. Keep the prefix consistent, and fix the dead changelog link in `v0.8.6`'s body, which compares against `v0.8.5DONTRY`, a tag that does not exist on the remote.
- **A CI release starts as a prerelease on purpose** — a guard, flipped off by hand once the build is blessed. While it is on, GitHub's `releases/latest` **skips it**, so the in-game update notice stays quiet: the flip is what announces the release, and forgetting it is how a release goes unannounced.
- **The double-digit minor is settled — stay on SemVer; a mid-stream switch costs more than it buys.** The fraction no longer reaches a player at all: the release title is the build number (`deploy.yml:79`) and `UpdateChecker` prefers the title over the tag, so the number they compare is monotonic while the tag keeps the semantic version. CalVer's surviving advantage here is the one thing a SemVer string cannot say — *when* this was built — and GitHub already dates every release, so switching would buy that alone at the price of the fix-versus-feature signal, which `calver.org`'s own FAQ gives up deliberately.

## Package gate

- [ ] Build Release, and confirm `ARS.dll` and `LemonUI.SHVDN2.dll` deploy to the game Scripts folder.
- [ ] Run the generated GitHub artifact from a clean install on a current, matched SHVDN asi/API-dll pair — the verified pair is nightly **`v3.7.0-nightly.188`**: asi `239104` B + `ScriptHookVDotNet2.dll` `984576` B, both from that one zip (`AGENTS-SHVDN.md`).
- [ ] Confirm the package includes the LemonUI credit — **JustALemon**, its nickname, stating the dll is used under the MIT License — plus tracks, `cars.txt`, `sillynames.txt` and an **empty `Settings\`**: no settings ship at all, the mod writes its four inis on first run (Race, Settings, Debug, Development).
  - **No `LemonUI-LICENSE.txt` and no verbatim MIT notice ship. That is deliberate and informed, not an oversight — do not "fix" it.** It sits outside the MIT condition that the notice accompany copies, and the copyright holder's real name is in the dll's own assembly metadata in any case, so the file bought no privacy.
- [ ] Confirm it excludes `ScriptHookVDotNet2.dll`, logs, crash artifacts and stale legacy settings files.
- [ ] Read the staged release `README.txt` for installation, matched-SHVDN, menu and log-troubleshooting instructions.

## Limitations to disclose

- In-game creation is live (the creator's Save Track); no track update, editing or deletion — the `UpdateRoute` path was cut (`c2ad2de`).
- No per-car XML saves, saved drivers, personalities or discipline filtering.
- The grid uses model-level pace, not the installed upgrades of a particular vehicle instance.
- Low-grip steering behavior still needs the dedicated release-gate drive above.
- The beater/rust matte paint rule is implemented and ships **unverified in game** — drive a `rusty`/`rat look`/`junkyard`/`primed` livery before trusting it.

## Release sequence — 0.9.0 (prep state)

`AssemblyInfo.cs` carries `0.9.0.0` ahead of the cut, so the in-game banner announces the version the tag claims; the two are still hand-moved and nothing derives either from the other. The notes are hand-written in `docs/RELEASE-NOTES.md`, because the auto-generated body would span every commit since `v0.8.6` and read as noise.

1. Land and push the release commit; CI builds from a fresh checkout, so a local build counter ahead of it does not matter.
2. Tag `v0.9.0` and push the tag — `deploy.yml` attaches a prerelease with an auto-generated body.
3. Replace that body: `gh release edit v0.9.0 --notes-file docs/RELEASE-NOTES.md`.
4. **Clean install before flipping**: download the attached zip, extract into a clean `Scripts\` beside the matched SHVDN pair, and run the race-loop gate above.
5. Flip the prerelease off — that is what moves `releases/latest` and lets the in-game update notice fire.
6. Close what the release answers, one line each: **#8** (a build the mod cannot drive on now logs a forced error instead of crashing) and **#3** (the downforce toggle — the reporter confirmed it works). **#1** and **#7** are old and unrelated: close as stale or leave, the owner's call rather than the release's.

## WIP — projection throttle rule (spec, user-stated 2026-10-23)

`OffshootInputCap` is being simplified. Keep gates 1, 2, 3 exactly as they are:

1. `_laneOverride` must be set (a car holding its own line is exempt).
2. Speed >= `OffshootMinSpeedMph` (20).
3. `Math.Abs(CurrentTrackPoint.Angle) > OffshootMinRouteAngleDeg` (2).

New shape:

- **Throttle only.** The path no longer brakes and no longer caps the combined input; `offtrackBrakeCommand` becomes always false and the `BrakeReason.Offtrack` tag goes with it.
- **Ramp 1 -> 0 from the track edge out to 5 m beyond it** (`OffshootRangeMeters` 2 -> 5; inside the edge the cap is 1.0, so the old centre-line ramp and `OffshootBlendBrake` are gone).
- **Only when the projection lands farther from the centre than the car's own position.** If the car is already wider than where it is heading, nothing fires.

Shipped in build 736 as `OffshootThrottleCap` (`Racer.cs:1607`): the cap is applied with `Math.Min` to `newThrottle` after the throttle composition, so it is a ceiling over every reason. `OffshootBlendBrake`, the brake command and `BrakeReason.Offtrack` are gone from the brake path; the field is `_offtrackThrottleCap` and the pedal bar draws it over the whole 1..0 range. Compile-verified, drive owed.

## WIP — NodesToDoubleRadius, report mode (build 738)

The blur gate holds how many nodes the road runs before `IsOpenedFor`
sees a radius of `2 x` the apex's **smoothed** radius (`SpanDoublingMultiple = 2`, `SpanConsecutiveNodes = 8`),
walked to `exitMaxStep` = min(500, nodes left) and **not** clamped to 40. Each corner logs:

`Corner open: apex=1487 smoothedR=41.3 tightestR=33.9 nodesToDouble=12/500`

**Now a gate (build 739).** `nodesToDouble > SpanBlurNodes (60)` returns before the corner is built, and the
`CornerCeilingMeters` screen is gone. The 60 comes from 32 corners over FIG, LSIA and a third track, whose
distribution splits between 52 and 70 nodes; a 40-node cut was tried and rejected because node count mixes distance
with speed, while seconds mix a road property with the car's grip (1G..3G across mods), so seconds are not a road
unit at all. Expected effect on the three measured tracks: 4 corners culled of 32.

Residual to know: `CornerCeilingMeters` still feeds `regionRadius` in `BuildApexTable`, so a region is never cut
where the road is wider than the ceiling. The screen is retired; the region delimiter is not. A 250 m radius bend
still never forms a region and so never reaches the blur test - worth attacking in the region detector itself.

## Field-verified, 2026-10-23 (builds 732-748)

- **ABS as a lock guard (732)** — driver-verified: the brake is no longer held off by the old slip taper.
- **Blur gate at 60 nodes (739)** — verified by import on Figureight (1 culled), LSIA (2), By The Highway (1),
  and a fourth track (apex 511, R 162.7, 154 nodes). The population is consistent: sweeps of 160-185 m radius
  take 149-165 nodes to open, corners of the same radius take 24-34. Nothing in between was measured.
- **Projection throttle cap (742-745)** — the descending-Remap bug is why it never fired; it now fires on ~0.75%
  of ticks, all of them genuine (the logged samples show the car already past the edge when the cap drops).
- **Off-track recovery throttle (743/748)** — 0.75, not 0.5: half would not pull the car out of the grass from a
  stop. Verified in the field.
- **Development gate (747)** — the row must NOT require `PlayerParticipating`: Racers holds the player only when
  they are on the grid, and the levers exist for a one-AI test with the player spectating.

### Owed: the LSIA 66.3% hard brake

The corner at 66.3% of LSIA was culled by the blur gate and the car now brakes sharply there without leaving the
road. The suspicion is a step change in the route speed: with no corner over that stretch, `followTrackSpd` is the
only law, and it may be reading the raw `PreciseCurveRadius` (a +/-4-node circumradius, noisy by construction)
rather than a smoothed one. Not started.

## Handoff — harmonising the speed governors

Four systems decide how fast a car goes. They were built one at a time and never composed, which is the next job.

1. **Corner** — `ApexBrakingSpeed` builds a ramp from the braking target to the apex and releases at the braking target (build 734, reverting 731). It knows the distance left, so its deceleration is planned. Full brake authority.
2. **Curvature** — `RouteRadiusSampled` (three-node plan-view circumradius at the car, speed/grip and twice that) through `RouteIdealSpeedForRadius` = sqrt(grip * g * R). It is a level, not a transition: no memory, no lookahead, no time. While it is the term setting the speed, the brake is capped at `RouteBrakeCap` 0.1 (build 755, a probe).
3. **Crest and slope** — `CrestGripSpeedFactor` (the no-fly gate, samples 1 s and 2 s ahead, drawn blue and red) and `slopeSpeedFactor`. These multiply **both** the corner and the route speed, so they are the only pair that already composes.
4. **Projection** — `OffshootThrottleCap`: throttle only, 1.0 at the track edge ramping to 0 over the five metres beyond it, and it fires only when the projection is outside the edge **and** wider than the car. Applied with `Math.Min` after `ComposeThrottleCap`, so it outranks every reason field.

**Where they meet.** `Intention.Speed = min(cornerTerm, routeTerm)`, then slope and crest multiply, then the error `Speed - currentSpeed` is divided by 3 m/s for throttle and 6 m/s for brake. Those two numbers are the amplifier: at R = 150 m, 23 m of radius is the whole throttle range and 47 m the whole brake range, so any residual step in a radius reading becomes a saturated pedal. It is a proportional law with a band, and outside that band it is bang-bang.

**Measured, not assumed.** On the real LSIA route a three-point sample steps up to 12.84 km/h of target speed in one node, which is 59% of the brake range in ~26 ms. A least-squares four-point fit halves that inside the governed regime (R < 200 m) but is unusable unguarded: on a uniform node spacing the samples are centrally symmetric, the algebraic fit's curvature terms cancel to ~1e-9, and it returns the RMS size of the sampling window as a "radius" (45 m on a road whose real radius is 5-9 km). The sagitta guard (`RouteStraightSagittaShare` 0.002) fixes it; the fit still under-reads tight corners by ~16%.

**Field-verified.** Trails show red only where a corner brakes, and green-yellow with no red at all through a curve without an apex: the two systems are visibly separate. Radius readout 15 m at an apex, 182 m through a wide junction, 999 m on a viaduct.

**Open defects to harmonise, in order of what they cost:**
- The pedal law is proportional on a speed error with no deceleration term: a car 1 m/s over at 200 m out and one 1 m/s over at 10 m out get the same pedal.
- The curvature term has no time dimension while the corner does. Rate-limiting its fall to a grip-consistent deceleration is the cheap fix; a feed-forward from the required deceleration is the correct one.
- `999` is the sentinel for both "huge radius" and "degenerate solve", so the route can be blind in a curve and say so with a number that looks like a straight. Suspected at a viaduct; needs the estimator checked in a genuinely tight curve.
- The brake learner measures `share` of full pedal. The 0.1 cap makes share 0, so it raises the factor by 18% every pass and teaches the car to brake even less. It should measure relative to the cap.
- `CornerCeilingMeters` survives only as the region delimiter (200 m); the blur gate at 60 nodes decides what exists.
**Probe state.** `RouteBrakeCap` 0.1 while the route term governs, and `steerLimitedSpeedGovernsRoute` false: the steer-limited speed is still computed for readout but no longer replaces 70 % of the curvature term, so the route speed is the track's radius alone. Both are one-line switches.
