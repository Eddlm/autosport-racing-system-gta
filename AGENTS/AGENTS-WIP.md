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
- [ ] Run the generated GitHub artifact from a clean install on a current, matched SHVDN asi/API-dll pair — the verified pair is **SHVDNE 3.9.0.6** (asi + `ScriptHookVDotNet2.dll` 2.11.6).
- [ ] Confirm the package includes the LemonUI credit — **JustALemon**, its nickname, stating the dll is used under the MIT License — plus tracks,
  `cars.txt`, `Drivers\\names.txt` and `Settings\\tips.txt`. **No ini ships** (owner, 2026-10-26): `SettingsRepair` writes every missing key on
  first load from the same schema that validates them, so the defaults live in one place - the code - and cannot drift from a generated file.
  `Settings\\` ships with `tips.txt` and `.gitkeep` only, and `Dist/AutosportRacingSystem/Settings/*.ini` is gitignored, so a local run cannot leak
  personal values into the release. The script that used to write them (`docs/emit-default-settings.py`) is gone with the rule.

  **Dist is assembled by hand before a deploy** (owner, 2026-10-26): the build mirrors the game folder into it, but the shipped folder is
  reviewed and finished manually, so nothing here is automated end to end.  - **No `LemonUI-LICENSE.txt` and no verbatim MIT notice ship. That is deliberate and informed, not an oversight — do not "fix" it.** It sits outside the MIT condition that the notice accompany copies, and the copyright holder's real name is in the dll's own assembly metadata in any case, so the file bought no privacy.
- [ ] Confirm it excludes `ScriptHookVDotNet2.dll`, logs, crash artifacts and stale legacy settings files.
- [ ] Read the staged release `README.txt` for installation, matched-SHVDN, menu and log-troubleshooting instructions.

## Limitations to disclose

- In-game creation is live (the creator's Save Track); no track update, editing or deletion — the `UpdateRoute` path was cut (`c2ad2de`).
- No per-car XML saves, saved drivers, personalities or discipline filtering.
- The grid uses model-level pace, not the installed upgrades of a particular vehicle instance.
- Low-grip steering behavior still needs the dedicated release-gate drive above.
- The beater/rust matte paint rule is implemented and ships **unverified in game** — drive a `rusty`/`rat look`/`junkyard`/`primed` livery before trusting it.

## MUST GO IN THE README (owner, 2026-10-26)

When 0.9.0 is published, tell people this in the README, not only in the release body:


- **The pool now admits every four-wheeled class.** Off-road, industrial, utility, commercial, service, emergency and military cars
  are no longer filtered out; only motorcycles, bicycles, boats, helicopters, planes and trains are excluded. The indexed fleet grew
  from ~472 to whatever the roster holds, so grids can mix types they never mixed before (a truck can end up in a sports car's pace
  band). That is intended: the band decides who races whom, not the body style.
- Why it changed: a car could vanish from a one-car roster with the message "No car in the pool carries a pace index", which named
  the wrong cause - the Karin Everon is an off-roader and the off-road class was blacklisted.

## Release sequence — 0.9.0 (prep state)

`AssemblyInfo.cs` carries `0.9.0.0` ahead of the cut, so the in-game banner announces the version the tag claims; the two are still hand-moved and nothing derives either from the other. The notes are hand-written in `docs/RELEASE-NOTES.md`, because the auto-generated body would span every commit since `v0.8.6` and read as noise.

1. Land and push the release commit. The published zip is **assembled and uploaded by hand** (owner, 2026-10-26): CI no longer runs on tags,
   so nothing can race our asset onto the release - it builds, and keeps a testable artifact on every push to master.
2. Build locally, so `RefreshDist` mirrors the game folder into `Dist\AutosportRacingSystem`. That mirror is data only (tracks, `cars.txt`,
   `Drivers\names.txt`, `Settings\tips.txt`); the binaries and the banner are added at packaging time.
3. Assemble the zip with `powershell -NoProfile -File tools\make-package.ps1` (add `-SkipBuild` to package what is already on disk).
   It emits `AutosportRacingSystem-DD-MM-YYYY.zip` in the repo root - the name the README advertises - containing exactly one folder,
   `AutosportRacingSystem\`, with: `ARS.dll`, `LemonUI.SHVDN2.dll`,
   `build-number.txt`, `README.txt` (install + requirements + LemonUI credit), `Tracks\`, `Vehicles\cars.txt`, `Drivers\names.txt`,
   `Settings\tips.txt`. Never `ScriptHookVDotNet2.dll`, never `*.ini`, never `Vehicles\tests.txt`, never logs.
4. **Clean install before publishing**: extract that zip into a clean `Scripts\` beside the matched SHVDN pair and run the race-loop gate above.
5. Publish: `gh release create v0.9.0 <zip> --title "0.9.0" --notes-file docs/RELEASE-NOTES.md --prerelease`, then
   `gh release edit v0.9.0 --prerelease=false` when you are ready for `releases/latest` to move and the in-game notice to fire.
   WARNING: two files look like release notes - `docs/RELEASE-NOTES.md` (the one this command publishes) and `docs/release-0.9.0.md`
   (the curated draft). Decide which is canonical before the cut and keep the other from drifting.
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

### Closed: the LSIA 66.3% hard brake

The corner at 66.3% of LSIA was culled by the blur gate, so `followTrackSpd` is the only law over that stretch and the
car braked sharply there without leaving the road. **Closed by the route brake cap — and the original suspicion was
stale on both counts:**

- The route speed does **not** read the raw `PreciseCurveRadius`: `RouteRadiusSampled()` (`Racer.cs:3921`) is the
  four-point least-squares fit behind the sagitta guard, and `PreciseCurveRadius` is read only by the lean code
  (`:1011`, `:1020`). There was no unsmoothed step to fix.
- The curvature's brake is capped at `RouteBrakeCap` **0.2** (`Racer.cs:243`, applied at `:2448` — only while the route
  term is the limiting one *and* no corner plans a brake — consumed at `:2787` with `Math.Min` and tagged
  `BrakeReason.Route`). A step in the reading can ask for a fifth of the brake, never for a slam.

Driver-reported fixed 2026-10-25. The cap had already closed it; what was owed was the update to this note.

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
**Probe state.** `RouteBrakeCap` 0.2, applied only while **no corner is inside its braking distance**: a live corner plan owns the brake outright. The first rule read which term asked for the lower speed, ramp included, and it was driven and rejected — inside the ramp the cap blocked the braking the corner had planned, so the car could not slow for the corner it was already committed to. The plan wins by being a plan, not by being the strictest number., and `steerLimitedSpeedGovernsRoute` false: the steer-limited speed is still computed for readout but no longer replaces 70 % of the curvature term, so the route speed is the track's radius alone. Both are one-line switches.
**The projection is a time escalation, and the clock is its only form.** The five-metre distance ramp is gone — it answered the same question from the other direction, and two laws on one pedal is one too many. While the one-second projection is outside the edge **and widening**, the clock runs, and it spends the input at two units per second, bled every tick rather than in steps: full throttle at 0 s, level at 0.5 s, the **0.5 brake floor** at 0.75 s — half of the pedal, because off the line the car does not need to stop, only to be slow enough to turn back. The escalation also stops on speed, not only on the clock: while the car is at or under the speed **its own curvature asks for, minus `OffTrackSpeedSlackMph` 10**, the projection does not bind at all — the plan drives, and the car has the speed it needs to steer back. A car that stays out long enough still reaches the recovery mode. Back inside, the clock resets instantly; the pedal's own slew is what stops that from snapping. The projection outranks the plan and the route cap, because it is the only term that knows the plan has already failed, but it does not outrank the ABS (the physical ceiling the floor respects) nor full countersteer (the tyre is already working diagonally, so brake there saturates the circle instead of sharing it).
## Drive owed — 2026-10-23 evening (builds 763-779)

**Driven and accepted in the car:** the curvature brake cap at 0.2 with the corner owning full authority while it asks for the lower speed; the steer-limited blend switched off (no observable effect, which is itself the finding that the route term rarely binds); the countersteer share at half; the damper's plain crossing with `SteerDampingGain` at 0.5.

**Compile-verified, drive owed:**
- The projection clock at 2/s, with the lane gate, the distance ramp and the hysteresis all removed. Driven once at 1/s with the ramp still in place, never in this shape.
- The brake learner's entrance cut (0.02 per 10 m/s over, uncapped) and the factor range widened to 0.5–1.5.
- `SteerDampingGain` shipping at 0.5 instead of 1.0 — the value the plain crossing was driven at, now the default.

**Known untested interaction:** the entrance cut is uncapped and the share step still runs beside it, so a corner can rebound between two factors. The notice feed is the place it shows; there is no log line behind it.
**The corner keeps the arc, and grows through it.** The plan no longer lets go at its braking target: it holds its own number until the car has actually reached it — forward speed within `CornerLiftGraceMph` 5 mph of the target — and only then starts lifting at `CornerLiftMphPerSecond` 5 mph/s, so the car builds speed through the arc instead of sitting on the apex number until the apex. The gate reads the lifted number, so a car that falls behind pauses the growth without losing what it earned. Past the apex the plan releases outright. The route term still votes through `min`, so a radius that says slower is still the ceiling — the two are meant to work together, not to hand over.

## Terrain speed cuts apply to the route and only the route (2026-10-23)

The slope's `sqrt(hill grip)` and the crest's `routeCrestFactor` multiply **`followTrackSpd` alone**. The corner plan
(`cornerSpd`) takes neither, and `apexTargetWithCurrentRouteGrip` - an unused copy of it - is gone. Why: both factors are
sampled **ahead of the car** (slope over the 1.0-2.0 s window, crest at the 1.0 s and 2.0 s nodes), so multiplying a corner
target by them made an apex two hundred metres away rise and fall with the road under the wheels. Found from a corner that
read as strangely slow on every lap; with the corner cut removed it took its authored speed again while the route term went
on protecting the crests where the car actually is. `MaxCrestLift` (0.1-2.0 g, default 1.0) is the crest tolerance and it is
live again, because the multiplier is back on the route.


## Session close 2026-10-23/24 — builds 816-880 (deployed: 880)

**Driven and reported good by the driver this session**: the outside apex line measured in metres (250 plateau + 50 ramp to 5 m, raw
half width), the lean standing down while the line is active, the stability rate, and the four audit fixes. Everything earlier in the
range was driven at some point unless stated.

**Not driven yet** (compile-verified only, deployed in 880): the outside leg of the lane mover at **half rate** (13 m/s2 and 12 m/s
while the reason is `OutsideLine`), and the eight-second `AvoidHoldTimeoutMs` backstop.

**Open, filed in `AGENTS-BACKLOG.md`, deliberately not fixed**: `rival-walls-500ms-snapshot` (the wall corridor reads a 500 ms rival
snapshot and reopens at 10 m/s - the next one to attack, and the most likely remaining cause of "never the same corner, never the same
lap"), `track-point-resnap` (a whole-route re-snap with no heading test), `lane-two-frames` (`ownLane` and `steerRefOffset` compared
across two reference frames, ~2.3 m of sagitta at R 50), `stability-speed-scaling`.

**Instrument this analysis is missing**: the log has second resolution and every lane line is 1 Hz, so lane findings can only be
falsified by same-second correlation. A per-tick lane record for one car - node, metres to apex, wants, entry, reason, request,
carried - is what a proper investigation needs next.

**Editing note**: `Racer.cs` had accumulated mixed line endings from patches that did not convert LF to CRLF; it is uniform again. A
pattern that fails to match text that visibly exists is almost always that. `vscode_replace_text` matches literally, normalises the
EOL and refuses an ambiguous search instead of guessing.


## Session close 2026-10-24 (afternoon) — deployed build 885

**Driven and reported good**: the apex line in metres of route (outside 250 m, ramp 50→5 m, raw half width, latch to the apex, nearest
unpassed apex, corners under 50 m radius), the lean standing down only while the line is active, and the four audit fixes.

**Compile-verified, drive owed**: the line's speed floor at **15 mph** under the apex speed (build 885); the outside leg of the lane mover
at half rate (13 m/s2, 12 m/s, build 880); the 8 s `AvoidHoldTimeoutMs` backstop.

**The ABS is not drive-owed — it ships off by design** (corrected 2026-10-25). The row is **unchecked and greyed**
(`absItem.Enabled = false`, `AutosportRacingSystem.cs:1086-1089`) with the description *"GTA forces native ABS on non-player driven
cars. I'm working on undoing that."*, and `AbsEnabled` is forced false at load whatever the stored key says (`:3463`), because a value
restored from a previous session would be a switch the user cannot see or change and the car would still obey it. The code path stays
built and one line from live, for the day `STATUS_PLAYER` can be put on an AI car.

**Two documents added today**: `AGENTS-VEHICLE-STATUS.md` (all 117 engine sites that branch on STATUS_PLAYER vs STATUS_PHYSICS) and
`docs/KNOWN-BUGS.md` (defects understood and deliberately not fixed; first entry `first-corner-eagerness`).

**The open decision of the day**: GTA re-grants `WF_ABS` to every non-player-driven car each tick (`Automobile.cpp:3775`), so an AI car
cannot be made ABS-free from `strModelFlags`. The engine's own clamp holds slip at 0.6 and our taper's knee is 0.5, i.e. 0.1 of room.
Either ARS writes `STATUS_PLAYER` into an AI car - which drags in air control, damage thresholds, population culling and collision
response, all catalogued in `AGENTS-VEHICLE-STATUS.md` - or the ABS stays a player-only feature.

## 2026-10-24 late — LANE CHAIN DRIVEN AND VERIFIED, build 915

**Driver, after running F8 in both directions: every apex on every direction works.** That closes the long lane investigation.

What was wrong, in the order it was found (each one produced the same class of symptom, which is why they had to be peeled):

1. **`demandInside` inverted on right-hand corners** (build 915). `cornerDir * offset` grows outward on a left and inward on a right, so
   comparing it with `cornerDir * ownLane` read "inside" only on lefts. `_apexLineInsideNode`, `_apexLineInsideCornerDir` and therefore the
   whole inside mode existed on lefts only. Found by running the same circuit backwards: the misbehaving apexes swapped sides, which is the
   signature of a sign asymmetry rather than a timing one. Side-neutral form: `(demand - ownLane) * -cornerDir > 0`.
2. **The cross teleport needed a reversal** (build 910). Comparing aim and demand against the car's lane alone pinned the aim to the car's
   lane every tick of an out-in, so the line reached the centre and hovered. It now also needs `gap * _laneRate < 0`.
3. **The adopted corner owns the line** (build 909) — `NearestApexAhead` hysteresis of 30 m, and the ratchet is no longer wiped when the
   blend window flickers (it contains the speed floor, which moves with grip).
4. **`steerRefOffset` measured in the car's own node frame** (build 905) — against the steer reference up to 30 m ahead it carried the
   sagitta `L^2/2R`, up to 9 m on a 50 m corner, with the sign following the corner.
5. **The last second is a commitment** (build 914): `secondsToApex <= 1` forces the inside extreme whatever the blend managed.

**Tools left in place**: the white diagonal from the car to the adopted apex node (build 913, `ShowTrackAnalysis`), and `Lane cross` /
`Corner line` (`t=`, `apex=`, `dir=`, `out=`, `bias=`, `in=`) in the log.

**Still open, unchanged**: audit findings 4, 5, 7, 8, 9 from `deleg_7fee6020`; `apex-stays-referenced` and `first-corner-eagerness` in
`docs/KNOWN-BUGS.md`.

## 2026-10-24 late - corner entry/exit floor, VERIFIED

Build 932: `AddCornerRegion` walks the green entry and the red exit out to at least one track width (in track nodes) from the apex.
Driver on F8: **the cars take the corners better, with more corner to manoeuvre in.** It binds on most corners rather than rarely - the
logged walks were returning 6 to 64 list positions - which is exactly the class of strange apex it was meant to remove.
## Session close 2026-10-24 (night) — build 932 deployed, NOT pushed

**Where the tree is**: everything committed locally; the last pushed commit is `749157c`. The only dirty entries are the driver's own
untracked track files (`Dist/AutosportRacingSystem/Tracks/F8.xml`, `TestOct.xml`, `Tracks/testing/`) - theirs, untouched. Deployed build is
**932** (`4db8ff93...`).

**Verified on track today, all by the driver**:
- Every apex, in both directions, on F8 - after the `demandInside` sign fix (915).
- The brake learner: one lap to learn a corner, +/-0.02 per lap once settled (922).
- The corner entry/exit floor at one track width (932): *"los coches toman las curvas mejor - tienen mas espacio considerado corner"*.

**Driven since, driver report 2026-10-25**:
- The apex blend's clock on the entry speed latched at the window, with no cushion (917/926) — **correct**.
- The blend ends and pins the inside at a quarter second (`IdealLineBlendEndSeconds` 0.25, `Racer.cs:1203`, read at `:1389` as
  `apexCommitted`) — **adequate behaviour, not exactly verified**: the driver is not certain the pin actually fires, and no log
  line proves it. What was judged is the result on track, not the mechanism.
- The braking modulation of the clock (`LineClockBrakeMphPerG` 2, `Racer.cs:1352`, clamped to ±10 mph) — **driver-observed
  working**: *"veo la línea morada ajustar su velocidad acorde; coche más lento, la línea morada avanza al interior más lenta"*.
  What that observation pins down is the clock running on the **latched entry speed** (`:1348-1351`, `secondsToApex = metres /
  clock`): a slower car stays further from the apex longer, so the purple line's inward advance slows with it. The ±10 mph brake
  boost rides on the same denominator and has no separate observable, so it was not isolated — accepted on feel.

**Changed but not driven**:
- The outside leg of the lane mover at half rate (13 m/s2 and 12 m/s while the reason is `OutsideLine`, 880).
- The eight-second `AvoidHoldTimeoutMs` backstop (880) — **not ChillOut**: this is the *lane* hold after a pass (`ComputeAvoidanceLane`,
  `Racer.cs:1470-1500`), which keeps the pass lane until nothing is left alongside (14 m behind, 1.5 m lateral) and dies after 8 s
  because a pass that long is not a pass any more. Without the backstop the held lane outranked the corner line for the rest of the
  race when the geometric release never fired. ChillOut is the separate **throttle** manoeuvre that holds a standoff behind the
  closest rival (`:2450`, `ManeuverType.ChillOut`).
- The line's speed floor at 15 mph under the apex speed (885).

**Filed, not fixed**: `docs/KNOWN-BUGS.md` - `apex-stays-referenced`, `first-corner-eagerness`.
**Audit findings still open** (`deleg_7fee6020`): the overshoot deadband at 0.1 m against overshoots of metres; the lean sampling a single
node; `lineActive` vs `entryActive`; the needle-thread averaging two rivals' lanes; the turn-in exempted on a 1 Hz queue.

**First thing next session**: the audit's finding 4 (overshoot deadband) - it is the one the driver would feel.

## 2026-10-25 — the avoidance hold is off (build 934, deployed)

`ComputeAvoidanceLane` carries `bool avoidHoldEnabled = false`: the pass lane now lives only while the target does, i.e. the
pre-`54fc02a` behaviour. The fields stay, unused, so it can be turned back on without re-writing it.

**Why**: `ApplyRivalWalls` already constrains every lane request - corner line, lean, avoidance, recovery - against a rival
beside the car, closing instantly and opening at 10 m/s. The hold may therefore be a second law for the same nose clip, and
the 8 s ceiling made it the dangerous one: a stuck geometric release would hold a pass line for up to 480 m.

**Driven, 2026-10-25 (build 934): no cross after a pass — the claim is invalid.** The behaviour the hold was written for, the
lane snapping back and the car turning in on the car it just passed, does not reproduce with it off. `ApplyRivalWalls` covers
it on its own: the wall holds while `|LongitudinalGap| < CombinedSize.Y` (body half-sum + 0.25 m behind, +1 m ahead) and then
opens at 10 m/s, which is enough for the car to unwind clear of the passed car.

**Kept as zombie code, driver's decision (2026-10-25)**: the fields, `ComputeAvoidanceLane` and `AvoidHoldTimeoutMs` stay in
the source, unused, marked `ZOMBIE` at the switch and at the constants — *due to be revisited and removed if it truly is of no
use*. Nothing is rebuilt for the annotation: 934 stays in the car.

### Same day, build 941 — the margin was a half width wide twice

`CombinedWidth` is the **average of the two full widths**, i.e. the car's half width plus the rival's — the code calls the same
formula `combinedHalfWidth` at `:787`. Both the pass lane (`:1559`) and the wall target (`:1631`) then added
`VehicleData.BoundingBox * 0.5f` **again**, so a clean pass left a full half width of air (≈0.95 m) on top of the aggression
buffer (0.2–1.2 m), and the wall sat that far outside the rival's own body.

Both now carry `rival.RivalRacer.VehicleData.BoundingBox * 0.5f + aggroBuffer`, and the half width the caller adds puts the
car's **body edge** on the rival's edge, which is what `ClampLaneToWalls` was written to mean. Expected effect: passes closer by
~0.95 m, walls that visibly sit on the rival's flank, and — with a fixed 5 m/s rate — a shorter distance to travel either way.

**Drive owed**: whether the tighter corridor still passes without contact, at both ends of the aggression range.

### Build 943 — the predictive branch of the wall gate is gone

The wall now constrains only for a car whose **body is alongside**: `|LongitudinalGap| < CombinedSize.Y` and the same
30 m of route. The `aheadAndClose` branch — `RelativePos.Ahead`, `TimeToReach < 3 s`, `|DirectionDiff| ≤ 20°` — is deleted.
It put walls up for cars that were merely approaching, so the corridor moved with nothing on screen to explain it, and the
angle gate made the wall appear and vanish with the rival's heading.

What survives, and why: the same-section 30 m test stays, because it is what keeps a car on the far leg of a U from counting
as a trackside blockage when it is physically close.

**Known inconsistency, next**: the test still reads `LongitudinalGap`, which is measured **in my car's own frame** — so a rival
at 45° has a small longitudinal component and can fall out of overlap while it is close. See the proposal in the session that
follows.

### Build 945 — the overlap moves to the route axis, and the shape gains its cone

`overlaps` now reads `|RouteGapAhead| < CombinedSize.Y`: the same axis `0623276` gave rival detection, so the gap no longer
shrinks for a rival at an angle, a sliding car stops toggling it, and corners stop under-reading it. The 30 m same-section
guard went with it — an arc shorter than a body cannot reach the far leg of a U — so the wall gate is one condition on one axis.

The Show Rivalry drawing gains the `\./`: two lines from the car's own position to each wall's far end, in the wall's own colour,
so the corridor reads as a cone opening from the car instead of two floating lines.

**Drive owed**: whether the wider overlap window (arc ≥ chord through a corner) holds the wall too long, and whether the route
gap wraps correctly at the start line — a wrong wrap would flicker the wall once a lap.

### Build 947 — two rules against changing sides, and why they are two

Both live in the pass picker, and they answer different geometry:

1. **Within a metre nose to tail, no flip** (`SideFlipMinClearanceMeters` 1): the crossing itself is the risk — the rear of the
   car ahead is still in the way while this one goes round. Further back the flip stays free.
2. **Side by side, the side is latched** (`|NodeOffset − its NodeOffset| < CombinedWidth`): once the two bodies are abreast
   there is no room to cross over, so the lane the pass began on is the lane it finishes on. The geometry is not asked again
   until the lateral overlap ends, and the latch is per target, so a different rival starts a fresh decision.

They had to be split: written both as longitudinal arc, the 1 m rule lands **inside** the overlap window (`CombinedSize.Y` ≈ a
body and a bit) and would never fire. The narrow one is longitudinal, the committed one is lateral — one car behind you does not
pin you, only one alongside.

**Drive owed**: a pass where the rival moves across the centreline mid-overtake — the old behaviour flipped the lane with it.

### Build 949 — the wall's side is relative to the car, not to the road's centre

The root of "the wall crosses the rival and drags me to the other side of it": the side was keyed on `r.NodeOffset <= 0f` —
the rival's offset from the **track centreline**. On a two-lane road the midpoint runs between the two cars, so a rival
drifting by a few centimetres across it swapped walls in a single tick, and the corridor went with it.

- The side is now `sideGap = r.NodeOffset - myNodeOffset`, so **my own lane** sets which side of me the rival is on.
- `WallSideLockMeters` 1: within a metre nose to tail the side is not re-decided — crossing there clips its rear.
- `abreast` (`|sideGap| < CombinedWidth`): the side is latched outright; alongside there is no room to cross over, so the wall
  stays where the pass began. The latch is per slot **and per rival**, so a different car in the same slot decides fresh.

**Drive owed**: a rival that crosses the centreline mid-overtake, and a three-wide pinch where two slots latch opposite sides.

### Build 950 — the latch is gone, the fold is pure

Build 949's per-slot latch was the last thing making the corridor depend on anything but the current geometry: a rival that
left the three slots and came back kept the side it had been given before, and `UpdateRivals` re-sorts slots by distance every
tick, so the far car could outvote the near one. Deleted — no fields, no memory.

The side now comes from `r.LateralGap > r.CombinedWidth`: the rival's lateral offset **in this car's frame** (+ is right), with
a body width of deadband. Inside that band the rival is straight ahead and takes the same side every tick, so abreast never
swaps — which is what the latch was for — and nothing outside it can be remembered.

The accumulation is `Math.Max` on the left wall and `Math.Min` on the right, both commutative: the most constraining rival wins
and no rival can open a wall another closed.

**Drive owed**: the case that started this — a car constrained against a rival further away than the nearest one — and a pinch
where two rivals sit one each side.

### Build 955 — the angle gate leaves target selection too

`isAvoidanceCandidate` (`Racer.cs:3811`) no longer tests `DirectionDiff`: whether a car is in the way is a question about its
body, not about where it points. A spun rival inside this car's lateral corridor is now a legitimate target to go round,
which is the case the gate was quietly refusing.

The corridor test that does protect survives inside `TimeToReach`, which returns infinity for a rival outside `CombinedSize.X`
of this car's offset — so a car angled away but in another lane still cannot become a target.

`AvoidAngleGateDegrees` remains in use by the second-rival (needle) filter of the pass lane; it is no longer load-bearing for
the walls or for target selection.

### Build 956 — the walls can no longer leave the road (the jitter)

Driver saw the walls jump outside the track for a tick and snap back. Cause, found with a read-only subagent's simulation and
matching arithmetic:

1. `targetLeftWall = Math.Max(targetLeftWall, r.NodeOffset + rivalBuffer)` — **never clamped to the edge**. A rival near or past
   one edge (its `NodeOffset` 7.9 on a 6 m half-width) produced a left target of 9.55.
2. The corridor inverted, and the ordering guard wrote `mid ± 1` — with `mid` outside the road, **both walls** landed off the
   track (5.81 / 7.81), leaving the car aiming outside it.
3. Next tick the targets recomputed and the rate walked them back. One-tick spike outward, smooth return: exactly the symptom.
4. Compounding it: the side test `LateralGap > CombinedWidth` is **one-sided**, not a deadband, so a rival 1.5 m to this car's
   *right* is also sent to the left wall — it needs only a rival within a body width of the edge to reach the out-of-bounds case.

Three clamps, one rule — a wall is a lane offset inside the road:

- targets clamped to `±trackBound`;
- the stored walls re-clamped every tick, because they are measured against the previous tick's width and a narrowing road
  leaves them outside the new edge;
- the ordering guard keeps its ±1 m corridor *inside* the edges (`corridorHalf = min(1, trackBound)`).

**Drive owed**: the same narrowing straight and the edge-hugging rival, with Show Rivalry visible.

### Build 957/958 — the subagent's report was right, and 956 only bounded the symptom

956 clamped the values, so nothing left the road — but the **oscillator survived**: the ordering guard wrote `mid ± 1` into
the *state*, the mover dragged both walls back at its rate, the targets re-crossed 4-6 ticks later, and it repeated. A
relaxation oscillator with the period set by `wallCloseRate`.

The read-only subagent reproduced it numerically (a Python copy of the block, not a drive): one rival at `NodeOffset +5.6`
against a 6.0 m half-width, `rivalBuffer` 1.65, target 7.25 → guard 5.625/7.625 → 1.6 m outside the edge, jumping 0.84-1.0 m
every tick, identical at 20/30/60/120 fps. It also named the trigger: `LateralGap > CombinedWidth` is a **one-sided threshold**,
not the deadband its comment claimed, so a rival 1.5 m to the right was sent to the left wall — the collapse then fired on one
rival in ordinary two-wide running, not only on a pinch.

Three changes now in:

- **the guard runs on the targets**, not the state (`if (targetLeftWall > targetRightWall)`, corridor half a metre clamped
  inside the edges); the state write-back is deleted, so there is nothing left to undo — a fixed point instead of a cycle;
- **the teleport writes are clamped** to `±trackBound`, so a jump can never land outside;
- **`WallSideDeadbandMeters` 0.3** replaces the body-width deadband: only a rival on the centreline counts as straight ahead.

**Drive owed**: the same two situations. The subagent left a discriminator — a **blue** wall leaving the track could only be the
collapse, a **red** one resting exactly one `BoundingBox` short of its target is the teleport. After 958 neither should occur.

### Build 951 — the approach half comes back; 943 took both halves, not just the broken one

**Correction to 943**: deleting the `aheadAndClose` branch also removed the only rule that put a wall up for a car that is
*approaching*. The walls are what starts an overtake, not only what survives one, so with the branch gone a wall could not
appear until the bodies were already level — and the car never moved over.

The approach is back as one condition, `r.TimeToReach < WallApproachSeconds` (3 s). `TimeToReach` already carries every gate
that matters — ahead in arc, inside this car's lateral corridor, and genuinely closing — and returns infinity otherwise, so it
needs no angle test and no `RelativePos` classification. Those two were what made the old branch flicker; the approach itself
was never the problem.

## Session close 2026-10-25 (evening) — builds 934-988

**Audit swarm** (13 Kanban cards, read-only, ~330 KB in `docs/audit/`). Verified by hand before touching code: six
blockers real and fixed (NaN out of the tick, the player's car deleted at teardown, the player's ped left invulnerable,
a malformed track file stopping startup, unbounded wheel-pointer reads, rival records one second stale), one real by
design (the route brake cap only applies outside corner plans), one false (`ScriptSettings.Save()` returns void).
The rest of the reports — ~38 blockers — are still unverified claims.

**Nitro.** The player's burn never showed because nothing lifted the entity's speed cap: the 2.5x power multiplier drove
the car into the model's own top speed. `StartNitrous` now sets `SET_ENTITY_MAX_SPEED(Car, 50000)` and `StopNitrous` puts
the model's top speed back with `MODIFY_VEHICLE_TOP_SPEED(Car, 0)` (hash not in SHVDN's enum). Taken from
`FIying-Scotsman/GTAV-EnhancedNativeTrainer`, cloned to `D:/Projects/FromInternet/`.

**Power channel — the rule that now holds.** Only two reasons may raise power above stock:
- the burn, at `NitrousPowerMultiplier` (2.5, x `AiNitroPct` for AI);
- artificial rubberbanding, at `ComputeRubberBandFactor()` for an AI car that is behind.
`UpdatePowerCompensation` can only reduce (`_overspeedPowerLevel`, player-only), and `ApplyPowerMultiplier` takes one
reason per tick — nothing multiplies with anything else. Removed: the pedal-gap boost (x1.5), the over-top-speed boost
(x1.35), and the artificial rubberbanding that used to ride the throttle above 1.0.

**Menus.** Submenu titles carry the trail in the `Subtitle` (`MAIN MENU > RIVALS > RIVAL TRACTION`), written in one pass
after the tree exists — computing it while linking crashed the menu, because menus are built children-first. Menu build
is wrapped so a failure logs at Error instead of taking the mod down. Show Track & Grid Rows ships off and greyed.

**Defaults** now the driver's tuned set: AutoRepair, Laps 10, GridSize 8, CornerOffset 4 / RouteOffset 2, TCS 100/50,
Damper countersteer 2.0, Steer ceiling 0.9, MaxCrestLift 0.4, Rubberbanding 100, every debug toggle off except the
leaderboard.

**Owed a drive**: the AI without the pedal-gap padding (a car may now fall short where it used to be propped up), and the
artificial rubberbanding as explicit power.

## Nitro, closed 2026-10-25 — builds 989-993

The burn is now one law and nothing else. `StartNitrous` no longer touches `SET_ENTITY_MAX_SPEED`; the burn sets
CheatPower outright at `NitrousPowerMultiplier` (x `AiNitroPct`), the packing trim and the anti-overspeed cap are
both exempt while it lasts, and the AI's plan is therefore the road's plan for those three seconds.

**Measured, same car (F620), both at 2.50:** 64 -> 99 mph takes ~1.8 s for the player and ~1.8 s for the AI. There is
no asymmetry to fix; the perceived gap is felt-vs-watched, plus different PI. If it reappears, the readout is the
`Burn` line, not an opinion.

Settings the driver runs (worth knowing before reading any report): `TcsEnabled = False`, `OverspeedCut = Off`,
`AiNitroPct = 100`, `Rubberbanding = 100` in Natural. With TCS off the burn lights the tyres early
(`slip -3.04`) and grip only recovers after ~1 s of the 3 the bottle buys.

**Diagnostics left in place** (`Info`): `Burn <car> (player|AI), power, throttle, caps, plan, speed, slip` every 500 ms
during a burn; `AI nitrous: <car> lap N, plan X mph, packing xF`; `Nitro Gs: measured, wheel, excess` inside the
overspeed sampling block (only when `OverspeedCutPerTenthG > 0`).

**Still open:** the 13 audit reports in `docs/audit/` remain uncommitted, and ~38 of their blockers are unverified
claims. Owed a drive: the AI without the deleted pedal-gap compensation, artificial rubberbanding as explicit power.

### Throttle cap removed, 2026-10-25 (build 1005)

`ModulatedThrottleCap = 0.99` is gone, and with it `ShapeLaunchThrottle`: the throttle now goes to the game whole,
1.00 included. **Why it existed (driver's note, keep this):** at a literal full throttle GTA applies its own
full-throttle behaviour and *takes grip away*, so the 0.99 cap kept the launch out of that range instead of fighting
it. It is disabled on purpose — dodging a native behaviour is not the same as not having it, and the driver wants to
see what the car does with the whole pedal.

Consequence to watch: the launch and every full-throttle exit now run through the game's grip loss. With `TcsEnabled
False` on this machine there is nothing modulating it, so wheelspin off the line is expected. The pedal chain itself
is clean end to end: `Speed -> IntendedSpeedChange -> combined input -> throttle/brake` has no dead zone, verified
stage by stage (the launch relay was the only one, removed in 1004).

## Session close 2026-10-25 (late) — builds 994-1011

**Nitro, settled by measurement on track.** The player's burn and the rivals' are two different mechanisms: the
player's rides GTA's own nitrous force (`Transmission.cpp:1624`, `fDriveForce *= 1 + 3.0 x powerMod`, with our power
override at 1.0), while an AI car never sets `GetNitrousActive()` — nothing presses boost for it — so it only gets
`SET_VEHICLE_CHEAT_POWER_INCREASE`. Measured: the player at 1.0 and a rival at 2.5 produced the same shove. Bridging
that properly (calling `SET_NITROUS_IS_ACTIVE`, `0x9E566EA551F4F1A6`) was **declined as too complex for now**; the
values were doubled instead: player **2.0** (`PlayerNitrousPowerMultiplier`), rivals **5.0** at `AiNitroPct 100`.

**Pad binding.** The fire check was `Game.IsKeyPressed(Keys.X)` — keyboard only, so a gamepad had no trigger at all.
`GTA.Control.VehicleDuck` works; `GTA.Control.VehicleHorn` never fired and was removed. Index `2` = gamepad only;
keyboard X still works.

**Pedal law, now 1:3.** `FullPedalSpeedErrorMps = 1` (full gas at 1 m/s of deficit), `BrakeErrorMultiplier = 3`
(full brake at 3 m/s = 6.7 mph). The chain `Speed -> IntendedSpeedChange -> throttle/brake` was audited stage by
stage and has no dead zone; the launch relay that quantised the pedal to 0/1 below 27 mph was the only one and is
gone. `ModulatedThrottleCap` (0.99) was removed too — it existed to dodge GTA's own full-throttle grip loss, and
dodging a native behaviour is not the same as not having it.

**Cards.** ChillOut parked behind `const bool ChillOutEnabled = false`; the nitro card cannot fire inside the first
6 s (`NitrousArmDelayMs`); the `DIVEBOMB`/`DEFEND`/`YIELD`/`CHILL` labels over the roofs are now behind `ShowRivalry`
instead of always on. `RivalLiftFloor` 0.5 -> 0.1.

**Track surgery.** `Tracks/Figureight.xml` now carries F8's 2350 route nodes (same layout, new editor, same rotation,
start line ~55 m earlier); everything else — name, tags, 19 props, trackside — is still Figureight's. Backup at
`Tracks/Figureight.xml.bak` (deliberately not `.xml`, or the mod would list it as a track).

**Settings.** Laps default 6. The legacy migrations now seed only from a key that really exists, so a fresh install
takes its defaults from `KeySpec` and not from a number copied into a `GetValue<int>(..., 60)` fallback.

### Slip source toggle, closed 2026-10-25 (builds 1056-1059)

The Development submenu carries `Slip From Memory` (persisted in `Menu-Development.ini`): **on** reads the front tyres' side slip
from the wheel memory, **off** derives the demand from the inner wheel's radius against the grip the car has.

**Decision taken: the two sources are two laws, not two sensors.** The memory source reports how much of its peak slip the inner
tyre is using; the theory source reports how much of the grip the inner command is asking for. At 60 mph they put the ceiling at
7.97 deg and 2.81 deg respectively, so switching changes the behaviour and not merely where the number comes from. Normalising
them would mean turning the lateral demand back into an equivalent slip angle, i.e. reintroducing the tyre model the theory source
exists to avoid. That is left as a "later, if it can be done cheaply" item.

**The theory source, as built:** `innerRadius = L / tan(command)` (the command IS the inner wheel's angle; the outer takes 0.75
of it), `demand = v^2 / innerRadius`, `available = grip x 9.81 x share`, and the ceiling is the same question inverted on the
same radius. Guards: no answer below 3 m/s, on a NaN grip/speed/wheelbase, when the radius at the limit falls inside the
wheelbase, or in reverse (neither source answers backwards - the file's own contract is that reverse keeps the full lock).

Audited by two subagents (plumbing and mathematics). Their findings, re-checked here: the safe batch went in as build 1058
(NaN floors, reverse off, ceiling clamped to the lock, raw readout only from the memory source, the Development store wired into
`CompleteOwnedKeys`, docs synced); the geometry batch as build 1059. Open, deliberately: the two laws are not comparable, and
`LeftPct`/`RightPct` are duplicated from `InnerPct` in theory mode because the model has one axle and no sides.

### Front-slip guard, session closed 2026-10-25 (builds 1050-1089)

The steering limiter is now: `maxSlip` (peak at speed x share x slow-corner allowance) -> `clampTarget` (lock when
there is no overshoot, `request x factor` when there is, saturating at 2.5x where the tyre's curve flattens) ->
`chasingClamp` (the only state; shuts at 1-10 deg/s by severity, opens at 4). The pedal follows the same band,
throttle only, closed between x1 and x2.5 and never below zero. Full design note, game-source evidence
(`sfTractionPeakAngle` 1.0, `sfTractionMinAngle` 2.5), the on-track capture and the open items:
`docs/steering-and-pedals.md`.

Two things the session established that are easy to lose:

- **The whole guard is AI-only.** `ApplySteerLimits` sits inside `if (!ControlledByPlayer)`, and the readout only
  draws for `DebugFocusRacer`, so the numbers on screen always belong to a rival. The player's steering is never
  written by ARS.
- **The 0.40 factor floor is load-bearing and nobody wrote it down.** It falls out of `100 / 250` (the 2.5x saturation),
  not out of `FrontSlipFactorFloor`, which is 0.30 lower and never reached. Proposed fix in
  `docs/steering-and-pedals.md` item 5: `Clamp(100 / InnerPct, 0.40, 1)` and delete the saturation constant.
- **A four-wheel slide is not an understeer.** `F` and `R` near each other in `docs/steering-and-pedals.md`'s
  capture means both axles are past the peak, where cutting the wheel buys nothing; speed is the only lever.

## Session 2026-10-26 (early) — the second audit, and eleven fixes

Board `ars`, five cards on `deepseek/deepseek-v4-flash` (Command Code), reports in `docs/audit/301-305`. It did **not**
repeat the first audit: it was aimed at the gaps. What it found: 43 of the 44 inherited rows were **real** (301), no
blocker in appearance/vehicles (302), one blocker in the grid builders (303-G1), three in the grid-fill refusal (304),
and nine of nine fixes verified with four named residuals (305).

Fixed and deployed, builds **1107-1111** (each its own commit, each hash-checked against the deployed dll):

- **304** — a starved grid says which cause it is (PI band / no pace index / empty) and Start Race does not start.
- **303-G1, G2** — both grid builders adapt to short routes; `Place` never indexes an empty grid.
- **305 residuals** — the abort clears moved into a `finally`; a spent burn still clears; unticking the setting cuts a
  live burn; a routeless file is refused before the fade; the creator's message counts nodes.
- **303-F1, F3** — freecam leaves on death, respawn or a vehicle; no view mode is forced that cannot be restored.
- **302** — the tuner re-tests the player's car at the tick, not only at enqueue.

**Owed, in this order**: drive the 1111 — (1) Start Race with nothing in the PI band, (2) a solo race on a short
circuit, (3) freecam while entering a car. Nothing here is driver-verified.

**Open, deliberately**:
- `301` — the 43 real rows. All release-note or file grade; a multi-session job, not a blocker.
- `303-F2` — the dead `HideHudMode` reset. Its proof is a decompile this side could not reproduce; touching it blind
  can leave the HUD hidden, so it stays.
- `302` — the player keeps an AI car's permanent look if they steal one. A design consequence, not a defect.
- `B5` (rival record lag) and `R12` (leaderboard per frame) — parked for 0.11.0 in `AGENTS-BACKLOG.md`, R12 with its
  98 % claim corrected.
- The tag: `v0.9.0` is not cut, and `Dist/AutosportRacingSystem/Tracks/Figureight.xml` is modified and uncommitted.
