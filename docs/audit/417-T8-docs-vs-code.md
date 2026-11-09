# 417-T8 — docs vs code (read-only audit)

Date: 2026-10-30 · Build: **1380** (`build/build-number.txt`) · Scope: AGENTS.md, AGENTS/AGENTS-WIP.md, AGENTS/AGENTS-BACKLOG.md, docs/FEATURES.md, docs/TEST-PLAN.md, docs/VERIFY-NOW.md, README.md, tools/PACKAGE-README.txt, tools/make-package.ps1.

No source, doc or build was edited. Every claim below was read in the file named, at the line named, on this build.

## Files read

| File | What was checked |
|---|---|
| `AGENTS.md` (142 lines) | every `file.cs:NNN` anchor; the steering-clamp block (§132-133); the menu/log/reload claims |
| `AGENTS/AGENTS-WIP.md` (655 lines, first 120 read in full) | the release gate, package gate, versioning, the shipped-vs-owed wording |
| `AGENTS/AGENTS-BACKLOG.md` (ladder lines 7-151, decision-ready top 222-245) | state tags, `Racer.cs:` / `AutosportRacingSystem.cs:` anchors, closed items |
| `docs/FEATURES.md` (143 lines) | every player-facing claim with a number, a key, a menu row or a default |
| `docs/VERIFY-NOW.md` (91 lines) | drive list, build header, file names, log names |
| `docs/TEST-PLAN.md` (the steering/ceiling section, ~lines 363-374) | the live checklists under the revision notes |
| `README.md`, `tools/PACKAGE-README.txt`, `tools/make-package.ps1` | requirements, bundled versions, package contents, install text |
| `src/*.cs` | `AutosportRacingSystem.cs`, `Racer.cs`, `SettingsRepair.cs`, `MenuSettings.cs`, `GridBuilder.cs`, `TrackLoader.cs`, `DataStructures.cs`, `Tips.cs`, `Properties/AssemblyInfo.cs` |

Confirmed-correct claims worth recording (checked, no drift): menu chord `Left Shift + E` (`AutosportRacingSystem.cs:175-176`); `MenuModifierKey`/`MenuKey` in `Menu-Settings.ini` (`SettingsRepair.cs:161-162`); `arsmenu` cheat (`:2117`); `ARS.log` at the game root (`:637`); LemonUI 1.5.2 and SHVDN2 2.11.6 (file versions of `libs/*.dll`); four ini files `Menu-Race/Menu-Settings/Menu-Debug/Menu-Development`; `Peak Slip Share` ladder 0.05-2.00 shipping 0.90 (`SettingsRepair.cs:151`); the clamp constants 15/s, 90 deg/s, 72 deg/s (`Racer.cs:273-277`); `chasingClamp` as the only applied ceiling (`Racer.cs:1697-1710`); player grid slot last (back on a circuit, front on a point-to-point, `GridBuilder.cs:21-23`); the leaderboard's PI and BEST LAP columns (`:2333-2335`); 15 track XMLs and 828-entry `cars.txt` in `Dist/`; `Settings/` shipping only `tips.txt` + `.gitkeep`; no `*.ini`, `tests.txt` or `ScriptHookVDotNet2.dll` in the package.

## Contradictions

Severity: **H** = misleads a player or a release decision; **M** = sends a drive or a reader to the wrong place; **L** = a pointer that misses.

### 1. The player's nitrous bottle is live; FEATURES twice says it is not — H

```
docs/FEATURES.md:79
Your own bottle is coming. Theirs is live now.
docs/FEATURES.md:139
- **The player's own nitrous bottle.** Coming soon — the rivals' bottles are live.
```
```
src/AutosportRacingSystem.cs:2079-2081
if (NitrousEnabled && PlayerRacer != null
    && (Game.IsKeyPressed(Keys.X) || Game.IsControlJustPressed(2, GTA.Control.VehicleDuck)))
    PlayerRacer.TryFireNitrous();
src/Racer.cs:3027-3045  public bool TryFireNitrous() … ARS.Log(… "Player nitrous fired on lap " + Lap); StartNitrous();
```
Wrong side: **the doc**. The player has a bottle, one per lap (`Racer.cs:2807` `Lap <= _nitrousLapUsed`), fired on `X` or the duck control while a race runs, and it logs `Player nitrous fired on lap N`. The tour also never documents the `X` key, so the one hotkey a player needs for their own bottle is missing from the only player-facing file.

Second-order defect found in the same path: the burn's boost is `PlayerNitrousPowerMultiplier = 1f` (`Racer.cs:419`, applied at `:3507-3509`) while the AI's is `5f` (`:417`) — so the player's bottle charges and plays, and multiplies power by 1.0.

### 2. "AI Nitrous Power: 100 the same" — the number scales a 5× base against the player's 1× — M

```
src/AutosportRacingSystem.cs:1179 (menu description)
"The AI's bottle as a percentage of yours: 100 the same, 200 twice. Your own burn is unchanged."
docs/FEATURES.md:77
AI Nitrous Power sets their bottle as a percentage of yours, anywhere from 50 to 200.
```
```
src/Racer.cs:417-419   const float NitrousPowerMultiplier = 5f;   const float PlayerNitrousPowerMultiplier = 1f;
src/Racer.cs:3507-3509 multiplier = ControlledByPlayer ? PlayerNitrousPowerMultiplier
                                    : NitrousPowerMultiplier * ARS.AiNitroPct * 0.01f;
```
Wrong side: **the code or the label** (a tooltip that contradicts the code is a bug). At `AiNitroPct = 100` the AI's multiplier is 5.0 and the player's is 1.0, so "100 the same" cannot hold; the percentage also never changes the bottle, only the power. The offer list is `{50,100,150,200}` (`AutosportRacingSystem.cs:70`), so FEATURES' "anywhere from 50 to 200" also overstates the granularity.

### 3. The outstanding-drives file is 68 builds behind — M

```
docs/VERIFY-NOW.md:3
Build desplegada: **1312**.
```
```
build/build-number.txt
1380
```
Wrong side: **the doc**. Every ✅ in the file is dated 1294/1312, and 68 builds (including the lane, card, state and grip commits since) have not been folded in, so the file's own banner no longer says which binary the open drives refer to.

### 4. The rotated log the drive is told to check is not the file the code writes — M

```
docs/VERIFY-NOW.md:26
existe `Log.prev.log` con la sesión anterior
docs/VERIFY-NOW.md:91
Si vuelve, `Log.prev.log` ya guarda la sesión anterior.
```
```
src/AutosportRacingSystem.cs:638
public static string PreviousLogPath => Path.GetFullPath(Path.Combine(ScriptsFolder, "..", "..", "ARS.prev.log"));
src/AutosportRacingSystem.cs:366-367
try { if (File.Exists(LogPath)) File.Copy(LogPath, PreviousLogPath, true); } catch (Exception) { }
```
Wrong side: **the doc**. A7 is a drive that can only "fail" as written: no `Log.prev.log` is ever created. (`AGENTS/AGENTS-SHVDN.md:44,48,80` carries the same stale `Log.log` name, outside this scope.)

### 5. A closed register item states a menu range the code does not have — M

```
AGENTS/AGENTS-BACKLOG.md:136
- `ceiling-factor-range` — closed (`29b267b`): the domain is 0.05 to 1.25 in 0.05 steps. **closed**
```
```
src/SettingsRepair.cs:151
new KeySpec("SteerCeilingFactor", "0.90", Kind.Number, new[] { "0.05", … "1.95", "2.00" });
AGENTS.md:132
The ladder runs 0.05-2.00 and ships **0.90**
```
Wrong side: **the backlog line** — it contradicts both the code and AGENTS.md, and the item is marked closed, so nobody will re-open it to notice.

### 6. A live TEST-PLAN checklist still walks a removed menu item, and quotes a stale range for another — M

```
docs/TEST-PLAN.md:365
the **Steer Limit At Speed** item (0.05 to 1.25 in 0.05 steps, default 1.00)
docs/TEST-PLAN.md:371  3. **The damper knob's range** — at 10 the damper gives up early … at 250 …
docs/TEST-PLAN.md:373  5. **The menu and the file** — both items sit under Steer Damping in Rivals > Rival Steering
```
```
AGENTS.md:131  **The speed axis is REMOVED (driver):** … the schedule and its menu item went
src/SettingsRepair.cs:151  SteerCeilingFactor default "0.90", domain to "2.00"   (menu row: "Peak Slip Share", AutosportRacingSystem.cs:1094)
grep SteerDampingSpeed src/ → no match
```
Wrong side: **the doc**. The section carries its own "**Revised: the Steer Damping Speed item is REMOVED (driver, build 711)**" paragraph and then leaves two numbered checks that assume it exists — item 3 (a 10-250 dial) and item 5 ("both items sit under Steer Damping"). The same section's ceiling row is now `Peak Slip Share`, 0.05-2.00, default 0.90.

### 7. "Nothing else touches your car" vs the player's own power cut — M/L

```
docs/FEATURES.md:21
The one lever ARS does write on every car, yours included, is the game's own power multiplier: it sits at its resting value in normal driving …
docs/FEATURES.md:140
No ARS traction control, launch shaping or crest discipline on your pedals: your car drives exactly the way the game drives it.
```
```
src/Racer.cs:2483-2494  bool straight = ControlledByPlayer ? Math.Abs(VehicleData.GetLateralGs(Car.ForwardVector)) < OverspeedArmMaxLateralGs : …
                        if (ARS.OverspeedCutPerTenthG <= 0f || !armed || !(VehicleData.OverspeedExcessGs > 0f)) return 1f;
                        float cut = ARS.OverspeedCutPerTenthG * (VehicleData.OverspeedExcessGs - OverspeedCutStartGs) / 0.1f;
src/Racer.cs:3483-3489  if (ControlledByPlayer) { _overspeedPowerLevel = GlideCap(…); counter *= _overspeedPowerLevel; }
src/SettingsRepair.cs:140  OverspeedCut default "0.2" (on)
```
Wrong side: **the doc**. A non-zero default cuts the player's power multiplier below its resting value whenever the car pulls more than its wheels can explain — that is ARS writing on the player's car beyond nitrous, on a lever the tour calls resting.

### 8-11. Stale anchors (all verified by re-reading the line named)

| Anchor as written | What the line actually holds now | The symbol's real line | Severity |
|---|---|---|---|
| `AGENTS.md:31` — "Use `ARS.IsBetween` (`AutosportRacingSystem.cs:542`)" | `item.SelectedIndex = 0;` inside `RefreshCarPoolItem` | `AutosportRacingSystem.cs:601` `public static bool IsBetween(float value, float min, float max)` | L (+59) |
| `AGENTS/AGENTS-BACKLOG.md:19` — "`SpeedSteerCeiling` (a `static readonly bool`, `Racer.cs:2083`)" | the brake-learner over-sample block (`float ramp = ApexBrakingSpeed(…)`) | `Racer.cs:1657` `static readonly bool SpeedSteerCeiling = false;` | L (−426) |
| `AGENTS/AGENTS-BACKLOG.md:47` — "Code: `Racer.cs:4151` puts the AI driver in the passenger seat (`_isPassengerized`) and `2237` sets `Car.SirenActive = true`" | `4151`: inside `return ARS.Circumradius3D(…)`; `2237`: `if (NextApexNode < 0) return true;` in `HasPassedBrakingTarget` | `Racer.cs:3443` (`SetIntoVehicle(Car, VehicleSeat.Passenger)`); siren at `Racer.cs:1826` | L (−708 / −411) |
| `AGENTS.md:37` — "The route-frame hit test is the shape (`DataStructures.cs:200`)" | `gap -= Math.Sign(gap) * ARS.RouteLengthMeters;` — the route-gap wraparound in `UpdateRouteGap` | the publish line the sentence describes is `DataStructures.cs:196` (`NodeOffset = RivalRacer.Brain.CurrentPerception.CurrentNodeOffset;`); the hit test itself is `Racer.cs:3600 UpdateTrackPosition` | L |

## Register items that are already closed but still sit under the open ladder

Method: 76 slug entries between `## The ladder — open items` (line 7) and the next `##` heading (line 152). **38** carry an explicit closure word (closed / CLOSED / dropped / removed / declined / dismissed / deleted). Of those, **32** are closed, dropped, declined or removed by their own leading state tag:

`crest-gate`, `lane-guidance-gate`, `settings-key-symmetry`, `settings-doc-anchor-drift`, `pace-index-cache`, `slide-limit-raise-removal`, `constant-radius-generator`, `side-slip-scale`, `corner-approach-braking`, `rival-publish-core`, `snap-oversteer`, `yaw-damper`, `two-projection`, `downhill-braking-term`, `brake-learning`, `low-speed-ramp-peak`, `steering-batch-residuals`, `immersive-join-points`, `ceiling-factor-review`, `steer-slip-allowance`, `wheel-contact-count`, `ceiling-factor-range`, `corner-record-dormant-fields`, `surface-path-unread`, `intention-maxspeed-ramp`, `speed-plan-dead-ends`, `amputated-remnants`, `grip-learner`, `update-checker`, `reverse-throttle`, `outside-approach-window`, `lane-crossing-share`.

The remaining 6 are mixed — a closure word in an entry that still carries an open state: `council-minors` ("one live, five done"), `stale-track-path-load` ("**to fix**"), `track-list-index-throw` (BUILT → **closed**, both states in one entry), `weaponized-grid-filter` (DECIDED, pending a roster edit), `gravity-grip` (BUILT, drive owed), `radius-variants` (audited and corrected, no state tag).

Section `## The decision-ready top` (line 222) repeats 12 of them as closed/verified (`ceiling-factor-review`, `slip-axis-scale`, `lane-crossing-share`, `outward-lead-bound`, `intention-maxspeed-ramp`, `corner-record-dormant-fields`, `grip-learner`, `side-slip-scale`, `outside-approach-window`, `crest-gate`, `rival-publish-core`, `radius-variants`), which is why the top reads as a status board rather than a queue.

One owed drive whose code is gone: `docs/TEST-PLAN.md:371-373` (contradiction 6) asks for a `Steer Damping Speed` item and a 10-250 dial; neither the item nor `SteerDampingSpeed` exists in `src/`. Its own revision paragraph (line 367) is what removed them, so the removal was recorded and the checklist was not.

## Comment budget

The project script named in the brief, `scripts/strip_comments.py`, **does not exist** in this tree (`python scripts/strip_comments.py count` → `No such file or directory`; there is no `scripts/` directory, and the only `.py` files are `docs/*.py` and `Util/engine-resistance-curve.py`). Measured instead with a purpose-written counter (string literals masked, `//` to end of line, every `src/**/*.cs`):

```
TOTAL // characters (all src/**/*.cs): 12058
TOTAL excluding Properties/AssemblyInfo.cs: 11075
FILES with comments: 20
```

| File | `//` chars | comment lines |
|---|---|---|
| `src/Racer.cs` | 7816 | 79 |
| `src/AutosportRacingSystem.cs` | 1439 | 14 |
| `src/Properties/AssemblyInfo.cs` (stock template) | 983 | 19 |
| `src/TrackLoader.cs` | 304 | 3 |
| all 16 others | 1516 | 16 |

Against the ~5,000-character allowance this is **≈2.4× over**, and `Racer.cs` alone is **1.56×** the whole budget. The lines counted are prose (`// Lane law: one line in two states…`, `Racer.cs:1-4`), not commented-out code.

## What I could not decide

- **README.md:33-34** — "ScriptHookV 15 Jul 2026 or newer" and "SHVDNE v1.1.0.6 or newer": both are third-party release facts, not repo facts. `libs/` confirms the API dll is 2.11.6 and AGENTS-SHVDN.md:7 confirms the same SHVDNE as `3.9.0.6 (1.1.0.6)`, so the two names agree; whether a *newer* SHVDNE still works is not readable here.
- **AGENTS.md:65** — "the reload key is `Insert` alone, because that ini reads a combo binding as its first token only": the claim is about the install's `ScriptHookVDotNet.ini`, which is outside the repo.
- **AGENTS/AGENTS-BACKLOG.md:238** — "all 14 shipped tracks need a load": `Dist/AutosportRacingSystem/Tracks/` holds **15** `.xml` files (`Around Grapeseed 2.xml` and `Around Grapeseed.xml`, `Chianski.xml` and `Chianski Long.xml` are both present). Whether one is not intended to ship is a data decision, not a code fact, so I did not call it a contradiction.
- **FEATURES.md:53** — "capped … by its own modelled peak slip at the speed it is doing" is a fair abstraction of `MeasureSlipFromMemory` (`Racer.cs:1575-1592`), even though the applied ceiling is `chasingClamp`; I could not call it wrong without rewriting the author's prose, which is out of scope.
- **VERIFY-NOW.md:71 D3** — "`docs/audit/` son 348 KB internos": the directory now measures **1116 KB**. The number is stale, but it is a remark in a decision row, not a claim about the build.
- `docs/RELEASE-NOTES.md` vs `docs/release-0.9.0.md` — AGENTS-WIP.md:101-102 itself flags them as two candidate release notes; deciding which is canonical is the owner's call, so I only record that both exist and that only `RELEASE-NOTES.md` is named in the publish command at WIP:99.
