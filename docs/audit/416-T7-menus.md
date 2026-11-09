# AUDIT 416-T7 — the LemonUI menu and every row it owns

**Surface**: `src/AutosportRacingSystem.cs` — `InitializeMenu` (`:685-1390`), `SaveRacerSetting` (`:1391-1394`), `AddDebugCheckbox` (`:1395-1406`), `SaveDevToggle` (`:1407-1410`), `HookListTextPicker` (`:1412-1427`), `RefreshTrackList` (`:1624-1663`), `RefreshPowerControls` (`:1665-1700`), `ApplyPaceModeUI` (`:1702-1712`), `OnKeyDown` / `IsMenuModifierPressed` / `IsMenuHotkeyPressed` (`:1834-1852`), `LoadSettings` (`:3344-3465`), the `Options` enum (`:49-52`) with its `DebugToggles` table (`:185-204`) and every `DebugToggles[...]` read; `src/MenuSettings.cs`; `src/SettingsRepair.cs` (the schema at `:113-179`).
**Date**: 2026-10-30.
**Build**: the tree's counter reads **1380** (`build/build-number.txt`); the brief said 1379, so this is the tree one increment later than the brief — said plainly rather than guessing which is current.
**Method**: read-only. No build, no deploy, no game, no git. Third-party behaviour was read, not assumed: `libs/LemonUI.SHVDN2.dll` decompiled with `ilspycmd` for `NativeListItem<T>.SelectedIndex`, `NativeMenu.Process` and `NativeItem.Enabled`.
**Verdict**: **no blocker.** Every persisted row has all four sites (constructor, change, load, store) and every one of them agrees on the file it writes — the failures in this sweep are at the row's *contract*: three tooltips tell the user something the law below them does not do, and one persisted key is still carried by a row that cannot fire.

---

## Three library facts the table depends on (verified, not assumed)

1. **`SelectedIndex` fires `ItemChanged` only when the index actually changes**, and it throws when the list is empty — `NativeListItem<T>.SelectedIndex` setter: `if (Items.Count == 0) throw new InvalidOperationException("There are no available items."); … if (index != value) { index = value; TriggerEvent(value, Direction.Unknown); UpdateIndex(); }`. So a row that seeds to the index it already holds (`0`) writes nothing; every list row that paints a non-zero index at build time re-writes its own value once, harmlessly.
2. **The `Items` setter does not reset `index`** (`items = value; UpdateIndex();`). `RefreshCarPoolItem` (`:541-543`) and `RefreshTrackList` (`:1641-1642`) both set the index to 0 *before* replacing the list because of this.
3. **A disabled item cannot raise anything**: `NativeMenu` gates activation on `if (SelectedItem != null && SelectedItem.Enabled)` and skips disabled rows in navigation; `NativeMenu.Process()` also calls `Draw()`, which is why nothing is painted before `_loaded` gates the tick at `:1867-1871`.

**Ordering fact the whole review rests on**: `LoadSettings()` (`:372`) runs before `InitializeMenu()` (`:376`) in the constructor, so every `NativeCheckboxItem`/`NativeListItem` that is constructed from a static painted the *loaded* value. That is why prior finding A is closed rather than merely masked.

---

## Every persisted row: constructor / change / load / store

`load` names the site that puts the stored value into the static (or into `SelectedIndex` when the row is the only reader). `store` is the schema line that declares the key.

### `Menu-Race.ini`

| Row | ctor | change (writes) | load | store |
|---|---|---|---|---|
| Select Track | `:703` (empty), items `:1637-1642` | `:704-711` → `Set("Track")` `:709` | `:687`, index `:1657` | `SettingsRepair.cs:118` |
| Laps | `:715` `OptionsFor("Laps")` | `:716` | `:717-718` | `:119` |
| Reverse Route | `:722` (via `AddDebugCheckbox`, store arg) | `:1401` `store.Set(option…)` | `:3460` | `:125` |
| Target Grid Size | `:733` `GridSizeLabels()` | `:734-739` | `:740-741` | `:120` |
| Grid PI Mode | `:751` `EnumLabels<PaceMode>()` | `:752-757` | `:3441`, index `:802` | `:121` |
| PI Offset | `:761-766` (loop) | `:767-775` | `:3442`, index `:1695` | `:122` |
| PI Target | `:779` (empty) | `:780-788` | `:1679-1681`, index `:1691-1692` | `:123` |
| Max PI Deviation | `:791` `OptionsFor` | `:792-796` | `:3443`, index `:797` | `:124` |

### `Menu-Settings.ini`

| Row | ctor | change | load | store |
|---|---|---|---|---|
| Grid Sorting | `:886` | `:887` | `:888` (row), `:2505` (runtime) | `:129` |
| Car Roster | `:896-897` | `:898-906` | `:893-895`, `:910`, refresh `:526-549` | `:130` |
| Repeat Cars To Fill Grid | `:916` | `:917-921` | `:3398` | `:139` |
| Auto Styling | `:924` | `:925-929` | `:3400` | `:135` |
| Menyoo Skins | `:932` | `:933-937` | `:3394` | `:138` |
| Corner Speed Offset (mph) | `:962` | `:963-967` | `:3401`, index `:968` | `:133` |
| Road Curvature Offset (mph) | `:972` | `:973-977` | `:3402`, index `:978` | `:134` |
| Brake Learning | `:982` | `:983-987` | `:3409` | `:141` |
| Max Crest Lift (g) | `:990` | `:991-995` | `:3421`, index `:996` | `:153` |
| Gs Lane Throttle Floor | `:1000` | `:1001-1005` | `:3413` | `:155` |
| Open Up on Throttle | `:1008` | `:1009-1013` | `:3414`, index `:1014` | `:156` |
| Slope Loss (%) | `:1018` | `:1019-1023` | `:3431`, index `:1024` | `:154` |
| Overspeed Correction | `:1028` | `:1029-1033` | `:3399`, index `:1034` | `:140` |
| TCS | `:1038` | `:1039-1043` | `:3415` | `:142` |
| TCS Wheelspin Limit (%) | `:1046` | `:1047-1051` | `:3417`, index `:1052` | `:145` |
| TCS Cut Softness (%) | `:1056` | `:1057-1061` | `:3418`, index `:1062` | `:146` |
| ABS | `:1066` | `:1067-1071` | `:3416` | `:143` |
| Steer Damping | `:1074` | `:1075-1079` | `:3403`, index `:1080` | `:149` |
| Damper Countersteer | `:1084` | `:1085-1089` | `:3404`, index `:1090` | `:150` |
| Peak Slip Share | `:1094` | `:1095-1099` | `:3405`, index `:1100` | `:151` |
| Slip Cut Floor | `:1104` | `:1106-1110` | `:3406`, index `:1111` | `:152` |
| Timeout (s) | `:1122` | `:1123` | index `:1124`, runtime `:2215` | `:131` |
| Racer Autofix | `:1128` | `:1129` | `:3386-3392`, index `:1130` | `:132` |
| Hide DNFs | `:1134` | `:1135-1139` | `:3419` | `:147` |
| Rubberbanding (%) | `:1142` | `:1143-1147` | `:3432`, index `:1148` | `:157` |
| Rubberband Mode | `:1152` | `:1153-1157` | `:3433`, index `:1158` | `:158` |
| No Collision | `:1162` | `:1163-1168` | `:3420` | `:148` |
| Nitrous | `:1171` | `:1172-1176` | `:3397` | `:136` |
| AI Nitrous Power (%) | `:1179` | `:1183-1188` | `:3457`, index `:1181-1182` | `:165` |
| Racing Tips | `:1201` | `:1202-1206` | `:3436`, index `:1207` | `:137` |
| Show Track & Grid Rows | `:1217` (**literal `false`**) | `:1219-1233` (**cannot fire**) | **none** — `:3435` hard-sets `false` | `:159` |
| Hotkeys | `:1251` | `:1252-1256` | `:3452` | `:160` |
| Menu Modifier | `:1259` | `:1260-1265` | `:3453`, index `:1266` | `:161` |
| Menu Key | `:1270` | `:1271-1276` | `:3454`, index `:1277` | `:162` |
| Log Level | `:1298` | `:1299-1303` | `:3455`, index `:1304` | `:163` |
| Corner Ceiling | `:1308` | `:1311-1318` | `:3456`, index `:1309-1310` | `:164` |

### `Menu-Debug.ini` — 16 rows, one shape

All are built by `AddDebugCheckbox`: ctor `:1397` (`DebugToggles[option]`), change `:1398-1404` → `:1409` `DebugMenuStore.Set`, load `:3458-3459`. Schema `:175-176` (loop over the table). Rows: Show Pedal Trail `:1287`, Show Track Analysis `:1288`, Show Rivalry `:1289`, Show Bumps `:1290`, Show Wheel Slip `:1291`, Show Front Slip `:1292`, Lock Lane Centre `:1293`, Show Gs Projection `:1294`, Show G Line `:1295`, Show Route Radius `:1296`, Online Downforce Curve `:1192`, Show Corner Checkpoints `:1211`, Show Edge Chevrons `:1212`, Show Leaderboard `:1213`, Show Driver Names `:1214`, Show Rival Lap Times `:1215`. The **17th** `Options` member, `ReverseRoute`, has its row in `Race` — see F6.

### `Menu-Development.ini`

| Row | ctor | change | load | store |
|---|---|---|---|---|
| Steer Trace | `:1337` | `:1338-1342` | `:3411` | `:170` |
| Brake Learn Notify | `:1345` | `:1346-1350` | `:3410` | `:169` |
| Slip From Memory | `:1353` | `:1354-1358` | `:3412` | `:171` |

**Rows with a domain the row cannot produce**: none. Every `OptionsFor("…")` key used at `:715, :791, :962, :972, :990, :1008, :1018, :1028, :1046, :1056, :1074, :1084, :1094, :1104` is declared with a non-null `Domain`, so no row is built from an empty array; and the index-cast rows (`PaceMode`, `GridSort`, `AutofixMode`, `TipFrequency`, `RubberbandMode`, `LogImportance`) are built by `EnumLabels<T>()`/`Enum.GetNames`, whose order is the enum's own, so `(T)args.Index` is sound.
**Rows writing the wrong file**: none. All 63 rows (8 race, 36 settings, 16 debug, 3 Development) write the file whose `LoadSettings` line reads them back, and every one of the 63 schema keys has exactly one row. The four stores are used consistently (`RaceMenuStore` for the eight race keys, `SettingsMenuStore` for the 36 settings keys via `SaveRacerSetting`/direct `Set`, `DebugMenuStore` for the 17 debug keys — 16 rows plus `ReverseRoute`, see F6 — and `DevelopmentMenuStore` for the three Development toggles).
**Dead handlers**: one — `Show Track & Grid Rows` (F2).

---

## Findings

Severity: **blocker** = must change before the tag; **release-note** = true as shipped but it will mislead or needs a decision; **file** = a defect in a tracked file. **Blockers: none.** Ranked by what they cost a user.

### F1 — release-note — the ABS row's description says the row does not work, and describes the opposite law

`src/AutosportRacingSystem.cs:1066`

```csharp
NativeCheckboxItem absItem = new NativeCheckboxItem("ABS", "GTA forces native ABS on non-player driven cars. I'm working on undoing that.", AbsEnabled);
```

The row is live and its law is the mod's *own* ABS taper, not the removal of GTA's: the static is read from the store at `:3416` (`AbsEnabled = SettingsMenuStore.GetBool("AbsEnabled", AbsEnabled);`), the checkbox is seeded with it and is **not** disabled, and `Racer.cs:2579-2589` turns it into a brake cap that is composed into the pedal at `Racer.cs:2629`:

```csharp
float AbsCapLevel()
{
    if (!ARS.AbsEnabled) return 1f;
    …
    if (ARS.AnyWheelRotationBelow(Car, AbsLockRadPerSecond)) return AbsBrakeFloor;
```

So ticking the box *adds* ABS-like brake release; the tooltip tells the reader the feature is unfinished and that ticking it removes ABS. Under house rule 5/6 an owner who reads the text can only conclude the row is a placeholder — while it silently cuts AI brake pedal. `docs/OPEN-ITEMS.md:33` repeats the stale state ("the taper is behind a forced-off, greyed `AbsEnabled`"), which is no longer true of this tree.

**Falsify**: `grep -n "AbsEnabled" src/*.cs` → reads at `:3416`, law at `Racer.cs:2581`; there is no `Enabled = false` anywhere near `:1066-1072`. In game: set `AbsEnabled = True` in `Menu-Settings.ini`, drive an AI car to 60 mph on tarmac and brake hard — `Log.log` gains the `ABS <name>: locked=… cap=…` line from `Racer.cs:2574`, i.e. the row is doing something, contrary to its own text.

### F2 — release-note — `Show Track & Grid Rows` is a row that cannot fire, writing a key the load throws away (prior C, still true)

`src/AutosportRacingSystem.cs:1217-1219`

```csharp
NativeCheckboxItem stagedSpawnsItem = new NativeCheckboxItem("Show Track & Grid Rows", "Load Track and Build Grid in the Race menu. Off and not changeable.", false);
stagedSpawnsItem.Enabled = false;
stagedSpawnsItem.CheckboxChanged += (sender, args) =>
```

Four separate facts, all current: (a) the box is constructed with a literal `false`, not with `StagedSpawns`; (b) it is disabled, and `NativeMenu` only activates `if (SelectedItem != null && SelectedItem.Enabled)`, so the handler at `:1219-1233` — including the add/remove of `instanceTrackItem`/`instanceGridItem` — is unreachable; (c) the load overwrites the key regardless, `:3435` `StagedSpawns = false;`, so nothing ever reads the stored value; (d) the unconditional removal at `:1322-1326` (`if (!StagedSpawns) { _raceMenu.Remove(instanceTrackItem); … }`) is therefore always taken.

```csharp
            if (!StagedSpawns)
            {
                _raceMenu.Remove(instanceTrackItem);
                _raceMenu.Remove(instanceGridItem);
            }
```

The parking is deliberate — the description says *"Off and not changeable"* — so the rows being absent is by design; what is not by design is the surviving plumbing: `SettingsRepair.cs:159` keeps the key in the file forever, `Migrate("StagedSpawns", …)` at `:3408` writes a legacy `True` into a file whose value is discarded three lines later, and the handler is one of only two unreachable handlers in the menu. House rule 8 (a value written and never read) and rule 1 (one writer).

**Falsify**: put `StagedSpawns = True` in `Menu-DevSettings.ini`, delete the key from `Menu-Settings.ini`, load: the file gains `StagedSpawns = True` (from `:3408`), the Interface menu still shows the row greyed and unchecked, and the Race menu has no Load Track / Build Grid. `grep -n "StagedSpawns" src/AutosportRacingSystem.cs` → `:3435` is the only read that matters and it is a constant.

### F3 — release-note — `No Collision` promises the whole field passes through; the actuator ghosts one pair per car

`src/AutosportRacingSystem.cs:1162`

```csharp
NativeCheckboxItem noCollisionItem = new NativeCheckboxItem("No Collision", "The racers do not detect each other and pass through each other. World collision is untouched.", NoCollision);
```

The first and third sentences are true (`Racer.cs:4655-4658` clears the rival brains; only entity-entity pairs are touched). The second is not: the handler at `:1167` calls `ApplyNoCollision`, which stores exactly one partner per car — its **nearest** — because the engine keeps one `m_pNoCollisionEntity` per entity:

```csharp
                if (nearest != null) Function.Call(Hash.SET_ENTITY_NO_COLLISION_ENTITY, a.Car, nearest.Car, !enable);
```

`docs/TEST-PLAN.md:137-139` states the residual outright: *"a pair is ghosted only when at least one of the two stored the other, so a car whose sole close rival points elsewhere can still be touched"*. A pack of three where the middle car is everyone's nearest works; a fourth car alongside does not, and the row's text gives the user no way to know that contact is expected.

**Falsify**: set the grid to 8, tick No Collision, and drive into the car that is second-nearest: you are stopped by it (`SET_ENTITY_NO_COLLISION_ENTITY` was only ever called for the nearest pair), while the row reads "pass through each other".

### F4 — file — the Menu Modifier/Key rows offer Left and Right Shift (Control, Alt) as separate keys; the law collapses them

`src/AutosportRacingSystem.cs:3500-3503` — the labels the row shows and the store persists:

```csharp
            if (key == Keys.LShiftKey) return "Left Shift (160)";
            if (key == Keys.RShiftKey) return "Right Shift (161)";
            if (key == Keys.LControlKey) return "Left Control (162)";
            if (key == Keys.RControlKey) return "Right Control (163)";
```

`src/AutosportRacingSystem.cs:1843-1844` — what is actually consulted:

```csharp
            if (MenuModifierKey == Keys.ShiftKey || MenuModifierKey == Keys.LShiftKey || MenuModifierKey == Keys.RShiftKey) return e.Shift;
            if (MenuModifierKey == Keys.ControlKey || MenuModifierKey == Keys.LControlKey || MenuModifierKey == Keys.RControlKey) return e.Control;
```

The row offers six distinct choices (`160`, `161`, `162`, `163`, `164`, `165`, plus the agnostic `16/17/18`), persists which one was picked at `:1264`/`:1275`, and the hotkey then answers to *either* side. Rule 5: the label promises a specific key the code does not use, and the stored value cannot be honoured.

**Falsify**: Interface → Controls → Menu Modifier = `Right Shift (161)`; reload and confirm `Menu-Settings.ini` holds `MenuModifierKey = 161`; press **Left** Shift + E and the menu opens.

### F5 — file — the `CrestEffect` → `MaxCrestLift` migration cannot run: the seeding getter writes the key first, and the prune has already deleted its source

`src/AutosportRacingSystem.cs:3421` then `:3423-3429`

```csharp
            MaxCrestLift = SettingsMenuStore.GetFloat("MaxCrestLift", MaxCrestLift);
…
            int legacyCrestEffect = retiredPaceSettings.GetValue<int>("MENU", "CrestEffect", -1);
            if (legacyCrestEffect > 0)
            {
                float legacyCap = legacyCrestEffect * 0.01f * 0.2f;
                string rounded = Math.Round(legacyCap, 1).ToString("0.0", CultureInfo.InvariantCulture);
                SettingsMenuStore.Migrate("MaxCrestLift", rounded);
```

`GetFloat` is a seeding getter (`MenuSettings.cs:56-61` → `:89-94`): on a file without `MaxCrestLift` it writes `0.4` and saves, and `Migrate` (`MenuSettings.cs:114-118`) returns as soon as `Get(key, null) != null`. The upgrade the migration exists for therefore always lands on `0.4`. Two further gaps in the same block: `retiredPaceSettings` (`:3354`) is a snapshot of `Menu-Settings.ini` taken *before* `PruneOwnedFiles()` (`:3360`) drops `CrestEffect` as an undeclared key, so the legacy value is deleted from the user's file and then discarded; and `if (legacyCrestEffect > 0)` has no branch for the old "off" `0`.

**Falsify**: in `Menu-Settings.ini` delete `MaxCrestLift`, add `CrestEffect = 50`, load: the file ends with `MaxCrestLift = 0.4` (not `0.1`), no `Settings repair: … MaxCrestLift` line appears, and `CrestEffect` is gone. `grep -n "MaxCrestLift" src/AutosportRacingSystem.cs` shows the read at `:3421` above the migrate at `:3428`.

### F6 — file — `ReverseRoute` still has a second home in `Menu-Debug.ini`, read and then discarded (documented as deliberate; recorded so a cleanup does not delete half of it)

`src/AutosportRacingSystem.cs:3458-3460`

```csharp
            foreach (Options option in DebugToggles.Keys.ToArray())
                DebugToggles[option] = DebugMenuStore.GetBool(option.ToString(), DebugToggles[option]);
            DebugToggles[Options.ReverseRoute] = RaceMenuStore.GetBool("ReverseRoute", DebugToggles[Options.ReverseRoute]);
```

`SettingsRepair.cs:124` declares `ReverseRoute` in `Menu-Race.ini` (where the row's store argument at `:722` writes it) **and** `SettingsRepair.cs:175-176` declares it again in `Menu-Debug.ini` because the debug schema is generated from the whole `Options` table:

```csharp
            FileSpec debug = Owned("Menu-Debug.ini");
            foreach (Options option in ARS.DebugToggles.Keys)
                debug.Specs.Add(new KeySpec(option.ToString(), ARS.DebugToggles[option].ToString(), Kind.Bool));
```

So `Menu-Debug.ini` carries a key that `CompleteOwnedKeys` seeds on every load, that the loop at `:3458` reads, and whose value the very next line throws away. Rule 1 (one writer) and rule 8 (a value read for nothing) both fail; the checkbox itself is honest. This is the same defect as audit 06 S9 and is called deliberate in `AGENTS/AGENTS-TECHNOTES.md`; it is restated here because it is still the current state of the tree, not because it is new.

**Falsify**: turn Reverse Route on (writes `Menu-Race.ini`), reload, then `cat Settings/Menu-Debug.ini` — `ReverseRoute` still reads `False`, and editing it to `True` changes nothing in game.

---

## Prior findings, re-checked

| # | Verdict today | Where |
|---|---|---|
| **A** — a persisted checkbox built without its stored value paints every saved toggle as OFF, and the first click writes false | **CLOSED.** `LoadSettings()` at `:372` precedes `InitializeMenu()` at `:376`, and all fourteen persisted checkboxes are constructed from their loaded static: `:916` `ForceFillGrid` (`:3398`), `:924` `SmartTuning` (`:3400`), `:932` `UseMenyooSkins` (`:3394`), `:982` `BrakeLearning` (`:3409`), `:1000` `GsLaneThrottleFloor` (`:3413`), `:1038` `TcsEnabled` (`:3415`), `:1066` `AbsEnabled` (`:3416`), `:1134` `DNFUnderTrack` (`:3419`), `:1162` `NoCollision` (`:3420`), `:1171` `NitrousEnabled` (`:3397`), `:1251` `HotkeysEnabled` (`:3452`), plus the three Development toggles `:1337/:1345/:1353` and the sixteen debug boxes via `DebugToggles[option]` at `:1397` loaded at `:3458-3459`. The one remaining literal `false` is the disabled Staged Spawns box (F2). And because the setter fires only on an index/value change, no row re-writes a value it was not showing. |
| **B** — the `SpeedSteerCeiling` row is a zombie: its law is parked, so the row changes nothing | **CLOSED as stated (the row is gone).** There is no `SpeedSteerCeiling` row in `InitializeMenu`; the successor row is `Peak Slip Share` `:1094`, key `SteerCeilingFactor`, and its law is live: `Racer.cs:1589` `float capShare = Math.Max(0.05f, ARS.SteerCeilingFactor) * SlowCornerSlipShare(fwdSpeed);` and `:1608`, with `:3277` on the theory path. The parked constant `SpeedSteerCeiling` (`Racer.cs:1657`, `static readonly bool … = false`) now gates only the *measured-slip* ceiling sub-branch (`:1646`, `:1652`), which no menu row owns. **Caveat worth stating**: the shipped row's description still describes the live `capShare` law, so the text is right today — but if the row's meaning is ever re-pointed at the parked branch, the description becomes a lie. That is a note, not a finding. |
| **C** — `Staged Spawns` ships disabled but keeps its schema entry and its handler | **TRUE today**, at `:1217` (literal `false`), `:1218` (`Enabled = false`), `:1219-1233` (handler, unreachable), `:1222` (`SaveRacerSetting("StagedSpawns", …)`), `:1322-1326` (removal), `:3435` (`StagedSpawns = false;`), `SettingsRepair.cs:159` (schema entry). See F2. |
| **D** — `AbsEnabled` is forced off and greyed because the engine re-grants ABS to every AI car | **CLOSED — both halves are false today.** There is no forcing line: `:3416` `AbsEnabled = SettingsMenuStore.GetBool("AbsEnabled", AbsEnabled);` is a real read; there is no `Enabled = false` on the ABS item (`:1066-1072`); and the static drives a live brake cap (`Racer.cs:2581`, composed at `Racer.cs:2629`). What survives from D is only its *wording*: the tooltip still tells the user the row is unfinished (F1), and `docs/OPEN-ITEMS.md:33` still repeats "forced-off, greyed". |

---

## Rows whose sites are incomplete

- **Show Track & Grid Rows** (`:1217`) — ctor uses a literal, handler unreachable, **no load read at all** (`:3435`). (F2)
- **Reverse Route** — all four sites exist and point at the Race store, but a *fifth* site, `Menu-Debug.ini`, is declared, seeded and read uselessly (`SettingsRepair.cs:175-176`, `:3458-3460`). (F6)
- **Max Crest Lift** — its only migration site (`:3423-3429`) is shadowed by its own seeding read (`:3421`), so the legacy path is dead. (F5)
- **PI Offset / PI Target** — no index at build: both rows are left at index 0 (PI Offset shows `-10`, PI Target shows an empty list) and are only corrected later by `RefreshPowerControls` at `:1691-1695`, which runs from the background load. Not a defect in practice — `NativeMenu.Process()` is gated behind `_loaded` at `:1867-1871`, so the submenu cannot be opened or drawn in the window where it would show `-10` — but it is the only pair of rows whose painted value does not come from their own load site.

---

## What I could not decide

- **`No Collision`'s teardown.** `ApplyNoCollision(false)` (`:1813`) re-pairs each car with whatever is nearest *now* and passes `!enable` = `true` to the native, while the pair that was actually ghosted was chosen at toggle time. Whether that clears the ghosted pair or merely re-arms a self-clearing one on a different partner depends on the engine's third argument, which `docs/TEST-PLAN.md:130-136` describes but I cannot test read-only. Filed only as the honest-scope problem (F3), not as a claim about the reset.
- **Whether the legacy `Hotkeys` value can be a chord string.** `SettingsMenuStore.Migrate("Hotkeys", legacyDevConfig.GetValue<string>("GENERAL","Hotkeys", null))` (`:3447`) then `GetBool("Hotkeys", …)` (`:3452`) would disagree if the retired `DevConfig.ini` held something other than `True`/`False`; I found no copy of that retired file to check, so I make no claim.
- **The shop rows.** The brief mentions "shop/leaderboard toggle rows"; there is no shop menu, row or key anywhere in `src/` (`grep -rn "Shop"` → nothing). The leaderboard rows are `:1213` (Show Leaderboard) and `:1214` (Show Driver Names), both live, both read at `:2288` and `:2311`/`:2334`. I reviewed the leaderboard rows; I could not review a shop that does not exist.
- **`Grid Size` capped below the stored value.** `:1476` `int gridSize = Math.Min(_intendedOpponents, Math.Max(0, GridSlotsAvailable - 1));` silently caps the row's value when the track is short of slots and only logs it. That is a race-builder behaviour rather than a menu row defect, and it is outside this scope, so I did not file it.
