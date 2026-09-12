# ARS — Smart Tuning (companion to AGENTS.md)

The grid auto-tuner, and everything learned while building it. Code: `src\SmartTuner.cs`. Written 2026-10 after
the feature was verified in game ("now I see dem liveries").

**What it does**: per car, read the car's own livery names → pick a style from keywords → apply a livery → fit
that style's cosmetic parts → paint last. Replaces the old random-index tuning (`AITuningLevel`). Everything is
`internal static`; `SetupRace` enqueues the grid and `OnTick` drains **one car per tick**.

## Why the tick queue (the cost constraint)
- The old `RandomTuning` walks **every** `VehicleMod` slot with `Script.Wait(30)` per part plus a `Wait(100)` —
  roughly a second per car, 10+ seconds for a 12-car grid, all of it *inside* setup, before the player sees
  anything. It also streams ~20-30 mod parts per car (bumpers, spoilers, wheels, liveries), which is a real
  streaming hit under memory pressure.
- The new pass has **no sleeps at all**. Measured in game: 12 cars processed inside the same second, and the
  per-car work is spread over frames instead of landing in one burst.
- `RandomTuning` is **not** dead: `ApplyCarAppearance` still calls it for tuner-discipline cars (see open items).

## The trap: two livery spaces
Enumerating the wrong space returns **no names for every car, silently** - it cost a full test cycle.
- **Modern cars expose liveries as mod slot 48** (`VMT_LIVERY_MOD`): `GET_NUM_VEHICLE_MODS(veh, 48)` →
  `GET_MOD_TEXT_LABEL(veh, 48, i)` → `_GET_LABEL_TEXT` (`0x7B5280EBA9840C72`) → apply with
  `SET_VEHICLE_MOD(veh, 48, i, false)`. This is the space the Comet7 and the drift add-ons use.
- **Older cars (e.g. the Sanchez) use the livery list**: `GET_VEHICLE_LIVERY_COUNT` → `GET_LIVERY_NAME(veh, i)`
  → `_GET_LABEL_TEXT` → apply with `SET_VEHICLE_LIVERY` (SHVDN `veh.Livery`). Used as the fallback.
- `InstallModKit()` first: `GET_MOD_TEXT_LABEL` and `SET_VEHICLE_MOD` both want the kit installed.
- Add-ons usually have **no GXT entries**, so labels resolve to nothing even when liveries exist. Some cars
  genuinely have none at all (reaper, tyrant, t20, osiris, tempesta, turismor).

## Settings
- **`SmartTuning`** (bool, `Menu-Racers.ini`, default **True**) replaced the `AITuningLevel` 0-3 ladder. The old
  key is undeclared in the schema now, so `SettingsRepair.PruneOwnedFiles` drops it and the new key is created
  with its default. **No migration by convention** - a user who had tuning off gets it back on.
- The ladder's level 2/3 behaviour (performance mods, engine boost) was dropped by request; this pass is
  **appearance only** (no Engine/Brakes/Transmission/Suspension slots).
- One log line per car, deliberately self-diagnosing:
  `Smart tune <car>: <Style>, <N> liveries, <name or "none named">`.

## Style taxonomy
- `Keywords`: lowercased substring match, **first hit wins**, so order is deliberate — Beater (rusty, rat look,
  junkyard, primed) → Offroad (camo, safari, naval, forest, desert, expedition) → Racing (rally, racing, race,
  works + sponsor brands) → Stripes → Muscle (flame, scallop, tribal) → Tuner (graffiti, tagged, abstraction,
  geometric, halftone, then the tuner/JP-street brands).
- The tuner/JP block sits **last on purpose** so existing precedence holds: "Rusty Prolaps" stays Beater,
  "Drift Camo Blue" stays Offroad. Tenshun, Kabel, Hyper Function and Jackal were **moved out of Racing** — they
  are tuner brands, not race sponsors.
- `Parts`: cosmetic slots per style (Racing = spoilers/bumpers/skirts/exhaust/frame/roof/plate/trim/windows;
  Stripes = light trim; Muscle = hood/exhaust/frame/hydraulics/seats/wheel; Offroad = roof/frame/bumpers/trunk/
  aerials; Tuner = skirts/trim/seats/dash/dial/speakers/wheel/shifter; Clean = few; Beater = minimal).
  Each slot applies ~**75%** of the time so two cars of the same style do not come out identical.
- Livery names matched by no keyword land in **Clean** rather than being dropped.
- **Dead entry removed**: `pinstripe` could never fire, because `stripe` precedes it and is a substring.

## Colour system — precedence, highest first
1. **Brand rule** (`Brands`): body from the brand's families, accent + rims from its artwork colours. A colour
   the livery **name states** narrows those sets when they contain it (`PreferStated`) — a brand rule alone
   cannot know whether this car is the white variant or the gold one, but "Karin Performance White" and
   "Xero Gas Black" do.
2. **Two colours named**: body = the **first** colour mentioned, accent = the **last** (name order, *not*
   word-list order — the old scan made "Black Pfister White Stripe" read as white).
3. **One colour named**: body from that colour's **whitelist**, accent = that colour, rims neutral.
4. **No colour named, no livery, or a marque**: body neutral or free, accent neutral. (Body, secondary and
   pearlescent were once three independent random draws with rims following the secondary — that is what
   produced the "off" colours.)
- **Marques** (`Marques`, 11): per-livery artwork or a plain badge means there is no brand palette — they stay
  **monochrome** and only pick up colours the name states.
- `Neutrals` = black/white/silver. Rims are neutral **except for brand liveries**, which take a brand colour
  (a trial, matching R\*'s own Sprunk Buffalo); reverting is one line.
- **Pearlescent is always black** (= no pearl tint), by request, for now. Do not "fix" it back to a random draw.
- The palette is **Metallic-only** by request (55 paints, 12 families, every family has at least one member;
  no Matte/Worn/Util/Chrome/Brushed/Pure variants). Keep that rule when adding colours.
- Body **whitelists** are per livery colour, 5-7 families each; "white" deliberately excludes white — contrast is
  structural rather than a veto. Whitelists *are* the rule: never reintroduce "everything except X".
- Brand matching is **whole-word** (`ContainsWord`): a plain `Contains` fires "ron" inside "Chevron", and a
  mis-firing brand rule is invisible — it just quietly paints the wrong car.

## Brand evidence (2026-10 research pass)
- 23 brand rules and 11 marques, from the GTA Wiki, Rockstar's Respray Colours pages, and the fan livery-colour
  database.
- **Canonical pairings are the strongest evidence** — the paints R\* itself ships on a brand-liveried car:
  Sprunk Buffalo = white/white with **green wheels**; Redwood Gauntlet = white with **red wheels**. Only those
  two were ever found, so most body recommendations are artwork-inferred and marked medium: treat them as taste.
- Corrections worth keeping: the interim's "Pisswasser = yellow/black" was **withdrawn** (that is a licence-plate
  option, not a livery — the brand is a red/silver heraldic shield with a gold accent), and "Atomic white/yellow"
  tyre options **do not exist**.
- Two cautions: **no real-world marque folklore** (Ocelot's works livery is red/white/blue, not Jaguar green);
  **manufacturer colour is per-livery, not per-marque** (Ubermacht is orange/yellow/white on a Zion Classic but a
  flat diamond on a Sentinel XS4, and Benefactor has no works livery at all).
- Offline source for label→text **and** model→livery mapping: `D:\Projects\DLCMagic\gtav_rpf\tables\`
  (`modshop_labels.csv` columns: source, variant, modShopLabel, hash, category, display, alt_display, text_source,
  kind, origin, kitName, kit_id, modelNames; `master_new_deduped.txt` is `source - label - hash - display`).
  3822 distinct livery labels. Fan databases freeze before Los Santos Tuners, so newer brands are blank there.

## Enum names are not guessable — reflect the DLL
`GTA.VehicleMod` / `GTA.VehicleColor` member names differ from the native docs and from what you would guess:
`Spoilers` (plural), `Frame` (not Chassis), `Hood` (not Bonnet), `SteeringWheels`, `ColumnShifterLevers`; and
there is no `MetallicYellow` / `MetallicTeal` / `MetallicBrown` (use `MetallicTaxiYellow`, `MetallicSeaGreen`,
`MetallicChocoBrown`). Load the deployed `ScriptHookVDotNet2.dll` with PowerShell and dump
`[Enum]::GetNames(...)` — it saves a build cycle every time.

## Open items
- **Unnamed-livery fallback — approved, unbuilt**: a car with liveries but no GXT names gets **no** livery at all
  today. Should apply a random one plus generic parts.
- **Colour vocabulary extension — parked**: navy, maroon, burgundy, crimson, olive, moss, teal, aqua, bronze,
  champagne, ivory, sand currently read as "no colour named". Cheap route: map them onto the existing families
  (navy→blue, maroon/burgundy/crimson→red, olive/moss→green, teal/aqua→blue, bronze→gold, champagne/ivory/sand→
  cream) so the twelve whitelists get reused.
- **Tuner/JP-street brands have no colour evidence** (their research agents were stopped before returning):
  Tenshun, Kabel, Wasabi, Dense, ERIS, Haiso, Higashi, Minami, Nishi, Kita, Missile, NARC, Ragga Rum, Stance
  Andreas, Jackal — all on the generic colour paths.
- **Karin's body pool is the whole blue family** (10 paints), so "deep blue" is likely but not guaranteed. A
  separate deep-blue family is the clean fix if it comes out too bright in the field.
- **`RandomTuning` still drives the tuner-discipline path** in `ApplyCarAppearance` (left alone by choice), so
  tuner-tagged cars get that random treatment on top of this pass.
- The **~75% parts fill** is a guess at how complete a build should look; tune it after seeing a few fields.
- Marque cars have only been seen as code, not in the field: the monochrome look is unverified visually.
