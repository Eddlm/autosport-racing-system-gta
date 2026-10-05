# GTA V vehicle flags — the engine's own sets

**Read this when** the request, the code or the bug touches: `strModelFlags`, `strHandlingFlags`, `strAdvancedFlags`, `strDamageFlags`, handling flags, `HF_` / `MF_` / `FLAG_` / `CF_` / `DF_` prefixes, handling.meta flag hex, "what does this flag actually do", `CAdvancedData` / `CCarHandlingData`, or a car that behaves unlike its handling values suggest.

**Provenance**: every row below was extracted from the leaked engine tree at `E:\GTA\GTAVSP\GTAV Source\src\dev_ng\game` — the enum's own header comment where one exists, plus a consumer site read in the code. Community flag lists are deliberately *not* the source: several of them paraphrase, and at least one (the off-road gravity pair) describes the *consequence* of a flag as if it were a second effect. Where a community list and this file disagree, this file wins, because the file cites the code.
**The tree is huge — never recursive-grep all of `src`**: it exceeds the 30 s tool timeout. The flag headers live under `...\game\Vehicles\`.

**What ARS reads today, measured**: exactly one flag call — `VehicleMemory.GetHandlingFlags` (`VehicleMemory.cs:35`), read once per racer at `Racer.cs:469-473` and used only for the two off-road gravity bits. **No model-flag, advanced-flag, damage-flag or special-flight-flag field is read anywhere in ARS** (the one model-level property it does read, `FLAG_IS_ELECTRIC`, comes through a native, not through a flag field). So most of what follows is ARS-blind by default; the relevance section at the end says which ones matter.

## `strHandlingFlags` — `VehHandlingFlags` (`HF_*`), `handlingMgr.h:84-127`

All 32 bits are defined with no gaps: **31 functional flags plus `HF_LAST_AVAILABLE_FLAG`, a sentinel that nothing consumes**.

**Parse path** (verified): the meta is loaded by `CHandlingDataMgr::LoadHandlingMetaData` (`handlingMgr.cpp:2026`), which fills members through the schema in `...\game\Vehicles\Metadata\HandlingInfo.psc:66` and then calls `ConvertToGameUnits` (`:1985`) on the pool. The actual parse is one line — **`sscanf(m_strHandlingFlags.TryGetCStr(), "%x", &hFlags);` at `handlingMgr.cpp:1699`** — landing in **`u32 hFlags`** (`handlingMgr.h:972`). A sideloaded or supplementary meta takes the second entry point, `AppendHandlingMetaData` (`:2037`, ending at `:2049`), which is the path this project's `Sideload\0_vsl-handling\handling.meta` uses.
**A malformed value has no error path at all**: the `sscanf` return is discarded, the string is never validated or logged, and `hFlags` is written in only two places (the constructor at `:1332` and `:1699`) — so a bad value leaves **`hFlags = 0`, i.e. silently flagless**, never an error.

| bit | engine name | header comment | primary consumer | what it does |
|---|---|---|---|---|
| `0x00000001` | `HF_SMOOTHED_COMPRESSION` | — | `wheel.cpp:3391-3395` | Wheel compression is interpolated across the integration step instead of used raw |
| `0x00000002` | `HF_REDUCED_MOD_MASS` | — | `VehicleModelInfo.cpp:6655-6657` | Mod collision bones use the standard mod mass/inertia instead of the bone group's |
| `0x00000004` | `HF_HAS_KERS` | `// Has a KERS energy recovery system` | `Transmission.cpp:1385` | KERS boost, gated on `NetworkInterface::IsGameInProgress()` ("Only supposed to be used in MP") |
| `0x00000008` | `HF_HAS_RALLY_TYRES` | — | `wheel.cpp:3167-3191` | Takes the alternative traction branch — *"grip increases with slip angle"*, min/max curve terms swapped |
| `0x00000010` | `HF_NO_HANDBRAKE` | — | `vehicleDamage.cpp:4341`, `Bike.cpp:2085` | `m_bHasHandBrake` false: no handbrake force, and the camera/input treat it as absent |
| `0x00000020` | `HF_STEER_REARWHEELS` | — | `Automobile.cpp:660-663`, `:3720` | Front wheels stop steering, rear wheels steer; the speed-based steer auto-centre is disabled |
| `0x00000040` | `HF_HANDBRAKE_REARWHEELSTEER` | `// Steer the rear wheels when the handbrake is on` | `Automobile.cpp:3512-3515` | Rear wheels form a circular formation via the second steer angle |
| `0x00000080` | `HF_STEER_ALL_WHEELS` | — | `Automobile.cpp:665-684` | Rear wheels steer; with `HF_STEER_REARWHEELS` the fronts steer too (unless tracks) |
| `0x00000100` | `HF_FREEWHEEL_NO_GAS` | — | `wheel.cpp:3799-3801` | Off-gas wheels use the free-wheel friction coefficient, with a low-speed boost so the car still stops |
| `0x00000200` | `HF_NO_REVERSE` | — | `Transmission.cpp:352-353` | Gear forced to 1 regardless of throttle or rev ratio; reverse cannot be engaged |
| `0x00000400` | `HF_REDUCED_RIGHTING_FORCE` | — | `Automobile.cpp:1135-1139` | Aftertouch/righting scale cut to `sf_ReducedAfterTouchScale` (0.45) |
| `0x00000800` | `HF_STEER_NO_WHEELS` | — | `Automobile.cpp:670-674` | Neither axle is flagged as steering |
| `0x00001000` | `HF_CVT` | — | `Transmission.cpp:271-275` | Drive force comes from the CVT branch |
| `0x00002000` | `HF_ALT_EXT_WHEEL_BOUNDS_BEH` | `// Alternative extra wheel bound behavior. Offset extra wheel bounds forward so they act as bumpers and enable all collisions with them.` | `wheel.cpp:2656-2666` | Extra wheel bounds offset forward (0.4 wheel radius, 0.6 with the shrink bit) and no longer deactivate wheel impacts |
| `0x00004000` | `HF_DONT_RAISE_BOUNDS_AT_SPEED` | `// some vehicles bounds dont respond well to be raised up too far, so this turns off the extra bound raising at speed.` | `Automobile.cpp:3020-3022` | Skips ground-clearance bound raising at speed |
| `0x00008000` | `HF_EXT_WHEEL_BOUNDS_COL` | `// Extra wheel bounds collide with other wheels.` | `wheel.cpp:2556-2560`, `Vehicle.cpp:31179` | Monster-truck style extra wheel bounds get real collisions; also disables ped IK/climbing on them |
| `0x00010000` | `HF_LESS_SNOW_SINK` | — | `Vehicle.cpp:20537` | Selects the less-sink second-surface config (inside `#if HACK_GTA4_BOUND_GEOM_SECOND_SURFACE`) |
| `0x00020000` | `HF_TYRES_CAN_CLIP` | `// Tyres can clip into the ground when bottoming out.` | `wheel.cpp:1359-1367` | The compression cap becomes `susLength + (wheelRadius − rimRadius) × sfAllowableTyreClipAmount` — the tyre sinks into the ground when bottoming out |
| `0x00040000` | `HF_REDUCED_DRIVE_OVER_DAMAGE` | `// Don't explode vehicles when driving over them` | `vehicleDamage.cpp:8012-8014` | Vehicle-on-vehicle collisions are flagged as monster-truck damage, so driving over a car does not damage it |
| `0x00080000` | `HF_ALT_EXT_WHEEL_BOUNDS_SHRINK` | — | `wheel.cpp:2620-2629` | Extra wheel bound width multiplier 0.1 instead of 1.1, pulled in by 0.6 × wheel radius |
| `0x00100000` | `HF_OFFROAD_ABILITIES` | `// Extra gravity` | `Vehicle.cpp:31304-31306` | `fGravityMult = 1.1`, applied at `:31323` |
| `0x00200000` | `HF_OFFROAD_ABILITIES_X2` | `// Even more gravity` | `Vehicle.cpp:31308-31310` | `fGravityMult = 1.2`; also auto-levelling in air (`Automobile.cpp:3657`) and a 1.5× aftertouch (`:1141`) |
| `0x00400000` | `HF_TYRES_RAISE_SIDE_IMPACT_THRESHOLD` | `// this vehicle may be used for driving over vehicles to raise the side impact threshold so we dont miss contacts when driving over vehicles.` | `wheel.cpp:1370-1373` | Sets `bUseHigherSideImpactThreshold` in the wheel bound setup |
| `0x00800000` | `HF_OFFROAD_INCREASED_GRAVITY_NO_FOLIAGE_DRAG` | — | `Vehicle.cpp:31308-31311`, `:28529` | The same ×1.2 gravity branch, plus an early return that skips foliage drag (still computed for audio) |
| `0x01000000` | `HF_ENABLE_LEAN` | — | `VehicleFactory.cpp:3045-3048` | Adds a `CVehicleLeanHelper` gadget to the vehicle |
| `0x02000000` | `HF_FORCE_NO_TC_OR_SC` | — | `Bike.cpp:2091-2097` | While not braking, the free `WF_CHEAT_TC` / `WF_CHEAT_SC` wheel flags are **not** set — i.e. it removes the bike's built-in traction/stability cheat |
| `0x04000000` | `HF_HEAVYARMOUR` | `// Vehicle is resistant to explosions` | `Explosion.cpp:4216-4220` | A high-damage close explosion does not instantly destroy it while health > 0 |
| `0x08000000` | `HF_ARMOURED` | `// Vehicle is bullet proof` | `vehicleDamage.cpp:7346`, `8000` | Petrol tank immune, doors cannot be broken off, wheels survive the explosion |
| `0x10000000` | `HF_SELF_RIGHTING_IN_WATER` | — | `VehicleModelInfo.cpp:5466-5474` | Two extra buoyancy spheres outside the hull so the craft self-rights in water |
| `0x20000000` | `HF_IMPROVED_RIGHTING_FORCE` | `// Adds extra force when trying to right the car when upside down.` | `Automobile.cpp:1131-1133` | Aftertouch scale factor 2.0 |
| `0x40000000` | `HF_LOW_SPEED_WHEELIES` | — | `Bike.cpp:3612-3640` | Lean is not scaled by throttle; extra stabilising control at severe wheelie angles; wheelie force 1.0 at low speed |
| `0x80000000` | `HF_LAST_AVAILABLE_FLAG` | — | **nothing** — the identifier appears only in the enum | Sentinel marking the last bit |

**The off-road gravity trio, exactly** (independently re-verified): `Vehicle.cpp:31304-31311` is a single `if (HF_OFFROAD_ABILITIES) … else if (_X2 || _INCREASED_GRAVITY_NO_FOLIAGE_DRAG)` — **mutually exclusive, the base bit winning** — the Mesa3 exception overwrites `fGravityMult` at `:31313-31317`, multipliers are at `:449-451` (1.1 / 1.2, quadbike 1.2 by vehicle *type* at `:31296`), and the single application point is `m_fGravityForWheelIntegrator = fGravity * fGravityMult` at `:31323` — **the wheel integrator's gravity only; no traction coefficient is written**. `handlingMgr.cpp` contains no `HF_` consumer at all, only the declaration and the parse.

## `strDamageFlags` — `VehDoorDamageFlags` (`DF_*`), `handlingMgr.h:129-138`

Six members with no comments in the header at all, parsed into `u32 dFlags` ("door damage flags", `handlingMgr.h:973`) by the third of three adjacent `sscanf` calls (`handlingMgr.cpp:1698` / `:1699` / `:1700` for model, handling and damage). The single consumer in the whole tree is `CAutomobile::InitDoors()` (`Automobile.cpp:8671`): each bit guards one door bone's `Init` and sets `CCarDoor::DONT_BREAK` on it, which flows through to `SetDontBreakFlag` on the frag child (`door.cpp:203-206`) — so **a set bit means that panel can never be torn off**. All six are consumed.

| value | engine name | bone | consumer |
|---|---|---|---|
| `0x00000001` | `DF_DRIVER_SIDE_FRONT_DOOR` | `VEH_DOOR_DSIDE_F` | `Automobile.cpp:8685` |
| `0x00000002` | `DF_DRIVER_SIDE_REAR_DOOR` | `VEH_DOOR_DSIDE_R` | `Automobile.cpp:8708` |
| `0x00000004` | `DF_DRIVER_PASSENGER_SIDE_FRONT_DOOR` | `VEH_DOOR_PSIDE_F` | `Automobile.cpp:8694` |
| `0x00000008` | `DF_DRIVER_PASSENGER_SIDE_REAR_DOOR` | `VEH_DOOR_PSIDE_R` | `Automobile.cpp:8718` |
| `0x00000010` | `DF_BONNET` | `VEH_BONNET` | `Automobile.cpp:8750` |
| `0x00000020` | `DF_BOOT` | `VEH_BOOT` and `VEH_BOOT_2` | `Automobile.cpp:8735`, `:8774` |

## The `handling.meta` flag inventory — five sets, not four

| element | schema (`Vehicles\Metadata\HandlingInfo.psc`) | field it fills | hex parse |
|---|---|---|---|
| `strModelFlags` | `:65` | `CHandlingData::mFlags` — model option flags, `handlingMgr.h:971` | `handlingMgr.cpp:1698` |
| `strHandlingFlags` | `:66` | `CHandlingData::hFlags` — handling option flags, `:972` | `:1699` |
| `strDamageFlags` | `:67` | `CHandlingData::dFlags` — door damage flags, `:973` | `:1700` |
| `strAdvancedFlags` | `:317` | `CCarHandlingData::aFlags` — advanced flags, `:713` | `:1190` |
| **`strFlags`** | `:353` | `CSpecialFlightHandlingData::m_flags`, `:769` | `:1204` |

**The fifth set is the one usually missed**: a special-flight vehicle (delimiter / rocket-flight handling) carries its own `strFlags`, whose bits are `SpecialFlightModeFlags` (`SF_*`, `handlingMgr.h:188-201`) — `SF_WORKS_UPSIDE_DOWN`, `SF_STEER_TOWARDS_VELOCITY`, `SF_FORCE_MIN_THROTTLE_WHEN_TURNING`, `SF_FORCE_SPECIAL_FLIGHT_WHEN_DRIVEN` and friends, consumed across `Vehicle.cpp:35584-36438` and `Transmission.cpp:274`. The schema also declares `m_ModelFlags` / `m_HandlingFlags` / `m_DamageFlags` numeric alternatives, but those sit inside XML comments and are inert.
A supplementary meta loads through `AppendHandlingMetaData` (`:2037`) — the path this project's `Sideload\0_vsl-handling\handling.meta` uses. The legacy `.dat` path (`:1398`, compiled only under `VERIFY_OLD_HANDLING_DAT` / `CONVERT_HANDLING_DAT_TO_META`) has tokens for the three base sets only — **no advanced or special-flight token exists there**. All three base parses discard the `sscanf` result and never validate the string, while the fields are zeroed in the constructor, so a malformed value silently means **no flags** rather than an error.

## `strModelFlags` — `VehModelFlags` (`MF_*`), `handlingMgr.h:41-82`

**The naming trap, and it is a real one**: handling.meta's `strModelFlags` element is **not** the `CVehicleModelInfoFlags` set described in the next section. It is a separate 32-bit legacy enum, `MF_*`, parsed by `sscanf(m_strModelFlags.TryGetCStr(), "%x", &mFlags)` (`handlingMgr.cpp:1698`) into `CHandlingData::mFlags` ("model option flags", `handlingMgr.h:971`) and read as `pHandling->mFlags & MF_...` (e.g. `MF_HAS_TRACKS` at `VehicleFactory.cpp:3039`). Two different sets, both called "model flags".

| bit | engine name | what it is |
|---|---|---|
| `0x00000001` | `MF_IS_VAN` | Van-class (rear doors open sideways) |
| `0x00000002` | `MF_IS_BUS` | Bus-class |
| `0x00000004` | `MF_IS_LOW` | Low vehicle |
| `0x00000008` | `MF_IS_BIG` | Big vehicle |
| `0x00000010` | `MF_ABS_STD` | ABS is standard equipment |
| `0x00000020` | `MF_ABS_OPTION` | ABS is optional equipment |
| `0x00000040` | `MF_ABS_ALT_STD` | Alternative ABS, standard |
| `0x00000080` | `MF_ABS_ALT_OPTION` | Alternative ABS, optional |
| `0x00000100` | `MF_NO_DOORS` | No doors |
| `0x00000200` | `MF_TANDEM_SEATING` | Tandem seating |
| `0x00000400` | `MF_SIT_IN_BOAT` | Seated position is boat-style |
| `0x00000800` | `MF_HAS_TRACKS` | Tracks instead of wheels |
| `0x00001000` | `MF_NO_EXHAUST` | No exhaust |
| `0x00002000` | `MF_DOUBLE_EXHAUST` | Twin exhaust |
| `0x00004000` | `MF_NO_1STPERSON_LOOKBEHIND` | No first-person look-behind |
| `0x00008000` | `MF_CAN_ENTER_IF_NO_DOOR` | Enterable with the door missing |
| `0x00010000` | `MF_AXLE_F_TORSION` | Front axle: torsion |
| `0x00020000` | `MF_AXLE_F_SOLID` | Front axle: solid |
| `0x00040000` | `MF_AXLE_F_MCPHERSON` | Front axle: MacPherson |
| `0x00080000` | `MF_ATTACH_PED_TO_BODYSHELL` | Peds attach to the bodyshell |
| `0x00100000` | `MF_AXLE_R_TORSION` | Rear axle: torsion |
| `0x00200000` | `MF_AXLE_R_SOLID` | Rear axle: solid |
| `0x00400000` | `MF_AXLE_R_MCPHERSON` | Rear axle: MacPherson |
| `0x00800000` | `MF_DONT_FORCE_GRND_CLEARANCE` | Do not force ground clearance (same effect as `HF_DONT_RAISE_BOUNDS_AT_SPEED`) |
| `0x01000000` | `MF_DONT_RENDER_STEER` | Do not render steering |
| `0x02000000` | `MF_NO_WHEEL_BURST` | Wheels cannot burst |
| `0x04000000` | `MF_INDESTRUCTIBLE` | Indestructible |
| `0x08000000` | `MF_DOUBLE_FRONT_WHEELS` | Twin front wheels |
| `0x10000000` | `MF_IS_RC` | Remote-controlled car |
| `0x20000000` | `MF_DOUBLE_REAR_WHEELS` | Twin rear wheels |
| `0x40000000` | `MF_NO_WHEEL_BREAK` | Wheels cannot break off |
| `0x80000000` | `MF_EXTRA_CAMBER` | Extra wheel camber, to stop wheels clipping the arches (lowriders) |

**What this fleet actually declares**: all 41 cars carry `MF_ABS_STD` plus an axle pair, and nothing else of consequence — 35 at `0x440010` (MacPherson front and rear; `RATVIL` spells the same value `00440010`), `TAMPAL` and `LEMONTORA` at solid axles both ends, `GLENLEM` solid rear with MacPherson front, `elpresidente` MacPherson plus `MF_DOUBLE_EXHAUST`, and `diledo` carrying `MF_EXTRA_CAMBER` with twin exhaust. No car sets `MF_HAS_TRACKS`, `MF_IS_RC`, `MF_INDESTRUCTIBLE`, `MF_NO_WHEEL_BURST` or the double-wheel flags — so none of the axle or wheel-count traps in this set are live in the roster.

## `CVehicleModelInfoFlags` — 204 flags, model metadata (NOT the meta's `strModelFlags`)

Defined **only** in the RageParser schema `game\modelinfo\VehicleModelInfoFlags.psc` (`enumdef` at `:5`, 204 enumvals on `:6-209`) — its generated header is not on disk, and `Vehicles\VehicleFlags.h` is a different thing entirely (`class CVehicleFlags`, per-instance runtime bits with a "DO NOT ADD NEW VEHICLE FLAGS" warning). **No member has a `value=` attribute**, so each is the next bit in declaration order: `value = 1 << bit`, bits 0–203 across seven 32-bit words. Storage `VehicleModelInfo.h:1617`, accessor `GetVehicleFlag(flag)` at `:1036`, max `Flags_NUM_ENUMS` at `:755`. **`SetVehicleFlag` has zero hits in the whole tree** — these flags are authored in model metadata and never mutated at runtime. The verbatim human description for bit *N* is on **schema line `6 + N`**; the table below paraphrases it.

Seven flags have **no consumer anywhere** in `src\dev_ng`: `FLAG_NO_BOOT` (2), `FLAG_BOOT_IN_FRONT` (4), `FLAG_TAILGATE_TYPE_BOOT` (29, only two commented-out reads at `TaskParkedVehicleScenario.cpp:86,114`), `FLAG_USE_WEAPON_WHEEL_WITHOUT_HELMET` (106), `FLAG_HALF_TRACK` (132), `FLAG_TURRET_MODS_ON_CHASSIS` (147), `FLAG_DONT_LINK_BOOT2` (170). Twelve have no description at all (bits 71–74, 77, 98, 100, 101, 103, 106, 110, 111), and two descriptions are copy-paste wrong (165 carries the headlights text, 175 the nitrous text) — so the description is not always trustworthy, the consumer is. **★ marks the ones that can change what ARS models.**

| bit | value | engine name | what it does (paraphrase) | primary consumer | ★ |
|---|---|---|---|---|---|
| 0 | `0x1` | `FLAG_SMALL_WORKER` | Small worker vehicle (forklift) — specific paths only | `vehiclepopulation.cpp:4986` | |
| 1 | `0x2` | `FLAG_BIG` | Very big, avoids turning | `cargen.cpp:366` | |
| 2 | `0x4` | `FLAG_NO_BOOT` | No boot inventory | **dead** | |
| 3 | `0x8` | `FLAG_ONLY_DURING_OFFICE_HOURS` | Streams only in office hours | `populationstreaming.cpp:1994` | |
| 4 | `0x10` | `FLAG_BOOT_IN_FRONT` | Boot at the front | **dead** | |
| 5 | `0x20` | `FLAG_IS_VAN` | Van: rear doors sideways, passengers capped | `vehiclepopulation.cpp:8761` | |
| 6 | `0x40` | `FLAG_AVOID_TURNS` | Prefers going straight | `vehiclepopulation.cpp:4989` | |
| 7 | `0x80` | `FLAG_HAS_LIVERY` | Has liveries (texture swapped on instancing) | `VehicleFactory.cpp:3599` | |
| 8 | `0x100` | `FLAG_LIVERY_MATCH_EXTRA` | Texture swap follows the part swap | `CustomShaderEffectVehicle.cpp:1798` | |
| 9 | `0x200` | `FLAG_SPORTS` | Sports-car population class | `cargen.cpp:360` | |
| 10 | `0x400` | `FLAG_DELIVERY` | Delivery class (vans, small trucks) | `cargen.cpp:378` | |
| 11 | `0x800` | `FLAG_NOAMBIENTOCCLUSION` | No body ambient occlusion | `Vehicle.cpp:19400` | |
| 12 | `0x1000` | `FLAG_ONLY_ON_HIGHWAYS` | Highway-only spawning | `vehiclepopulation.cpp:4987` | |
| 13 | `0x2000` | `FLAG_TALL_SHIP` | Tall ship: avoids low bridges | `vehiclepopulation.cpp:5154` | |
| 14 | `0x4000` | `FLAG_SPAWN_ON_TRAILER` | Vehicle/trailer pairing | `cargen.cpp:3336` | |
| 15 | `0x8000` | `FLAG_SPAWN_BOAT_ON_TRAILER` | Boat/trailer pairing | `cargen.cpp:3340` | |
| 16 | `0x10000` | `FLAG_EXTRAS_GANG` | Gang extra selection | `Vehicle.cpp:22771` | |
| 17 | `0x20000` | `FLAG_EXTRAS_CONVERTIBLE` | Convertible roof extras | `Vehicle.cpp:22795` | |
| 18 | `0x40000` | `FLAG_EXTRAS_TAXI` | Taxi light extra | `cargen.cpp:333` | |
| 19 | `0x80000` | `FLAG_EXTRAS_RARE` | Lower chance of any extra | `Vehicle.cpp:22935` | |
| 20 | `0x100000` | `FLAG_EXTRAS_REQUIRE` | One extra is always chosen | `Vehicle.cpp:22786` | |
| 21 | `0x200000` | `FLAG_EXTRAS_STRONG` | Extras do not break off | `vehicleDamage.cpp:6721` | |
| 22 | `0x400000` | `FLAG_EXTRAS_ONLY_BREAK_WHEN_DESTROYED` | Extras survive until destruction | `VehicleModelInfo.cpp:4253` | |
| 23 | `0x800000` | `FLAG_EXTRAS_SCRIPT` | Script-only extras | `train.cpp:6905` | |
| 24 | `0x1000000` | `FLAG_EXTRAS_ALL` | All suitable extras on | `Vehicle.cpp:22781` | |
| 25 | `0x2000000` | `FLAG_EXTRAS_MATCH_LIVERY` | Livery index forced from extra index | `Vehicle.cpp:23361` | |
| 26 | `0x4000000` | `FLAG_DONT_ROTATE_TAIL_ROTOR` | Tail rotor not rotated | `Heli.cpp:2833` | |
| 27 | `0x8000000` | `FLAG_PARKING_SENSORS` | Parking sensor gadget | `VehicleFactory.cpp:3051` | |
| 28 | `0x10000000` | `FLAG_PEDS_CAN_STAND_ON_TOP` | Peds can stand on it | `Vehicle.cpp:30556` | |
| 29 | `0x20000000` | `FLAG_TAILGATE_TYPE_BOOT` | Tailgate boot — **dead** (two commented reads) | — | |
| 30 | `0x40000000` | `FLAG_GEN_NAVMESH` | Generates a navmesh | `PathServer_Objects.cpp:3426` | |
| 31 | `0x80000000` | `FLAG_LAW_ENFORCEMENT` | Law enforcement vehicle | `cargen.cpp:972` | |
| 32 | w1 `0x1` | `FLAG_EMERGENCY_SERVICE` | Emergency service | `TaskFlee.cpp:3239` | |
| 33 | w1 `0x2` | `FLAG_DRIVER_NO_DRIVE_BY` | Driver cannot drive-by | `TaskCar.cpp:3995` | |
| 34 | w1 `0x4` | `FLAG_NO_RESPRAY` | Cannot be resprayed | `garages.cpp:2332` | |
| 35 | w1 `0x8` | `FLAG_IGNORE_ON_SIDE_CHECK` | Enterable while on its side | `PlayerInfo.cpp:3649` | |
| 36 | w1 `0x10` | `FLAG_RICH_CAR` | Rich driver peds | `pedpopulation.cpp:8806` | |
| 37 | w1 `0x20` | `FLAG_AVERAGE_CAR` | Average driver peds | `pedpopulation.cpp:8810` | |
| 38 | w1 `0x40` | `FLAG_POOR_CAR` | Poor driver peds | `pedpopulation.cpp:8814` | |
| 39 | w1 `0x80` | `FLAG_ALLOWS_RAPPEL` | Rappelling allowed | `TaskRappel.cpp:235` | |
| 40 | w1 `0x100` | `FLAG_DONT_CLOSE_DOOR_UPON_EXIT` | Door left open on exit | `TaskVehicleBase.cpp:2145` | |
| 41 | w1 `0x200` | `FLAG_USE_HIGHER_DOOR_TORQUE` | Doors open/close faster | `door.cpp:725` | |
| 42 | w1 `0x400` | `FLAG_DISABLE_THROUGH_WINDSCREEN` | No windscreen ejection | `TaskInVehicle.cpp:507` | |
| 43 | w1 `0x800` | `FLAG_IS_ELECTRIC` | **No petrol tank — electric transmission branch** | `Transmission.cpp:852` | ★ |
| 44 | w1 `0x1000` | `FLAG_NO_BROKEN_DOWN_SCENARIO` | No broken-down-car scenarios | `ScenarioManager.cpp:3112` | |
| 45 | w1 `0x2000` | `FLAG_IS_JETSKI` | Jetski | `Vehicle.cpp:23400` | |
| 46 | w1 `0x4000` | `FLAG_DAMPEN_STICKBOMB_DAMAGE` | Dampened sticky-bomb damage | `door.cpp:578` | |
| 47 | w1 `0x8000` | `FLAG_DONT_SPAWN_IN_CARGEN` | Never parked-spawned | `cargen.cpp:284` | |
| 48 | w1 `0x10000` | `FLAG_IS_OFFROAD_VEHICLE` | Satisfies off-road spawn requests | `vehiclepopulation.cpp:4988` | |
| 49 | w1 `0x20000` | `FLAG_INCREASE_PED_COMMENTS` | More ped comments | `TaskCar.cpp:1438` | |
| 50 | w1 `0x40000` | `FLAG_EXPLODE_ON_CONTACT` | Projectiles can force an explosion | `vehicle.h:882` | |
| 51 | w1 `0x80000` | `FLAG_USE_FAT_INTERIOR_LIGHT` | Bigger interior light | `Vehicle.cpp:18929` | |
| 52 | w1 `0x100000` | `FLAG_HEADLIGHTS_USE_ACTUAL_BONE_POS` | Headlights from the real bone (expensive) | `Vehicle.cpp:17256` | |
| 53 | w1 `0x200000` | `FLAG_FAKE_EXTRALIGHTS` | Extra lights toggle but emit nothing | `Vehicle.cpp:17353` | |
| 54 | w1 `0x400000` | `FLAG_CANNOT_BE_MODDED` | Cannot be modded at all | `Vehicle.cpp:10272` | |
| 55 | w1 `0x800000` | `FLAG_DONT_SPAWN_AS_AMBIENT` | Not in the ambient population | `populationstreaming.cpp:4952` | |
| 56 | w1 `0x1000000` | `FLAG_IS_BULKY` | Bulky (vans, SUVs) | `TaskVehicleShotTire.cpp:260` | |
| 57 | w1 `0x2000000` | `FLAG_BLOCK_FROM_ATTRACTOR_SCENARIO` | Not attracted by vehicle scenarios | `ScenarioVehicleManager.cpp:345` | |
| 58 | w1 `0x4000000` | `FLAG_IS_BUS` | Bus | `Vehicle.cpp:12918` | |
| 59 | w1 `0x8000000` | `FLAG_USE_STEERING_PARAM_FOR_LEAN` | Steering param drives body lean | `TaskInVehicle.cpp:4111` | |
| 60 | w1 `0x10000000` | `FLAG_CANNOT_BE_DRIVEN_BY_PLAYER` | Player cannot drive it | `PlayerInfo.cpp:3635` | ★ |
| 61 | w1 `0x20000000` | `FLAG_SPRAY_PETROL_BEFORE_EXPLOSION` | Petrol spray vfx before exploding | `vehicle.h:884` | |
| 62 | w1 `0x40000000` | `FLAG_ATTACH_TRAILER_ON_HIGHWAY` | Trailer attach on highways | `vehiclepopulation.cpp:5018` | |
| 63 | w1 `0x80000000` | `FLAG_ATTACH_TRAILER_IN_CITY` | Trailer attach in the city | `vehiclepopulation.cpp:5019` | |
| 64 | w2 `0x1` | `FLAG_HAS_NO_ROOF` | No roof at all | `Ped.cpp:18346` | |
| 65 | w2 `0x2` | `FLAG_ALLOW_TARGETING_OF_OCCUPANTS` | Occupants can be locked on to | `PedTargetEvaluator.cpp:1820` | |
| 66 | w2 `0x4` | `FLAG_RECESSED_HEADLIGHT_CORONAS` | Recessed headlight coronas | `Vehicle.cpp:17180` | |
| 67 | w2 `0x8` | `FLAG_RECESSED_TAILLIGHT_CORONAS` | Recessed taillight coronas | `Vehicle.cpp:16916` | |
| 68 | w2 `0x10` | `FLAG_IS_TRACKED_FOR_TRAILS` | Leaves trails (grass) in the world | `VehicleModelInfo.h:1384` | |
| 69 | w2 `0x20` | `FLAG_HEADLIGHTS_ON_LANDINGGEAR` | Headlights on the landing gear | `Vehicle.cpp:17226` | |
| 70 | w2 `0x40` | `FLAG_CONSIDERED_FOR_VEHICLE_ENTRY_WHEN_STOOD_ON` | Enterable while stood on | `VehicleModelInfo.cpp:8327` | |
| 71 | w2 `0x80` | `FLAG_GIVE_SCUBA_GEAR_ON_EXIT` | Scuba gear on exit | `TaskExitVehicle.cpp:10422` | |
| 72 | w2 `0x100` | `FLAG_IS_DIGGER` | Seat matrix from the digger-arm gadget | `ModelSeatInfo.cpp:1044` | |
| 73 | w2 `0x200` | `FLAG_IS_TANK` | Tank | `Vehicle.cpp:14433` | |
| 74 | w2 `0x400` | `FLAG_USE_COVERBOUND_INFO_FOR_COVERGEN` | Cover generation from cover bounds | `VehicleModelInfo.cpp:3704` | |
| 75 | w2 `0x800` | `FLAG_CAN_BE_DRIVEN_ON` | Can be driven on without wheel-ignore | `Vehicle.cpp:21222` | |
| 76 | w2 `0x1000` | `FLAG_HAS_BULLETPROOF_GLASS` | Glass never smashes | `door.cpp:2089` | |
| 77 | w2 `0x2000` | `FLAG_CANNOT_TAKE_COVER_WHEN_STOOD_ON` | No cover while stood on | `TaskCover.cpp:28896` | |
| 78 | w2 `0x4000` | `FLAG_INTERIOR_BLOCKED_BY_BOOT` | Hollow interior entered via the boot | `Vehicle.cpp:20849` | |
| 79 | w2 `0x8000` | `FLAG_DONT_TIMESLICE_WHEELS` | **Wheel time-slicing disabled for this model** | `Automobile.cpp:4081` (AI `:4202`) | ★ |
| 80 | w2 `0x10000` | `FLAG_FLEE_FROM_COMBAT` | Occupants flee instead of fighting | `Vehicle.cpp:14611` | |
| 81 | w2 `0x20000` | `FLAG_DRIVER_SHOULD_BE_FEMALE` | Scenario driver is female | `ScenarioManager.cpp:4309` | |
| 82 | w2 `0x40000` | `FLAG_DRIVER_SHOULD_BE_MALE` | Scenario driver is male | `ScenarioManager.cpp:4305` | |
| 83 | w2 `0x80000` | `FLAG_COUNT_AS_FACEBOOK_DRIVEN` | Counts as driven for stats | `StatsMgr.cpp:2964` | |
| 84 | w2 `0x100000` | `FLAG_BIKE_CLAMP_PICKUP_LEAN_RATE` | Clamps bike pickup lean rate | `Bike.cpp:3192` | |
| 85 | w2 `0x200000` | `FLAG_PLANE_WEAR_ALTERNATIVE_HELMET` | Alternative helmet | `Ped.cpp:29590` | |
| 86 | w2 `0x400000` | `FLAG_USE_STRICTER_EXIT_COLLISION_TESTS` | Stricter slope exit collision | `TaskExitVehicle.cpp:6944` | |
| 87 | w2 `0x800000` | `FLAG_TWO_DOORS_ONE_SEAT` | Two-door-one-seat animations | `TaskInVehicle.cpp:895` | |
| 88 | w2 `0x1000000` | `FLAG_USE_LIGHTING_INTERIOR_OVERRIDE` | Interior lighting override for rear passengers | `Vehicle.cpp:32425` | |
| 89 | w2 `0x2000000` | `FLAG_USE_RESTRICTED_DRIVEBY_HEIGHT` | Clamped drive-by pitch | `TaskVehicleDriveBy.cpp:3278` | |
| 90 | w2 `0x4000000` | `FLAG_CAN_HONK_WHEN_FLEEING` | May honk while fleeing | `TaskVehicleCruise.cpp:1136` | |
| 91 | w2 `0x8000000` | `FLAG_PEDS_INSIDE_CAN_BE_SET_ON_FIRE_MP` | Occupants can burn (MP). **Also read as `KeepHatOnInOpenTopVehicles`** | `Fire.cpp:2263`, `Ped.cpp:29154` | |
| 92 | w2 `0x10000000` | `FLAG_REPORT_CRIME_IF_STANDING_ON` | Standing on it reports a crime | `Vehicle.cpp:4142` | |
| 93 | w2 `0x20000000` | `FLAG_HELI_USES_FIXUPS_ON_OPEN_DOOR` | Fixups on align/open-door states | `TaskEnterVehicle.cpp:2344` | |
| 94 | w2 `0x40000000` | `FLAG_FORCE_ENABLE_CHASSIS_COLLISION` | Non-BVH chassis bounds keep collision flags | `Vehicle.cpp:5744` | |
| 95 | w2 `0x80000000` | `FLAG_CANNOT_BE_PICKUP_BY_CARGOBOB` | Cannot be cargobob-lifted | `VehicleGadgets.cpp:14387` | |
| 96 | w3 `0x1` | `FLAG_CAN_HAVE_NEONS` | Can spawn with neons | `vehiclepopulation.cpp:6013` | |
| 97 | w3 `0x2` | `FLAG_HAS_INTERIOR_EXTRAS` | Extras 10–12 are interior extras | `Vehicle.cpp:22850` | |
| 98 | w3 `0x4` | `FLAG_HAS_TURRET_SEAT_ON_VEHICLE` | Turret seats (the most-read model flag) | `Automobile.cpp:1830` | |
| 99 | w3 `0x8` | `FLAG_ALLOW_OBJECT_LOW_LOD_COLLISION` | Objects may hit the low-LOD chassis | `Vehicle.cpp:5234` | |
| 100 | w3 `0x10` | `FLAG_DISABLE_AUTO_VAULT_ON_VEHICLE` | No auto-vault onto it | `Vehicle.cpp:4945` | |
| 101 | w3 `0x20` | `FLAG_USE_TURRET_RELATIVE_AIM_CALCULATION` | Turret aim from relative heading | `VehicleGadgets.cpp:2521` | |
| 102 | w3 `0x40` | `FLAG_USE_FULL_ANIMS_FOR_MP_WARP_ENTRY_POINTS` | Full anims on warp entry points | `TaskVehicleBase.cpp:881` | |
| 103 | w3 `0x80` | `FLAG_HAS_DIRECTIONAL_SHUFFLES` | 180° shuffle clips | `TaskEnterVehicle.cpp:17892` | |
| 104 | w3 `0x100` | `FLAG_DISABLE_WEAPON_WHEEL_IN_FIRST_PERSON` | No weapon wheel in first person | `CWeaponWheel.cpp:916` | |
| 105 | w3 `0x200` | `FLAG_USE_PILOT_HELMET` | Pilot helmet regardless of seat weapons | `PedHelmetComponent.cpp:899` | |
| 106 | w3 `0x400` | `FLAG_USE_WEAPON_WHEEL_WITHOUT_HELMET` | — | **dead** | |
| 107 | w3 `0x800` | `FLAG_PREFER_ENTER_TURRET_AFTER_DRIVER` | Turret seats preferred once driven | `ModelSeatInfo.cpp:2756` | |
| 108 | w3 `0x1000` | `FLAG_USE_SMALLER_OPEN_DOOR_RATIO_TOLERANCE` | Door counts as open sooner | `Vehicle.cpp:27027` | |
| 109 | w3 `0x2000` | `FLAG_USE_HEADING_ONLY_IN_TURRET_MATRIX` | Turret matrix from heading only | `VehicleGadgets.cpp:2610` | |
| 110 | w3 `0x4000` | `FLAG_DONT_STOP_WHEN_GOING_TO_CLIMB_UP_POINT` | Straight to the climb-up point | `ModelSeatInfo.cpp:3064` | |
| 111 | w3 `0x8000` | `FLAG_HAS_REAR_MOUNTED_TURRET` | Rear-mounted turret | `VehicleGadgets.cpp:2219` | |

| 112 | w3 `0x10000` | `FLAG_DISABLE_BUSTING` | Cops cannot bust a player in it | `TaskNewCombat.cpp:7695` | |
| 113 | w3 `0x20000` | `FLAG_IGNORE_RWINDOW_COLLISION` | Rear-windscreen bullets ignore the vehicle | `TaskVehicleDriveBy.cpp:901` | |
| 114 | w3 `0x40000` | `FLAG_HAS_GULL_WING_DOORS` | Doors open upwards | `door.cpp:88` | |
| 115 | w3 `0x80000` | `FLAG_CARGOBOB_HOOK_UP_CHASSIS` | Cargobob hook offset from the chassis bound | `VehicleGadgets.cpp:12466` | |
| 116 | w3 `0x100000` | `FLAG_USE_FIVE_ANIM_THROW_FP` | Wider throw animations in first person | `TaskMountAnimalWeapon.cpp:2836` | |
| 117 | w3 `0x200000` | `FLAG_ALLOW_HATS_NO_ROOF` | Hats kept in a no-roof vehicle | `Ped.cpp:29179` | |
| 118 | w3 `0x400000` | `FLAG_HAS_REAR_SEAT_ACTIVITIES` | Rear-seat activities/entry | `TaskCar.cpp:3147` | |
| 119 | w3 `0x800000` | `FLAG_HAS_LOWRIDER_HYDRAULICS` | Hydraulics set up at model init | `Vehicle.cpp:4200` | |
| 120 | w3 `0x1000000` | `FLAG_HAS_BULLET_RESISTANT_GLASS` | Glass has a health bar | `door.cpp:2089` | |
| 121 | w3 `0x2000000` | `FLAG_HAS_INCREASED_RAMMING_FORCE` | Rams roadblocks and cars more easily | `Vehicle.cpp:14479` | ★ |
| 122 | w3 `0x4000000` | `FLAG_HAS_CAPPED_EXPLOSION_DAMAGE` | One explosion hit is survivable | `vehicleDamage.cpp:5845` | |
| 123 | w3 `0x8000000` | `FLAG_HAS_LOWRIDER_DONK_HYDRAULICS` | Donk hydraulics | `Vehicle.cpp:4201` | |
| 124 | w3 `0x10000000` | `FLAG_HELICOPTER_WITH_LANDING_GEAR` | Retractable landing gear | `Heli.cpp:2763` | |
| 125 | w3 `0x20000000` | `FLAG_JUMPING_CAR` | Part of `HasJump()` | `Vehicle.cpp:2025` | |
| 126 | w3 `0x40000000` | `FLAG_HAS_ROCKET_BOOST` | Rechargeable rocket boost | `Vehicle.cpp:1858` | ★ |
| 127 | w3 `0x80000000` | `FLAG_RAMMING_SCOOP` | Scoop rams cars aside in the physics impact handler | `physics.cpp:2732` | ★ |
| 128 | w4 `0x1` | `FLAG_HAS_PARACHUTE` | Has a parachute | `Vehicle.cpp:25141` | |
| 129 | w4 `0x2` | `FLAG_RAMP` | Can be used as a ramp | `Vehicle.cpp:14567` | |
| 130 | w4 `0x4` | `FLAG_HAS_EXTRA_SHUFFLE_SEAT_ON_VEHICLE` | Extra shuffle seats | `TaskVehicleBase.cpp:1511` | |
| 131 | w4 `0x8` | `FLAG_FRONT_BOOT` | Boot at the front | `Automobile.cpp:8729` | |
| 132 | w4 `0x10` | `FLAG_HALF_TRACK` | Half-track — **dead** | — | |
| 133 | w4 `0x20` | `FLAG_RESET_TURRET_SEAT_HEADING` | Turret heading resets when unused | `VehicleGadgets.cpp:2778` | |
| 134 | w4 `0x40` | `FLAG_TURRET_MODS_ON_ROOF` | Turret mod from the roof slot | `Vehicle.cpp:4166` | |
| 135 | w4 `0x80` | `FLAG_UPDATE_WEAPON_BATTERY_BONES` | Weapon battery bones updated | `Vehicle.cpp:3350` | |
| 136 | w4 `0x100` | `FLAG_DONT_HOLD_LOW_GEARS_WHEN_ENGINE_UNDER_LOAD` | Changes up on reaching the point even under load | `Transmission.cpp:800` | ★ |
| 137 | w4 `0x200` | `FLAG_HAS_GLIDER` | Glider equipped | `Vehicle.cpp:25144` | |
| 138 | w4 `0x400` | `FLAG_INCREASE_LOW_SPEED_TORQUE` | Torque increased at low speed for hills | `Transmission.cpp:1338` | ★ |
| 139 | w4 `0x800` | `FLAG_USE_AIRCRAFT_STYLE_WEAPON_TARGETING` | Aircraft-style lock-on and reticle | `TaskVehicleWeapon.cpp:1888` | |
| 140 | w4 `0x1000` | `FLAG_KEEP_ALL_TURRETS_SYNCHRONISED` | One rotation for every turret bone | `VehicleGadgets.cpp:2957` | |
| 141 | w4 `0x2000` | `FLAG_SET_WANTED_FOR_ATTACHED_VEH` | Wanted level shared with attached vehicles | `Wanted.cpp:1435` | |
| 142 | w4 `0x4000` | `FLAG_TURRET_ENTRY_ATTACH_TO_DRIVER_SEAT` | Attached to the driver seat for align maths | `TaskEnterVehicle.cpp:13120` | |
| 143 | w4 `0x8000` | `FLAG_USE_STANDARD_FLIGHT_HELMET` | Standard flight helmet forced | `PedHelmetComponent.cpp:855` | |
| 144 | w4 `0x10000` | `FLAG_SECOND_TURRET_MOD` | Secondary turret mod slot | `Vehicle.cpp:23742` | |
| 145 | w4 `0x20000` | `FLAG_THIRD_TURRET_MOD` | Third turret mod slot | `Vehicle.cpp:23772` | |
| 146 | w4 `0x40000` | `FLAG_HAS_EJECTOR_SEATS` | Ejector-seat exit path | `TaskExitVehicle.cpp:10449` | |
| 147 | w4 `0x80000` | `FLAG_TURRET_MODS_ON_CHASSIS` | — | **dead** | |
| 148 | w4 `0x100000` | `FLAG_HAS_JATO_BOOST_MOD` | JATO rocket boost from the exhaust slot | `Vehicle.cpp:1859` | ★ |
| 149 | w4 `0x200000` | `FLAG_IGNORE_TRAPPED_HULL_CHECK` | Always goes straight to the climb-up point | `TaskGoToVehicleDoor.cpp:1685` | |
| 150 | w4 `0x400000` | `FLAG_HOLD_TO_SHUFFLE` | Shuffling is hold, not tap | `TaskCar.cpp:3686` | |
| 151 | w4 `0x800000` | `FLAG_TURRET_MOD_WITH_NO_STOCK_TURRET` | Turret only from a mod | `Vehicle.cpp:23715` | |
| 152 | w4 `0x1000000` | `FLAG_EQUIP_UNARMED_ON_ENTER` | Unarmed forced on entry | `VehicleModelInfoVariation.cpp:1242` | |
| 153 | w4 `0x2000000` | `FLAG_DISABLE_CAMERA_PUSH_BEYOND` | Camera push-beyond disabled | `camera\helpers\Collision.cpp:1436` | |
| 154 | w4 `0x4000000` | `FLAG_HAS_VERTICAL_FLIGHT_MODE` | Vertical flight mode | `Planes.cpp:2022` | |
| 155 | w4 `0x8000000` | `FLAG_HAS_OUTRIGGER_LEGS` | Outrigger legs to deploy | `Automobile.cpp:1880` | |
| 156 | w4 `0x10000000` | `FLAG_CAN_NAVIGATE_TO_ON_VEHICLE_ENTRY` | On-vehicle entry navigation | `TaskEnterVehicle.cpp:4249` | |
| 157 | w4 `0x20000000` | `FLAG_DROP_SUSPENSION_WHEN_STOPPED` | **Suspension lowers at rest** | `Automobile.cpp:6975`, `wheel.cpp:834` | ★ |
| 158 | w4 `0x40000000` | `FLAG_DONT_CRASH_ABANDONED_NEAR_GROUND` | No near-ground crash when abandoned | `TaskVehiclePlayer.cpp:5036` | |
| 159 | w4 `0x80000000` | `FLAG_USE_INTERIOR_RED_LIGHT` | Special red interior light | `Vehicle.cpp:18931` | |
| 160 | w5 `0x1` | `FLAG_HAS_HELI_STRAFE_MODE` | Helicopter strafe handling mode | `Heli.h:160` | |
| 161 | w5 `0x2` | `FLAG_HAS_VERTICAL_ROCKET_BOOST` | Boost that works vertically | `Vehicle.cpp:33857` | ★ |
| 162 | w5 `0x4` | `FLAG_CREATE_WEAPON_MANAGER_ON_SPAWN` | Weapon manager created at spawn | `Vehicle.cpp:4164` | |
| 163 | w5 `0x8` | `FLAG_USE_ROOT_AS_BASE_LOCKON_POS` | Lock-on from the root, not the bounding-box centre | `Vehicle.cpp:10745` | |
| 164 | w5 `0x10` | `FLAG_HEADLIGHTS_ON_TAP_ONLY` | Headlights toggle on tap | `Vehicle.cpp:17065` | |
| 165 | w5 `0x20` | `FLAG_CHECK_WARP_TASK_FLAG_DURING_ENTER` | Warp flag checked during entry (**description is copy-paste wrong**) | `TaskEnterVehicle.cpp:11965` | |
| 166 | w5 `0x40` | `FLAG_USE_RESTRICTED_DRIVEBY_HEIGHT_HIGH` | No low drive-by sweep | `TaskVehicleDriveBy.cpp:3282` | |
| 167 | w5 `0x80` | `FLAG_INCREASE_CAMBER_WITH_SUSPENSION_MOD` | Front camber up with a suspension mod | `WheelRendering.cpp:333` | |
| 168 | w5 `0x100` | `FLAG_NO_HEAVY_BRAKE_ANIMATION` | No heavy brake animation | `TaskInVehicle.cpp:413` | |
| 169 | w5 `0x200` | `FLAG_HAS_TWO_BONNET_BONES` | Two linked bonnet bones | `Automobile.cpp:8756` | |
| 170 | w5 `0x400` | `FLAG_DONT_LINK_BOOT2` | — | **dead** | |
| 171 | w5 `0x800` | `FLAG_HAS_INCREASED_RAMMING_FORCE_WITH_CHASSIS_MOD` | Extra ramming force from a chassis mod | `Vehicle.cpp:14493` | ★ |
| 172 | w5 `0x1000` | `FLAG_HAS_INCREASED_RAMMING_FORCE_VS_ALL_VEHICLES` | Ramming force applies to everything | `Vehicle.cpp:14511` | ★ |
| 173 | w5 `0x2000` | `FLAG_HAS_EXTENDED_COLLISION_MODS` | Mod bones can toggle collision | `Vehicle.cpp:34192` | |
| 174 | w5 `0x4000` | `FLAG_HAS_NITROUS_MOD` | **Mods can enable nitrous** | `Vehicle.cpp:1895` | ★ |
| 175 | w5 `0x8000` | `FLAG_HAS_JUMP_MOD` | Jump from mods (**description is copy-paste wrong**) | `Vehicle.cpp:2019` | |
| 176 | w5 `0x10000` | `FLAG_HAS_RAMMING_SCOOP_MOD` | Scoop enabled by a mod | `Vehicle.cpp:4380` | ★ |
| 177 | w5 `0x20000` | `FLAG_HAS_SUPER_BRAKES_MOD` | **Super brakes from a mod** | `Vehicle.cpp:14545` | ★ |
| 178 | w5 `0x40000` | `FLAG_CRUSHES_OTHER_VEHICLES` | Crushes cars it drives over | `Vehicle.cpp:21377`, `wheel.cpp:4883` | ★ |
| 179 | w5 `0x80000` | `FLAG_HAS_WEAPON_BLADE_MODS` | Blade mods | `Vehicle.cpp:4363` | |
| 180 | w5 `0x100000` | `FLAG_HAS_WEAPON_SPIKE_MODS` | Spike mods | `Vehicle.cpp:4664` | |
| 181 | w5 `0x200000` | `FLAG_FORCE_BONNET_CAMERA_INSTEAD_OF_POV` | Bonnet camera forced | `CinematicDirector.cpp:618` | |
| 182 | w5 `0x400000` | `FLAG_RAMP_MOD` | Ramp when the mod is enabled | `Vehicle.cpp:4384` | |
| 183 | w5 `0x800000` | `FLAG_HAS_TOMBSTONE` | Breakable off-bumper mod | `Vehicle.cpp:37925` | |
| 184 | w5 `0x1000000` | `FLAG_HAS_SIDE_SHUNT` | Side shunt mod | `Vehicle.cpp:1921` | |
| 185 | w5 `0x2000000` | `FLAG_HAS_FRONT_SPIKE_MOD` | Front spike mods | `Vehicle.cpp:4697` | |
| 186 | w5 `0x4000000` | `FLAG_HAS_RAMMING_BAR_MOD` | Ramming bar mods | `Vehicle.cpp:4580` | |
| 187 | w5 `0x8000000` | `FLAG_TURRET_MODS_ON_CHASSIS5` | Turret mods from CHASSIS5 | `Vehicle.cpp:23696` | |
| 188 | w5 `0x10000000` | `FLAG_HAS_SUPERCHARGER` | Supercharger shown on the gauges | `Vehicle.cpp:15313` | |
| 189 | w5 `0x20000000` | `FLAG_IS_TANK_WITH_FLAME_DAMAGE` | Tank that takes fire damage | `Vehicle.cpp:4084` | |
| 190 | w5 `0x40000000` | `FLAG_DISABLE_DEFORMATION` | Never deforms | `Vehicle.cpp:4076` | |
| 191 | w5 `0x80000000` | `FLAG_ALLOW_RAPPEL_AI_ONLY` | AI-only rappel | `TaskRappel.cpp:235` | |
| 192 | w6 `0x1` | `FLAG_USE_RESTRICTED_DRIVEBY_HEIGHT_MID_ONLY` | Only mid drive-by height | `TaskVehicleDriveBy.cpp:3286` | |
| 193 | w6 `0x2` | `FLAG_FORCE_AUTO_VAULT_ON_VEHICLE_WHEN_STUCK` | Forces the auto-vault scan | `Events.cpp:5447` | |
| 194 | w6 `0x4` | `FLAG_SPOILER_MOD_DOESNT_INCREASE_GRIP` | **The spoiler slot adds no downforce** — read by the very native ARS divides downforce out of | `wheel.cpp:7655`, `Vehicle.cpp:38153` | ★ |
| 195 | w6 `0x8` | `FLAG_NO_REVERSING_ANIMATION` | No reversing animation | `TaskInVehicle.cpp:716` | |
| 196 | w6 `0x10` | `FLAG_IS_QUADBIKE_USING_BIKE_ANIMATIONS` | Car-class quadbike using bike anims | `TaskExitVehicle.cpp:3039` | ★ |
| 197 | w6 `0x20` | `FLAG_IS_FORMULA_VEHICLE` | Formula vehicle (warp-out when on its side; wheel-track decal setting) | `wheel.cpp:6379` | ★ |
| 198 | w6 `0x40` | `FLAG_LATCH_ALL_JOINTS` | All joints latched at creation | `Vehicle.cpp:4171` | |
| 199 | w6 `0x80` | `FLAG_REJECT_ENTRY_TO_VEHICLE_WHEN_STOOD_ON` | Not enterable while stood on | `VehicleModelInfo.cpp:8335` | |
| 200 | w6 `0x100` | `FLAG_CHECK_IF_DRIVER_SEAT_IS_CLOSER_THAN_TURRETS_WITH_ON_BOARD_ENTER` | Driver-vs-turret distance on on-board entry | `TaskVehicleBase.cpp:1840` | |
| 201 | w6 `0x200` | `FLAG_RENDER_WHEELS_WITH_ZERO_COMPRESSION` | **Wheels rendered with zero compression** regardless of suspension | `Automobile.cpp:771` | ★ |
| 202 | w6 `0x400` | `FLAG_USE_LENGTH_OF_VEHICLE_BOUNDS_FOR_PLAYER_LOCKON_POS` | Lock-on position from the bounds length | `PedTargetEvaluator.cpp:133` | |
| 203 | w6 `0x800` | `FLAG_PREFER_FRONT_SEAT` | Front seat preferred on entry | `TaskEnterVehicle.cpp:10191` | |

**The fleet's handling flags, decoded** (41 cars in `Sideload\0_vsl-handling\handling.meta`):

| `strHandlingFlags` | cars | decodes to |
|---|---|---|
| `820100` | 38 | `HF_FREEWHEEL_NO_GAS` + `HF_TYRES_CAN_CLIP` + `HF_OFFROAD_INCREASED_GRAVITY_NO_FOLIAGE_DRAG` |
| `82000A` | 1 — `RETINUEL` | the same three, plus **`HF_HAS_RALLY_TYRES`** and `HF_REDUCED_MOD_MASS` |
| `20000` | 1 — `MINIMUSFD` | `HF_TYRES_CAN_CLIP` only |
| `20000001` | 1 — `TAMPADRL` | `HF_SMOOTHED_COMPRESSION` + `HF_IMPROVED_RIGHTING_FORCE` (no gravity bit) |

So 39 cars carry the off-road gravity bit, `RETINUEL` is the only car with the rally-tyre curve, and `TAMPADRL` and `MINIMUSFD` are the only two the gravity model treats as normal. **`TAMPADRL` is also the file's only `fEngineResistance`** (`0.04`, `handling.meta:2128`) — so the single car that blanks the bottom of its throttle is also one of the three that keeps normal off-throttle wheel drag instead of freewheeling.

## `strAdvancedFlags` — `CarAdvancedFlags` (`CF_*`), `handlingMgr.h:140-186`

Lives on `CCarHandlingData`, not on `CHandlingData` — so a car only has this set if its `<SubHandlingData>` carries a `CCarHandlingData` item. `CCarHandlingData::ConvertToGameUnits` (`handlingMgr.cpp:1186-1192`) does `sscanf(m_strAdvancedFlags, "%x", &aFlags)`; **`aFlags` is not a schema field** (the structdef at `HandlingInfo.psc:304-321` has no entry for it), so it is always derived from the string. The enum stops at `0x20000000` — bits 30 and 31 are unused.
**This set is not a flat flag list**: the first nine members are two *enumerated groups* (a differential type and a gearbox type) where the engine tests combinations, and the rest are independent bits.

| bit | engine name | header comment | consumer → meaning |
|---|---|---|---|
| `0x00000001` | `CF_DIFF_FRONT` | — | `wheel.cpp:5362`, `:8284`, `Automobile.cpp:9757` — front diff defers the wheel's rot-vel clamp and feeds the torque split |
| `0x00000002` | `CF_DIFF_REAR` | — | the same three sites, rear axle |
| `0x00000004` | `CF_DIFF_CENTRE` | — | `wheel.cpp:6550` — drive force split evenly for the diff to resolve |
| `0x00000008` | `CF_DIFF_LIMITED_FRONT` | — | `wheel.cpp:8325` — limited-slip lock ratio from the two wheels' rot speeds |
| `0x00000010` | `CF_DIFF_LIMITED_REAR` | — | `wheel.cpp:8323` |
| `0x00000020` | `CF_DIFF_LIMITED_CENTRE` | — | `Automobile.cpp:9762` — sets `hasViscousCoupling` |
| `0x00000040` | `CF_DIFF_LOCKING_FRONT` | — | `wheel.cpp:8318` — `lockRatio = 1.0` |
| `0x00000080` | `CF_DIFF_LOCKING_REAR` | — | `wheel.cpp:8316` |
| `0x00000100` | `CF_DIFF_LOCKING_CENTRE` | — | **inert** — nothing reads it |
| `0x00000200` | `CF_GEARBOX_FULL_AUTO` | — | **inert** |
| `0x00000400` | `CF_GEARBOX_MANUAL` | — | `Transmission.cpp:700`, `:1049` — no auto-clutch creep; under braking, revs follow speed only when *not* manual |
| `0x00000800` | `CF_GEARBOX_DIRECT_SHIFT` | — | **inert** |
| `0x00001000` | `CF_GEARBOX_ELECTRIC` | — | **inert** |
| `0x00002000` | `CF_ASSIST_TRACTION_CONTROL` | `// JUST REDUCE THROTTLE` | **inert** — verified across the whole tree, see the note below |
| `0x00004000` | `CF_ASSIST_STABILITY_CONTROL` | `// APPLY BRAKES TO INDIVIDUAL WHEELS` | **inert** — same |
| `0x00008000` | `CF_ALLOW_REDUCED_SUSPENSION_FORCE` | `// Reduce suspension force can be used for "stancing" cars` | `wheel.cpp:3511` — static suspension force drops to `sfSuspensionHealthSpringMult2` |
| `0x00010000` | `CF_HARD_REV_LIMIT` | — | `vehicle.h:942` via `Transmission.cpp:1151` — hard rev limiter (suppressed for tuners unless in top gear); force-set at `Automobile.cpp:4941` |
| `0x00020000` | `CF_HOLD_GEAR_WITH_WHEELSPIN` | — | `wheel.cpp:5281`, `Transmission.cpp:817` — keeps full slip ratio and blocks upshifts while wheel speed exceeds vehicle speed by 20% |
| `0x00040000` | `CF_INCREASE_SUSPENSION_FORCE_WITH_SPEED` | — | `wheel.cpp:477`, `:3489` — suspension force rises with speed |
| `0x00080000` | `CF_BLOCK_INCREASED_ROT_VELOCITY_WITH_DRIVE_FORCE` | — | `wheel.cpp:5238` — outside a burnout, drive force no longer raises rot velocity |
| `0x00100000` | `CF_REDUCED_SELF_RIGHTING_SPEED` | — | `Automobile.cpp:1246` — in-air rotation limit switches to the reduced constant |
| `0x00200000` | `CF_CLOSE_RATIO_GEARBOX` | — | `Transmission.cpp:2136` — first gear ratio scaled by `sfCloseRatioGearboxFirstGearRatio` |
| `0x00400000` | `CF_FORCE_SMOOTH_RPM` | — | `Transmission.cpp:1112`, `:1251` — smoothed idle floor and rev rate |
| `0x00800000` | `CF_ALLOW_TURN_ON_SPOT` | — | `Automobile.cpp:3992-4106`, `Transmission.cpp:2238` — low-speed turn-on-spot torque, reverse creep |
| `0x01000000` | `CF_CAN_WHEELIE` | — | `Automobile.cpp:1454` — joins the wheelie-capable set with muscle cars and the Tornado6 |
| `0x02000000` | `CF_ENABLE_WHEEL_BLOCKER_SIDE_IMPACTS` | — | `wheel.cpp:1510` — wheel blockers collide with the ground on side impacts |
| `0x04000000` | `CF_FIX_OLD_BUGS` | — | `wheel.cpp:1354`, `:3424`, `:3503`, `:7866` — rim radius, static delta, spring clamp and slip averaging all take the corrected path |
| `0x08000000` | `CF_USE_DOWNFORCE_BIAS` | — | `wheel.cpp:7536`, `VehicleModelInfo.cpp:1409` — **per-axle downforce, and it changes what the traction natives return** (see the note) |
| `0x10000000` | `CF_REDUCE_BODY_ROLL_WITH_SUSPENSION_MODS` | — | `wheel.cpp:3621`, `:4044` — roll-centre height biased by the suspension lowering |
| `0x20000000` | `CF_ALLOWS_EXTENDED_MODS` | — | `Vehicle.cpp:31592` — `HasExpandedMods`, the gate on all `m_AdvancedData` reads |

**30 members: 24 consumed, 6 inert** (`CF_DIFF_LOCKING_CENTRE`, `CF_GEARBOX_FULL_AUTO`, `CF_GEARBOX_DIRECT_SHIFT`, `CF_GEARBOX_ELECTRIC`, and both `CF_ASSIST_*`). Commented-out code and `__BANK` debug text were *not* counted as consumers; the scan returned every one of the 30 definition lines, so the six absences are real.
**`CF_USE_DOWNFORCE_BIAS` changes the number a script reads.** `CommandGetVehicleMaxTraction` adds a downforce term only when this bit is set (`commands_vehicle.cpp:6216`, and `:6291` for the model variant, plus the model-level `VehicleModelInfo.cpp:1411`) — so **the same car with the same handling values answers a different max-traction value depending on one bit.** ARS divides that term out by its own formula, which makes it immune, but any comparison of ARS's grip against the raw native must account for it.

## The advanced data: `CCarHandlingData` and `CAdvancedData`

`CCarHandlingData` (`handlingMgr.h:688-718`) — **15 members, every one read somewhere** (12 floats plus the string, the derived `aFlags`, and the array):

| field | schema name | consumer → meaning |
|---|---|---|
| `m_fBackEndPopUpCarImpulseMult` | :305 | `physics.cpp:2734` — vertical impulse when rear-ending another car |
| `m_fBackEndPopUpBuildingImpulseMult` | :306 | `physics.cpp:2739` — the same against non-car geometry |
| `m_fBackEndPopUpMaxDeltaSpeed` | :307 | `physics.cpp:2741` — caps that impulse by mass |
| `m_fToeFront` | :308 | `wheel.cpp:6456-6469` — toe applied to steering wheels |
| `m_fToeRear` | :309 | `wheel.cpp:6457-6481` — steer angle of the non-steering wheels |
| `m_fCamberFront` | :310 | `wheel.cpp:697` (contact point), `WheelRendering.cpp:331` (rendered) |
| `m_fCamberRear` | :311 | the same two sites, rear |
| `m_fCastor` | :312 | `WheelRendering.cpp:341` — **rendering only**, no physics reader found |
| `m_fEngineResistance` | :313 | `Transmission.cpp:1298` — **part-throttle** drive-force loss (a low-throttle deadband); dead at zero throttle, where the clamp bound is the throttled force and `:1261` makes that zero |
| `m_fMaxDriveBiasTransfer` | :314 | `Automobile.cpp:9814-9826` — clamps the front/rear torque split; the `handlingMgr.h:1001-1003` inlines read it where **`> -1` means all-wheel drive** |
| `m_fJumpForceScale` | :315 | `Vehicle.cpp:37503` — scales the jump impulse |
| `m_fIncreasedRammingForceScale` | :316 | `vehicle.h:655` → `Vehicle.cpp:21123` — scales the shunt force on the other car |
| `m_strAdvancedFlags` | :317 | parsed only by its own `ConvertToGameUnits` — the source of `aFlags` |
| `aFlags` | **not in the schema** | derived; ~40 read sites covering the 24 live flags above |
| `m_AdvancedData` | :318-320 | `Automobile.cpp:1543` (wheelie torque), `Transmission.cpp:150-198` (turbo power from VMT_KNOB, max-turbo scan), `VehicleModelInfoVariation.cpp:875-1057` (VMT_ICE top speed, front/rear downforce) |

`CAdvancedData` (`handlingMgr.h:229-239`) — one entry per mod slot, **3 members, all read**:

| field | schema name | meaning |
|---|---|---|
| `m_Slot` | :93 | the mod slot the value applies to (`-1` = any slot) |
| `m_Index` | :94 | the mod index, compared against the applied mod |
| `m_Value` | :95 | the value applied when that mod is fitted |

**18 of 18 fields are live** — no dead member in either struct. One provenance limit worth knowing: **the literal `<...>` tag spelling in `handling.meta` cannot be traced from the source** (the schema's `name=` attributes are C++ member names and there is no `.meta` in the tree). The spellings quoted throughout this file come from the live file itself, cross-checked against the schema's member names — with `<strFlags>` the one *inferred* spelling, since no car in this fleet carries it.

## ARS relevance — what these flags mean for this project

- **Read by ARS**: only `HF_OFFROAD_ABILITIES_X2` and `HF_OFFROAD_INCREASED_GRAVITY_NO_FOLIAGE_DRAG` (`Racer.cs:469-473`), and the ×1.2 they carry is the engine's own value. `HF_OFFROAD_ABILITIES` (the ×1.1 tier) is **not tested**, and the precedence trap means a car with both bits is modelled as ×1.2 where the engine gives it ×1.1. See the open item in `AGENTS.md` for the double-count that rides along with this. **Measured on this fleet: 39 of the 41 handling items carry the ×1.2 bit (`0x800000`) and none carries the ×1.1 tier alone or the `0x200000` bit**, so the untested tier is inert here and the precedence trap has no car to bite.
- **`HF_HAS_RALLY_TYRES` is the one to look at next**: it switches the tyre to a curve where *grip increases with slip angle* (min/max terms swapped, `wheel.cpp:3167`). Every ARS grip law assumes a peak at `fTractionCurveLateral` and falling force beyond it, so a rally-tyre car in the roster would be modelled with an inverted curve. Worth scanning the fleet's flags for `0x8` the same way the off-road bits were scanned. **Measured on this fleet: RETINUEL is that car** (`0x82000A`; the only one of the 41 handling items with bit `0x8`), so exactly one grid car is modelled with an inverted grip curve today.
- **Steering geometry**: `HF_STEER_REARWHEELS` / `HF_STEER_ALL_WHEELS` / `HF_STEER_NO_WHEELS` change which axle answers the command. The whole steering chain, the Ackermann ceiling and the wheelbase read assume front steering, so a car with any of these three is outside the model.
- **`HF_FORCE_NO_TC_OR_SC`** removes the bike's hidden TC/SC; the bikes that lack it get that built-in assistance for free, which is the AI-vs-player asymmetry already noted in `AGENTS-VANILLA-STEERING.md`.
- **`HF_TYRES_CAN_CLIP`** explains a car that looks sunk or clips its tyres: it raises the compression cap per model (`wheel.cpp:1359`). It is not a defect and not a player-vs-AI difference.
- **`HF_NO_HANDBRAKE`** (no handbrake force at all) and **`HF_CVT`** (a different drive-force path) are both outside what ARS assumes about pedal response and gearing.
- **`HF_HAS_KERS`** is a boost the engine only grants in multiplayer — relevant to the same fair-play question as nitro, though the flag is gated on `IsGameInProgress()`.
- **The fleet's actual flag reality, measured** (from `Sideload\0_vsl-handling\handling.meta`, 126 `<Item>` entries of which **41 are `CHandlingData` cars** — the rest are NULL or sub-handling blocks): all 41 carry `strModelFlags`, `strHandlingFlags` and `strDamageFlags`; only 16 carry `strAdvancedFlags`; **none carries `strFlags`**, so no special-flight set is in play. **39 of the 41 cars carry the ×1.2 off-road gravity bit** — nearly the whole grid, not a niche subset — so the double-count fix in `AGENTS.md` retunes almost every car's cornering speed by ~9.5% and its braking, and deserves its own drive rather than riding along with another change.
- **Damage flags are racing-irrelevant** (they only stop body panels being torn off), but worth knowing when reading the file: only `DF_BOOT` is used in this fleet, on LEMONEURO, NODE_G35 and SLGT.
- **Model flags are a two-set trap, and ARS sits on the right side of it by accident.** The meta's `strModelFlags` is the `MF_*` set (all 41 fleet cars: `MF_ABS_STD` plus an axle pair — nothing exotic), while the interesting model behaviour lives in the 204-member `CVehicleModelInfoFlags` set, which is *model metadata* and not in `handling.meta` at all. ARS reads neither: its one model-level read is `FLAG_IS_ELECTRIC` (**bit 43**) through the `GET_IS_VEHICLE_ELECTRIC` native (`0x1FCB07FE230B6639`, `VehicleCatalog.cs:108`, with availability probing in `AutosportRacingSystem.cs:106-115`) — so "ARS reads no model flags" is true of the *flag fields* and false of model properties in general.
- **`FLAG_SPOILER_MOD_DOESNT_INCREASE_GRIP` (bit 194) is a real modelling gap.** The traction native ARS reads branches on it (`commands_vehicle.cpp:6221`), but ARS divides the native's whole downforce term out and models downforce itself from the handling field plus speed — so on a car carrying this flag ARS grants downforce the engine deliberately refuses. Worth checking whether any roster car has it, since it is model metadata and cannot be seen in `handling.meta`.
- **`FLAG_DONT_TIMESLICE_WHEELS` (bit 79)** disables the wheel time-slicing that the earlier wheel investigation found applied to AI cars while upright (`Automobile.cpp:4081`, AI path `:4202`). It is per-model, so for a car carrying it there is no AI-vs-player wheel difference to reason about.
- **Wheel and suspension presentation**: `FLAG_DROP_SUSPENSION_WHEN_STOPPED` (bit 157) lowers the car at rest and `FLAG_RENDER_WHEELS_WITH_ZERO_COMPRESSION` (bit 201) renders wheels uncompressed regardless of suspension — both matter when judging whether a car "looks sunk", which is otherwise a `HF_TYRES_CAN_CLIP` question.
- **Boost and braking flags are a fairness and model-accuracy question for the grid**: `FLAG_HAS_NITROUS_MOD` (174), `FLAG_HAS_ROCKET_BOOST` (126), `FLAG_HAS_JATO_BOOST_MOD` (148) and `FLAG_HAS_VERTICAL_ROCKET_BOOST` (161) mark cars with a built-in boost ARS does not know about — the same class of asymmetry as `HF_HAS_KERS` — while `FLAG_HAS_SUPER_BRAKES_MOD` (177) marks cars whose braking is better than the grip-based model predicts.
- **Two roster hazards**: `FLAG_CANNOT_BE_DRIVEN_BY_PLAYER` (bit 60) would make a grid car undrivable, and `FLAG_INCREASE_LOW_SPEED_TORQUE` (138) / `FLAG_DONT_HOLD_LOW_GEARS_WHEN_ENGINE_UNDER_LOAD` (136) change the acceleration the pace model assumes.
- **The engine has no traction or stability assist at all** — `CF_ASSIST_TRACTION_CONTROL` and `CF_ASSIST_STABILITY_CONTROL` are **inert**, verified across the whole tree, despite carrying the comments "JUST REDUCE THROTTLE" and "APPLY BRAKES TO INDIVIDUAL WHEELS". So ARS's TCS is not fighting an engine assist on any car; the only assists in play are the bike cheat gated by `HF_FORCE_NO_TC_OR_SC` and the AI-side `STATUS_PHYSICS` ABS.
- **`CF_USE_DOWNFORCE_BIAS` decides what the traction native answers** — with it set, `GET_VEHICLE_MAX_TRACTION` adds a downforce term (`commands_vehicle.cpp:6216`), so the raw native is **not comparable between cars** without checking the bit. ARS divides that term out with its own formula, which makes it immune; only an outside comparison needs the caveat.
- **`m_fMaxDriveBiasTransfer`** is the field behind ARS's AWD exemption — the header inlines read it as `> -1` means all-wheel drive (`handlingMgr.h:1001-1003`), which is worth naming where that rule is documented.
- **`m_fEngineResistance` is a part-throttle loss, not lift-off engine braking** — it multiplies the base drive force by revs × clutch × `(1 - |throttle|)` and is clamped to the *throttled* force, which `Transmission.cpp:1261` makes zero at zero throttle, so lift-off deceleration never comes from it; an earlier note here called it engine braking and was wrong about when it acts. **The real lift-off deceleration is off-gas wheel friction** (`wheel.cpp:3838-3840`, using the `fFrictionMult` chosen at `:3797-3814`) — exactly what `HF_FREEWHEEL_NO_GAS` cuts — plus material/extra wheel drag and aero drag; ARS models none of them, so off-throttle behaviour still differs per car.
- **Camber and toe are authored, and the traction native does not include them.** `m_fCamberFront`/`m_fCamberRear` move the contact geometry (`wheel.cpp:697`) and `m_fToeFront`/`m_fToeRear` steer the axles statically (`wheel.cpp:6456`), while ARS's grip comes from the native — so a car with meaningful camber (the `MF_EXTRA_CAMBER` flag exists for exactly this) is modelled without it. `m_fCastor` is rendering-only.
- **`m_AdvancedData` is the mechanism by which fitted mods change performance** — per-slot, per-index values for turbo power (`VMT_KNOB`, `Transmission.cpp:150-198`), top speed (`VMT_ICE`, `VehicleModelInfoVariation.cpp:875-894`) and front/rear downforce (`:1021-1057`). It is the data an upgrade-aware pace model would read, which makes it the concrete answer to the "pace is model-theoretical, stock and fully upgraded score identically" open item in `AGENTS.md`.
