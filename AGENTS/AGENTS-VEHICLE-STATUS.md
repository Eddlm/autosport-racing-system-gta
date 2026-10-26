# GTA V — `STATUS_PLAYER` vs `STATUS_PHYSICS`, site by site

**What this is**: every place the engine branches on a vehicle's `m_nDEflags.nStatus` between `STATUS_PLAYER` and `STATUS_PHYSICS`, extracted from the leaked tree at `E:\GTA\GTAVSP\GTAV Source\src\dev_ng\game` (117 occurrences, 2026-10-24). It exists because the two statuses are **not** interchangeable and several of the differences are invisible until a car behaves unlike its handling values.

**The rule that makes all of this matter**: a vehicle is `STATUS_PHYSICS` from construction (`Vehicle.cpp:1262`) and becomes `STATUS_PLAYER` when a **player** takes the driver's seat (`Ped.cpp:28993`) or the player ped is set up in it (`PlayerInfo.cpp:1548`). Leaving the seat sets it back to `STATUS_PHYSICS` (`PlayerInfo.cpp:1430`, `PlayerPed.cpp:226`). It is **event-driven, never re-asserted per tick** — except for the consequences below that ARE re-asserted per tick.

## The ones that change how the car drives

| Site | `STATUS_PLAYER` | `STATUS_PHYSICS` | Why it matters |
|---|---|---|---|
| `Automobile.cpp:3775` | ABS only if the handling flags grant it | **`WF_ABS` granted every tick** | A player's car can be made ABS-free from `strModelFlags`; an AI car **cannot** — see `AGENTS-FLAGS.md`. |
| `Automobile.cpp:1017` `ProcessDriverInputsForStability` | Called (asserts PLAYER) | Not called | The stability correction on driver inputs is player-only. |
| `Automobile.cpp:3630` | Player gets **in-air control** and the chance to roll back onto its wheels after a crash | Neither | AI cars land dead. |
| `Bike.cpp:1900`, `Bike.cpp:1601` | Lean and jump variations read the player path | AI path | Bike lean is authored for the player. |
| `Heli.cpp:6175`, `Planes.cpp:3807`, `5738`, `6192`, `Vehicle.cpp` control reads | Reads `CControl` (raw stick input) | Reads the AI task | The control **source** switches, not just a flag. |
| `Planes.cpp:4177` | On destruction the player keeps control | Set out of control | |
| `Transition.cpp`/`Transmission.cpp` rev-limit debug | debug only | — | |

## Damage and collisions

| Site | `STATUS_PLAYER` | `STATUS_PHYSICS` | Why |
|---|---|---|---|
| `vehicleDamage.cpp:8524` | Gas-pedal threshold **0.5** | **0.2** | AI cars take damage at a much lighter throttle — comment says so outright. |
| `vehicleDamage.cpp:2255` | Hitting **a player's car** does more damage | Normal | |
| `vehicleDamage.cpp:3817` | Status tested for damage routing | — | |
| `scene/Physical.cpp:5313`, `DynamicEntity.h:247` `TreatAsPlayerForCollisions()` | **true** | false | Collision response differs for the player's vehicle. |
| `PedEventScanner.cpp:6382 / 6563 / 6697` | Bike/quad/jet-ski upside-down KO uses the **player** multiplier | AI multiplier | Different threshold to knock the rider off. |
| `WeaponDamage.cpp:2558` | Damage attribution to the player | — | |

## World, spawning, AI awareness

| Site | `STATUS_PLAYER` | `STATUS_PHYSICS` | Why |
|---|---|---|---|
| `vehiclepopulation.cpp:1685`, `cargen.cpp:1525` | **Never culled** | Can be removed by the population manager | An AI car can vanish; the player's cannot. |
| `VehicleIntelligence.cpp:849-850` | Crimes are **not** reported for the player's car | AI cars report crimes | |
| `TaskVehicleGoToAutomobile.cpp:6781` | — | `STATUS_PHYSICS` counts as a car to stop for | |
| `junctions.cpp:5127` | Handled in the same switch branch as PHYSICS | ditto | One of the few places they agree. |
| `PlayerInfo.cpp:7075`, `PlayerPed.cpp:808` | Excluded (`!= PLAYER && != PLAYER_DISABLED`) | Included | |

## Sets the status

| Site | Direction |
|---|---|
| `Vehicle.cpp:1262` | → `PHYSICS` at construction |
| `Ped.cpp:28993` | → `PLAYER` when a player sits in the **driver** seat |
| `PlayerInfo.cpp:1548` (`SetPlayerPed`) | → `PLAYER` when the player ped is (re)assigned and is in a vehicle |
| `commands_vehicle.cpp:2772` | → `PLAYER` from a script command |
| `Vehicle.cpp:13544/13547` | Wrecked car with a driver: player → `PLAYER`, otherwise → `PHYSICS` |
| `PlayerInfo.cpp:1430`, `PlayerPed.cpp:226` | → `PHYSICS` on leaving the vehicle |
| `Vehicle.cpp:30877` `SetIsPlayerDisabled` | → `PLAYER_DISABLED` |
| Replay interfaces (`ReplayInterfaceVeh/Pickup/Obj.cpp`) | → `PLAYER` for replay playback |

## Debug, audio and VFX — no gameplay effect

`__BANK` / `__DEV` blocks that only draw speedos, force limits or handling vectors: `Bike.cpp:833/2713`, `boat.cpp:1089`, `Heli.cpp:830/846/3265/6128/6138/6175/6394`, `Planes.cpp:2120/5758/6210`, `Transmission.cpp:414/490`, `Vehicle.cpp:26453/26656/26699`, `vehicleDamage.cpp:9291`, `VehicleGadgets.cpp:6855/6890`, `VfxWheel.cpp:910/1175`, `wheel.cpp:4603`, `Floater.cpp:1059`, `VehicleDebugInfo.cpp:685` (the status **name** printer), `Automobile.cpp:969/2368/4861/6097/10178`.

Audio: `caraudioentity.cpp:5090/5102/5116`, `boataudioentity.cpp:654`, `environmentgroup.cpp:446` — engine-audio debug and volume grouping by "is this the player's vehicle".

VFX: `VfxVehicle.cpp:3046` (heli effects gated on `bDrivenByPlayer && STATUS_PLAYER`), `PortalTracker.cpp:2575` (occlusion), `door.cpp:2590` (first-person view), `TaskVehiclePlayer.cpp:3788` (crop-duster smoke in MP).

## The one to remember

**Nothing re-asserts `STATUS_PLAYER`.** `Ped.cpp:28993` fires when the player sits down and `PlayerInfo.cpp:1548` when the ped is set up; after that the status holds until the vehicle is wrecked, abandoned, or the player leaves. So a player's car keeps whatever the handling flags said about ABS, while an AI car re-earns `WF_ABS` **every tick** from `Automobile.cpp:3775`.
