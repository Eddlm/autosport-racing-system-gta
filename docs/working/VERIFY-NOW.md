# VERIFICAR EN JUEGO — pendiente

Build desplegada: **1802**. Lo marcado ✅ es tuyo, verificado en juego; lo marcado ⬜ sigue siendo **compile-only**.
Marca una línea con `[x]` o dime «verificado N» y actualizo este fichero.

**Lo de hoy está en la sección N**, arriba del todo: Nearby Cars, builds 1784 → 1802, todo compile-only.
Recuento: **44 abiertas + 6 nuevas**. El detalle de la caza del 30-10 sigue en `docs/audit/410-hunt-2026-10-30.md`.

---

## N · 2026-11-13 (1784 → 1802) — Nearby Cars

Ten builds since the section below was written; every row here is **compile-only**.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| N1 | **the cars around you join the grid** (1784) | Nearby Cars on, one empty car in range, Start Race | it joins the grid and runs with the pack, not as a prop | ⬜ |
| N2 | **adoption runs after the pool, and a ghost seat gets a driver** (1790) | a nearby car whose driver handle does not exist | it is adopted anyway, a driver is spawned beside it, and the log's `ghostSeat` count rises | ⬜ |
| N3 | **the toggle is a three-way choice** (1794) | switch Ignore → Join As Is → Join And Tune | Ignore draws nothing and adopts nothing; Join As Is adopts and leaves Auto Styling alone; Join And Tune lets the tuner see them | ⬜ |
| N4 | **adopted cars are sorted, not pinned** (1799) | adopt mid-field, then look at the grid | they take their place by pace with the rest; nobody is nailed to a slot | ⬜ |
| N5 | **the chevrons share one angle** (1800, 1802) | a track start and a nearby car in the same view | both chevrons face the same way: one constant, `TrackChevronRotation` | ⬜ |
| N6 | **the chevron says which car is adoptable** (1801) | one healthy empty car and one unhealthy one in range | blue for race-fit, orange for not; an **occupied** car draws nothing — a corpse in the driver's seat counts as occupied | ⬜ |

---

## L · línea de carrera (1301 → 1312)

| # | Qué | Cómo | Pasa si | Estado |
|---|---|---|---|---|
| L1 | **fase por distancia** (1302) | curvas en V y en U, observando la línea | ya no acelera en los últimos nodos | ✅ **verificado 28-10 · 1302** |
| L2 | **lean por ancho/radio** (1308) | pista estrecha y pista ancha | en la ancha ataca el interior con más ritmo, sin volantazo | ✅ **verificado 28-10 · 1308** |
| L3 | **out-in eliminado** (1312) | cualquier curva | el naranja del exterior sigue en la aproximación, **sin** magenta, y la entrada se siente igual que en 1308 | ✅ **verificado 28-10 · 1312** |

## A · Sin conducir (log)

| # | Qué | Cómo | Pasa si | Estado |
|---|---|---|---|---|
| A2 | **settings atómicos** | cambia una fila; luego pon el ini **read-only** y cambia otra | el valor persiste; con el fichero bloqueado sale un `Error` en el log en vez de perderse | ⬜ |
| A3 | **304 · parrilla vacía** | `Max PI Deviation` bajo para no tener a nadie en banda, Start Race | **no arranca**, dice `No car within N PI of X`, y **no quedan bengalas** | ✅ **verificado 26-10** |
| A4 | **preflight** | mismo caso que A3, mirando el log | aparece `Start Race refused: …` **antes** de instanciar | ✅ **verificado 26-10 (1124)** |
| A5 | **layout · nombres** | arranca una carrera | los rivales llevan nombre (`Drivers\names.txt`) y salen tips | ✅ **verificado 26-10** |
| A6 | **roster nuevo** (1169) | cambia el roster en Rivals a `cars.txt` y pide parrilla | indexa **643 modelos** y monta la parrilla. Antes se colgaba (o eso parecía) al cambiarlo | ⬜ |
| A7 | **log rotado** (1171) | arranca dos veces seguidas | existe `ARS.prev.log` en la **raíz del juego** con la sesión anterior | ⬜ |

## B · Una vuelta normal

| # | Qué | Cómo | Pasa si | Estado |
|---|---|---|---|---|
| B1 | **costura de curvas** | **El Burro Rally** con `LogLevel = Info` | el primer `Corner add` da un `start=` sensato y el nitro **no** está vetado toda la vuelta | ✅ **verificado 26-10**: 14 curvas en El Burro / 18 en Oct1, `apex=0` solo en el `skipped`, y frena la primera curva |
| B2 | **brake learner** (1126) | una entrada pasada de frenada con `Brake Learn Notify` | el aviso **sale sin Show Track Analysis** y el recorte es **0,15 como mucho** | ⬜ |
| B3 | **muros a tres** | Show Rivalry, un rival a cada lado | los dos muros siguen dibujados | ⬜ |
| B4 | **board · nombres** (1117) | mira la columna | **sin** paréntesis de PI, y el PI con **un decimal** | ✅ **verificado 26-10** |
| B6 | **board · posición** (1125) | mira la esquina superior izquierda | el `POS` ya no se solapa con las hints/tips | ⬜ |
| B7 | **allowance lenta** (1127) | curva de 5-15 mph con `Show Front Slip` | el coche gira y aparece el token `x1.5`/`x2.0` | ⬜ |
| B8 | **Peak Slip Target** (1128) | token `x0.9` y `F` en curva rápida | el corte empieza antes del pico (F 100 = 90 % del pico) | ⬜ |
| B9 | **Deepest Cut (%)** (1385) | sube de 60 a 80 en un derrape largo | recorta más que antes; a 0 no corta nada; a 60 idéntico a la 1384 | ⬜ |
| B10 | **la referencia es la rueda exterior** (1391, vivo en `MeasureSlipFromMemory`) | `Show Front Slip` con `Slip From Memory` **on**: gira a un lado en una curva rápida y mira el `F` del token | el número sigue a la rueda **exterior** (la de menos derrape); con el volante recto es la media de las dos | ⬜ |
| B11 | ~~Target Under Power~~ (1393) | — | aparcado en la 1396: la fila ya no existe | ⛔ |
| B12 | **el suelo de gas del carril** (1396) | curvón apuntando al **interior**, con el plan pidiendo levantar | el gas **no baja**: acelera aunque el plan quisiera levantar; pegado al borde exterior el suelo desaparece | ✅ **verificado 31-10 · 1396**: «probado y funciona genial» |
| B13 | **Brake Aim Depth** (1397) | una curva donde ya frenaba, subiéndolo a 0.4 y luego a 0.8 | frena **más tarde** y entra con más velocidad; si a las pocas vueltas vuelve a frenar donde antes, es el aprendiz bajando el factor | ⬜ |
| B14 | **Car-Centred Radius** (1399) | Figureight entre ápex 2 y 3, una vuelta con el toggle **off** y otra **on** | off se sale igual que ahora; on aguanta la curva hasta salir de verdad, y **no** frena antes en la entrada | ✅ **verificado 31-10 · 1400**: «lo tenemos, esto ha ayudado» — falta confirmar que la entrada no se ha vuelto más lenta |
| B15 | **Slid Radius Lead + toggle del radio** (1404) | el mismo tramo: toggle **off**, luego lead 0.0 / 0.5 / 1.5 | off = reacciona tarde a la entrada; 0.0 = mide sólo lo recorrido, el más cauto al salir; 1.5 = casi el mismo número que el principal | ✅ **verificado 31-10 · 1404** |
| B16 | ~~carril inactivo se planta~~ (1408) | — | **descartado en 1409**: el umbral `LaneCrossMinMeters` convertía una cantidad continua en un estado binario y parpadeaba. Se quitó el bloque; el carril vuelve por el mover, como antes | ❌ descartado |
| B17 | **cierre exponencial** (1132) | `Slip From Memory` **off**: el readout es la proporción | a 500 % llega a 100 % en ~0,27 s (8 ticks); a 1000 % ~0,3 s | ⬜ |

## C · Abuso deliberado

| # | Qué | Cómo | Pasa si | Estado |
|---|---|---|---|---|
| C1 | **nitro al terminar** | dispara y **End Race dentro de los 3 s** | tu coche no sale con la botella armada; destildar Nitrous en pleno burn **para** el empuje | ⬜ |
| C2 | **recarga** | **Insert** en plena carrera | no crashea, la carrera desaparece, pedal neutro | ⬜ |
| C3 | **carrera tras carrera** | termina una y lanza otra antes de que expire el «Ns to end» | la nueva **no muere en sus semáforos** | ⬜ |
| C4 | **creador** | línea de salida + **una** sección, guardar | se niega y dice cuántos nodos faltan, sin pantalla negra | ⬜ |
| C5 | **solitario corto** | **Grid Size 0** en un circuito corto | arranca | ⬜ |
| C6 | **freecam** | actívala y **entra en un coche** | la freecam **sale sola** | ⬜ |

## D · Decisiones y papeleo

| # | Qué | Notas | Estado |
|---|---|---|---|
| D1 | **ABS desbloqueado** (1114) | fila viva, arranca con el valor guardado. **Falta la descripción nueva** que ibas a darme | ⬜ |
| D2 | **`Oprahs`** en `names.txt` | ¿se queda, se convierte en `Oprah` o se va? | ⬜ |
| D3 | **Tag `v0.9.0`** | **ya estaba cortado**: release publicado el 27-10 con el build 1293 y el zip de ese día, etiqueta en `1d8b91e`. El build de hoy va aparte, como `v0.9.0.1381` en borrador. `docs/audit/` son 1140 KB internos | ✅ 30-10 |
| D4 | **PI relativo a pie** (1124) | error inmediato, sin instanciar, sin teleport, sin fade | ✅ **verificado 26-10** |

## E · Lo de hoy, sin probar todavía

| # | Qué | Cómo | Pasa si | Estado |
|---|---|---|---|---|
| E1 | **off-track: 40 debajo, suelo 20** (1171) | sácate de la línea en una curva rápida | el coche baja de verdad, no 10 mph menos: una curva de 35 pide 20, una de 70 pide 30 | ⛔ superseded by I1 — no longer behind a switch, allowance now 20 mph (1569) |
| E2 | **ángulo de derrape horizontal** (1165) | curvas con **desnivel** | detecta el derrape igual en rasante que en llano; antes se inflaba con la pendiente | ⬜ |
| E3 | **chicanas** (1165) | pista con pendiente y eses | las chicanas siguen siéndolo (+10 mph); ninguna se clasifica por el desnivel | ⬜ |
| E4 | **slots de parrilla** (1165) | mira los coches en la salida | ninguno hundido ni flotando en pistas con pendiente | ⬜ |
| E5 | **el dither del volante** | ya no hay que probarlo | **resuelto** en la 1163: era la demora medida en 3D. Los intentos fallidos están en `AGENTS-BACKLOG.md` | ✅ |
| E6 | **aire en offroad** | no implementado | la señal existe y es fiable (`ARS.WheelsOffGround`, 0-4 ruedas leyendo `0x1C4`), pero solo alimenta el factor de estabilidad. Aparcado | ⬜ |

## F · La tanda de feedback (1294)

| # | Qué | Cómo | Pasa si | Estado |
|---|---|---|---|---|
| F1 | **gate de build** | cualquier carrera normal en la 3889 | **arranca igual** que antes: el gate solo rechaza si falta una firma | ✅ **28-10 (1294)**: la carrera corrió; el gate dejó pasar |
| F2 | **sin tormenta** | una carrera entera y `grep -c "OnTick error" ARS.log` | **0** | ✅ **28-10 (1294)**: 0 en 14 líneas de log (1,5 KB) |
| F3 | **tips vs board** | pasa un ápex con tips y leaderboard visibles | la caja del tip **no tapa** la cabecera `POS` | ✅ «todo bien», 28-10 (1294) |
| F4 | **nombre del coche** | Interface > *Show Driver Names* apagado | la cabecera dice `CAR` y sale el modelo | ✅ «todo bien», 28-10 (1294) |
| F5 | **10 min quietos, sin tormenta** (venía de A1) | deja el mod 10 min parado, sin carrera, y `grep -c "OnTick error" ARS.log` | **0**. La cifra «208» era de una sesión del 25; todas las sesiones desde entonces cierran con 0 | ⬜ |

---

### Fuera de esta lista, ya decidido

- **rival-log lag** (el viejo B5, ya sin fila) y **R12** (leaderboard por frame) → aparcados a **0.11**, en `AGENTS-BACKLOG.md`.
- **B5 · board · toggle (1112)** → cubierto por **F4** (verificado 28-10 · 1294); fila retirada el 13-11.
- **`SpeedSteerCeiling`** → zombie para **0.11**: revivir la ley o borrarla con su fila del menú.
- **`corner-chevron-exit`** → 0.9.1.
- **301** · 43 filas heredadas reales → deuda, no blocker. **303-F2** → sin tocar a propósito.
- **El crash al cambiar el roster**: causa **desconocida**. Los commits de la 1167/1169 quitan peligros reales pero no lo explican. Si vuelve, `ARS.prev.log` (en la raíz del juego) ya guarda la sesión anterior.

---

## H · La caza de bugs del 30-10 (1375 → 1380)

Seis commits, todos **compile-only**. Cada fila trae cómo provocarlo y qué debe pasar.

| # | Qué | Cómo | Pasa si | Estado |
|---|---|---|---|---|
| H1 | **pista rechazada** (1375) | Tracks → un XML con menos de 6 nodos de ruta | **no** dice «Track instanced», no arranca parrilla, sale el aviso | ✅ **verificado 30-10 · 1380** con `Chianski Long only Five.xml`: `Start Race refused: That track has 5 route nodes`, sin `Track instanced`. Ojo: la puerta que se ejercitó es el **preflight**, que ya funcionaba; la que arreglé (Load Track en el menú) sigue sin ser alcanzable |
| H2 | **el abort deja el ped en pie** (1375) | entra en freecam y luego Insert (reload) | el ped **no** queda flotando sin gravedad; la carrera siguiente arranca normal | ✅ **verificado 30-10 · 1380**: recargar en freecam ya no deja al ped flotando en ragdoll; el bug existía y ya no se puede reproducir |
| H3 | **la carga de fondo que falla avisa** (1375) | arranca y lee el log | una sola `Initialization complete`, sin `OnTick error` | ✅ **verificado 30-10 · 1380**: `build 1380`, `Initialization complete.`, y `grep -c "OnTick error"` = **0** |
| H4 | **el prune conserva comentarios** (1376) | pon `; prueba = 1` dentro de `[MENU]` de `Menu-Race.ini` y recarga | el comentario **sobrevive** a la carga | ❌ **falla 30-10 · 1380**: la línea desaparece. El prune ya la conserva, pero `MenuSettings.Save()` reescribe el fichero como `[MENU]` + claves declaradas en el primer `Set()`, y la carga hace media docena antes de que lo mires. Es D-T2-3, archivado; el pase lo verifiqué yo mal: comprobé el prune, no el fichero |
| H5 | **el prune dice lo que borra** (1376) | añade `ClaveInventada = 1` en `[MENU]` y recarga con `LogLevel = Error` | `ARS.log` trae `dropped stale` | ✅ **verificado 30-10 · 1380**: `Settings repair: Menu-Race.ini dropped stale ClaveInventada`, con el nivel de log por defecto |
| H6 | **restart sin carta heredada** (1377) | con una maniobra activa (etiqueta del rival visible), Restart Race | la carrera nueva **no** arrastra el tope de gas de la carta vieja | ✅ **verificado 30-10 · 1380**: «las plays desaparecen y se juegan en la nueva carrera de manera normal» |
| H7 | **Restart tras Load Track** (1378) | carrera → Load Track → Restart Race | **no** lanza `ArgumentOutOfRangeException`; arranca o se niega limpiamente | ⛔ **no alcanzable**: `Load Track` y `Build Grid` sólo se añaden al menú con `StagedSpawns`, que la carga fuerza a `false` (`:3432`). El agujero es real y queda cerrado, pero hoy no hay puerta. Para 0.11 / 0.9.1 |
| H8 | **el handbrake al lanzar** (1378) | salida parada, mira los primeros metros | ningún coche sale clavado 0,2–0,6 s; el pelotón sale junto | ✅ **verificado 30-10 · 1380**: «todos salen bien» |
| H9 | **el carril en pendiente** (1379) | tramo con desnivel, a velocidad | se comporta **igual** que en llano; el cambio es < 1 cm y no deberías notarlo | ⬜ pendiente: no encontró el tramo todavía |
| H10 | **clamp tras derrape + Restart** (1380) | derrapa, Restart Race, primera curva | gira como una carrera nueva, no con el volante ya cerrado | ✅ **verificado 30-10 · 1380**: «no veo problema» |

### Lo que espera decisión tuya (no tocado a propósito)

1. **`No Collision` está invertido** en los tres puntos de llamada: el ajuste no hace nada y su limpieza apunta al revés. `docs/audit/410-hunt-2026-10-30.md` → D-T4-1.
2. **«Open Up on Throttle»: retirado del árbol** (1389) — la fila, la clave y el método han desaparecido; ya nunca llegaba a la rueda. La fórmula vive en el backlog (`open-up-inert`) por si vuelve. D-T6-1.
3. **`LaneMoverSpeedCapMps` = 99** donde el comentario y el diseño dicen 24: el tope del latigazo no puede morder. D-T5-4.
4. **`avoid-hold-timeout`** (te debía la prueba) vive detrás de `avoidHoldEnabled = false`: la prueba apunta a código muerto. D-T5-2.
5. **Tooltip del ABS**: «I'm working on undoing that» en una fila que sí funciona. D-T7-1.
6. **`FEATURES.md` vende el nitro del jugador como «coming soon»** y el de la IA como porcentaje del tuyo: el tuyo corre en X/duck, y la IA lleva 5× contra 1×. D-T8-1, D-T8-2.

## I · 2026-11-02 (1569)

| # | What | How | Passes if | State |
|---|---|---|---|---|
| I1 | **Projection brake floor** (`ProjectionBrakeFloorMph`) | go wide in a fast bend with `Show Gs Projection` on and read the `Offshoot:` log line | `floor=` moves with the bend — 20 in a tight corner, higher in a fast one — and the car **keeps rolling**: the projection never holds a full brake with the speed under its own floor | ⬜ |
| I2 | **`ars` cheat** | type `ars` in the console | the ARS menu opens; `arsmenu` no longer does | ⬜ |
| I3 | **lane timings: identity, two clocks, and no glide** | a lap | the absolute ends when the lane preview crosses the apex and the request **snaps** to the curvature ask (the 1 s and 3 s glides are gone); the next corner's preparation does not claim the car on the apex | ⬜ |
| I4 | **apex merge raised 5 s → 6 s** | load a track with a close pair of corners, read `Apex table: N corners` | the pair now merges into one corner (count drops by one) and the second no longer runs dynamic-only | ⬜ |
| I5 | **drive kept off track** (`OffTrackThrottleLevel` 0.35, now a **floor**) | put a car well past the edge at 40+ mph and let it go | off the track the pedal cannot be cut below the ramp by TCS/stability/plan: the car keeps rolling and only arms the reverse recovery if it is genuinely stopped | ⬜ |
| I6 | **projection is a speed offset** (`ProjectionSlowdownMphPerSecond` 2, floor 30) | run wide on the approach to a fast bend, `Show Gs Projection` on | the car slows **gradually** (2 mph/s on the target, then the pedal law brakes) instead of the old 0.25 s throttle drop + 0.8 brake, and it never goes under 30 mph or the bend's speed less 30; the `Offshoot:` log line now prints `drop=` and `floor=` | ⬜ |
| I7 | **route turn-in is a pure inset** (`LaneInsetPerRadius` 125, no speed factor) | a slow corner taken fast, and the same corner taken slowly | the aim sits the **same distance inside** in both (2.5 m at R=50, 1.25 at R=100, 5 m clamp under ~25 m); it no longer widens with speed | ⬜ |
| I8 | **a corner is forgotten at the lookahead** (`ApexAheadOfLookahead` gates both selection and forgetting) | drive out of a corner on a stretch with no corner the lane will work | the lane never asks for the **outside** after the exit and never re-arms on the corner behind it; it holds the field until the apex crosses the preview, then lets go | ⬜ |

## J · 2026-11-09 (1755) — the corner pipeline

Ten bench flags all on in this build (`CornerPlan`, `RouteCurvature`, `ChicaneBoost`, `ProjectionCut`,
`OffTrackThrottle`, `CrestGate`, `StabilityCap`, `GsLaneFloor`, `OverspeedCorrection`, `SlopeGrip`). Ini:
`CornerCeiling` 500, `CornerLineShare` 20, `CornerOffset` 0, `RouteOffset` 0, `BrakeAimDepth` 0.

| # | What | How | Passes if | State |
|---|---|---|---|---|
| J1 | **the prep cap hangs off where the prep started** (47e9efd) | the bend where the pack used to be thrown off the road | no sudden steer-out: the creep is a fixed distance from the offset the prep began at, not a lead re-based on the car every tick | ✅ **verified 09-11 · 1755**: «Oh yeah much smoother. The same point cars got whacked off the road by their sudden steer out, they dont anymore.» |
| J2 | **release at the apex, four seconds to position** (1e8e57c) | the same pass, on the exit | the held corner lets go at the apex and the dynamic lane takes over, no fight against the shaped exit | ✅ 09-11 · 1755, same pass as J1 |
| J3 | **full brake asks for a 4.5 m/s error** (fe71f53) | overspeed a bend by ~5 mph, then ~10 | the pedal stays off full while the error is under 4.5 m/s (≈10 mph); the learner band, the projection share and the ramp shift with it | ⬜ |

Open on this bench: the exit asks for nothing outward — the dynamic lane only ever aims inside, so the car
leaves a corner on the inside line unless `DefendLane` is working it (his ruling: the exit is fluid, default
outward). A span holding two corners of different size is refused at the 1.6 radius ratio, not split. Grapeseed
2968 sits at 1.97.

## K · 2026-11-09 (1768) — the corner is spent when the car coasts

| # | What | How | Passes if | State |
|---|---|---|---|---|
| K1 | **coasting inside the corner releases it early** (`CornerSpent`) | brake into a bend as usual, then roll through the entrance off both pedals | once past the entrance with `long G` under 0.05 the corner leaves the selection: the line stops shaping and the plan moves on, instead of holding it to the apex | ✅ **verified 09-11 · 1769/1770**: «the 0.05gs thing works well» |
| K2 | **a lift on the straight releases nothing** | coast a hundred metres before a bend, both pedals up | the corner stays selected and the shape still starts at the entrance four seconds out — the release is gated on being past the entrance | ⬜ |
| K3 | **the input gate is gone** | trail brake lightly into a bend | the pedals no longer matter: only the longitudinal Gs decide, so a light brake above 0.05 g keeps the corner selected | ⬜ |
| K4 | **the lane lets go on its own corner** | the shaped line through a bend taken off the pedals | the shape stops once the car is through the entrance instead of running to the apex: the lane tests the corner it holds, because the queue's head has already moved on by the time steering runs | ⬜ |
| K5 | **the creep is a clock, not an angle** (1770) | the S where a left corner is approached through a right-hander | the cap opens at 0.1 m and reaches 2 m three seconds after the corner starts being asked, so a counter-turning prep no longer gets the tightest creep for being perfectly placed | ⬜ |

The signal is `VehicleData.GetLongitudinalGs`, the same one `UpdateCornerRequirements` already reads.
`CoastSpentGs` is the only knob (0.05 g ≈ 0.49 m/s², so the tail of a soft brake release counts as spent).

## M · 2026-11-09 (1776) — what the two auditors found, and what shipped

DS Flash and GLM 5.3 Flash audited `fc8dd13..HEAD` read-only (`docs/audit/500-session-ds.md`,
`docs/audit/501-session-glm.md`). The rows below are the ones I re-checked in the code and fixed; the rest,
including the ones I judged false or not worth a change, are in the reports.

| # | What | Passes if | State |
|---|---|---|---|
| M1 | **the lane really lets go** (1772) | brake into a bend, coast from the entrance: the shape stops there instead of running to the apex — the selection no longer re-adopts a corner it has just declared spent | ⬜ |
| M2 | **a stopped car is not coasting** (1773) | sitting on the grid, or stopped past an entrance, no corner is dropped from the plan | ⬜ |
| M3 | **the creep survives a race** (1774) | second race on the same track: the cap still opens at 0.1 m and climbs, instead of starting at 2 m | ⬜ |
| M4 | **a skipped shape has no entrance** (1775) | a corner the shape builder dropped no longer reports the start line as its entrance | ⬜ |
| M5 | **the lane looks past a skipped corner** (1777) | a wide corner the plan ignored gets no lean from the lane either — one selection, both readers | ⬜ |

Left alone on purpose, each one a verdict rather than a fix: the refill's `first = 0` fallback still commits a corner
the plan skipped — a queue needs a head, and `CornerRequiresBraking` keeps that head out of the braking table, so it is
a placeholder and not a decision to brake. The lane can adopt the next corner while the plan still brakes for the
current apex: that is what an S asks for, and the ramp is what keeps the first ask small. `UpdateCornerRequirements`
runs every frame because it is the detector, not the refill. `_cornerContexts` does prune on a revision change
(`SyncCornerContexts`), so that audit row was false. AGENTS/ now carries dated CURRENT blocks; its history stays as
history.

