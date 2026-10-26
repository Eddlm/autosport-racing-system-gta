# Autosport Racing System — what you get

ARS turns GTA V into a race series. Pick a track, pick a grid size, pick how many laps, and a field of AI drivers turns up in real cars — drawn from the roster you choose — and races you: braking for corners, holding a line, defending a position, taking a chance down the inside, and getting it wrong often enough to read as a driver rather than a script.

It is not traffic AI wearing a race skin. The cars are given a plan instead of a path: before the lights, the whole route has been read and every corner in it measured, and from there each car works out its own braking, its own line and its own speed, for its own car, and then deals with the others doing the same thing.

This file is the tour, written for players: what you get and what it feels like, not which number does what.

## What you need

- **ScriptHookVDotNet Enhanced (SHVDNE) — not upstream's stable release.** The stable bridge cannot start on current game builds and takes every other script down with it. ARS is built against the v2 API (`ScriptHookVDotNet2.dll` **2.11.6**), which SHVDNE ships.
- **ScriptHookV**, **.NET Framework 4.8** and the current **Visual C++ redistributable** — installed separately.
- **Extract the `AutosportRacingSystem` folder into the game's `Scripts` folder**, overwriting if you are updating. Tracks, car rosters and settings live inside it, a first run writes the settings files for you, and the menu library is included — so there is nothing to hand-edit before your first race.
- If ARS does not appear, `Scripts\AutosportRacingSystem\Log.log` names the reason.
- **The menu** opens on `Left Shift + E`, or by typing `arsmenu` in the chat console.

## The rivals play by your rules

The first question anyone asks about a racing AI is what it is allowed to cheat with. Here: nothing.

ARS never takes your wheel. Your car answers your inputs and nothing else — no hidden assist, no steering help, no throttle shaping on your pedals. The rivals are on the same physics you are: their own car's real handling values, read from the game, with no grip multiplier and no invisible downforce bolted on. They are quick because of how they drive, not because they are allowed more. The one lever ARS does write on every car, yours included, is the game's own power multiplier: it sits at its resting value in normal driving, it keeps a car from being faded early as it approaches its own top speed, and it is what a nitrous burn rides on.

The field is built to suit you as well. The grid is chosen around a performance index — your car's, by default — so the cars that turn up are the ones near your pace, not the fastest thing in the roster. Want a harder field? Ask for one. You never have to out-drive a physics advantage.

**What you notice:** a rival that beats you did it in a car you could have been driving.

## They know the track before the green light

Most driving AI in this game is answering the road in front of it: it is told where to go and it steers there, corner by corner, as the road arrives. That is why traffic brakes *into* a corner instead of *for* it.

ARS cars arrive knowing. When a track loads, the route is read once and cut into the corners along it: where each turn-in starts, where the apex sits, where the corner ends, how tight it is and what kind of corner it is. A chicane is flagged so the plan can give it a flick's worth of speed. That description of the road is shared by the whole field, so no car's idea of where a corner is depends on what it happened to notice.

The difference shows in the preparation. On a tight corner a rival takes the long way round — swings to the outside, brakes, and turns in — because it knows the corner is there and where it wants to be for it. The same read is what slows a car for a crest that would unload it, and what makes the braking start at a place rather than at a feeling.

**What you notice:** a rival takes the outside line into a tight corner while you are still aiming at the apex.

## And they learn it

A map can tell a car where the corner is. It cannot tell it how hard *this* car can stop for it, so ARS has the car keep asking the corner instead.

Every rival starts a race on a guess, with a draw of its own confidence and a slightly different draw for each corner — so two cars of the same model do not brake identically, and one car is not evenly bold all the way round. Then it marks its own work. Spend too little of the braking phase at full pedal and it brakes deeper next lap; spend too much and it brakes earlier. Slide through the corner and it learns the blunt way instead — less next lap, without waiting for the numbers — though a pedal sample from an approach that was already sliding is thrown out, because the slide is the evidence and the pedals are not.

It commits its verdict just after the apex and watches the exit as well as the entry, so a corner taken cleanly and lost on the way out still counts. The lesson is kept per car and per corner rather than averaged into one number, and it is bounded: a rival can only get so brave or so timid before the clamp stops it.

Nothing is carried into the next race. The rivals arrive on the guess and earn their braking again over the laps, which is why the first meeting of a corner is not the one to judge a rival on, and why a longer race is a harder race — by the second meeting the field is braking closer to what the corner will actually take. On a point-to-point stage there is no second meeting, so the guess is what you get. Turn Brake Learning off and everybody is on the same guess, all race.

**What you notice:** pick one heavy braking zone and watch the same rival across laps. It brakes deeper on lap two than it did on lap one, or earlier if it was over-braking.

## The plan asks, the tyres answer

The plan gives a rival its target; the tyres get the final say. They drive through the same controls you do — throttle, brakes and steering, set every moment — with no invisible force shoving the car round a corner and no grip of their own. The same corner does not look the same in a Comet as it does in a Dominator, because the car is not the same car.

The wheel: the car aims at a point up the road and steers toward it, a damper takes the wobble out of the rotation, and when the back steps out the countersteer is handed over inside the window where there is still grip to give — and the brake comes off while it does, which is about the most human thing in the whole chain. What the car can ask for is capped, not by a fixed angle but by its own modelled peak slip at the speed it is doing. That model can be wrong, and being wrong is what a slide is.

The pedals: traction control lifts the throttle when the driven wheels spin past a target just under the peak, and ABS gives a wheel's brake up rather than dragging it locked. Throttle is also taken away when the car is off the road, unstable, or being pulled harder than its wheels can explain. The road's shape feeds all of it — a climb and a drop both cost grip, and a crest that would unload the car is a slow-down instead of a launch.

They will still light the tyres up, and they will still miss a brake. The aids make those moments brief and rare rather than impossible, and both are switches — turn them off and the same corner is a different corner.

**What you notice:** a rival lights the tyres up out of a hairpin, catches it, and does not lose the corner. If you think the aids are driving the car, turn TCS off and watch the same corner again.

## They race you

Rivals have temperament, and it is handed out with the grid. On a circuit you start at the back, so the cars immediately ahead of you are the desperate end of the field; on a point-to-point they line up behind you instead. Every car on the grid has a different appetite for a fight, and none of it comes out of a difficulty setting.

Fight them and they fight back. Follow a rival closely and pressure builds — slowly, because it takes sustained following to make it count, and it drains fast the moment you drop back. A rival carrying enough of it gives you the place at the next corner rather than fight a battle it thinks it is losing.

They have moves, and you can see them coming. A rival covers the inside of the next corner when a faster car is closing and would get to the turn-in no later. A faster car alongside gets let by rather than fought. Someone dives for the inside and brakes deeper than their own plan wanted. Someone else sits behind you and waits for the field to spread out.

Side by side is its own problem: the cars match headings to hold a line together, steer away from contact when the two of you are actually closing rather than merely near, and will not turn into you when they are alongside. And they race each other by the same rules — the same pressure, the same moves, an order of their own to settle — which you can watch, lap times and all, by sitting a race out.

**What you notice:** you get defended. The rival that has been following you for two laps will have a look down the inside — and the one that is beaten gives you the place instead of fighting for it.

## Nitrous

Nitrous is a race rule, on or off. With it on, every rival gets a fresh bottle each lap, and they spend it themselves: on a straight, in a fight, or near the finish. It is not a free pass — a rival will not fire mid-corner or on the way in, off the road, at a big steering angle, or from too low a gear, and that last one is a rear-drive problem: an all-wheel-drive car can put the power down and is not held back.

AI Nitrous Power sets their bottle as a percentage of yours, anywhere from 50 to 200. It is the most surgical dial in the file, because it changes what a rival can do on a straight without touching how it drives a corner.

Your own bottle is coming. Theirs is live now.

**What you notice:** a rival pulls a gap on the straight that it cannot hold anywhere else.

## When it goes wrong

Spins, offs and wrecks happen. A rival that ends up stationary with its own plan still asking to move reverses out; one that has left the road drives back on; and one that truly cannot get going is set back on the road rather than left there. None of it arms in the opening stretch of a race, when a car crawling off the line is not stuck and the pack around it is not a place to reverse into.

With damage on, cars die too. Racer Autofix decides how much of that the field suffers: Off takes it the way you do, Auto Repair keeps them hard to break and rebuilds the car once body or engine drops below half, and Invincible — proofs and an instant repair — means a rival's car cannot be killed at all, because the field is there to race rather than to be eliminated by one stray kerb. A car whose engine is gone leaves the race properly: parked on the shoulder, or dropped out of sight and ignored by the AI, depending on the rule you picked.

**What you notice:** a spun rival rejoins instead of sitting there, and you are not left weaving through wreckage on lap two.

## Race day

The grid forms, the cars get dressed, and the lights count you down. Then the pack pulls away and you find out whether the pace matching did its job.

Out on track you get a board — position, car, performance index and best lap — and a corner display carrying your own position, current lap time and total time. Positions lock the moment a car crosses the line, so the order stops shuffling under you as the race closes. When the first car finishes, a grace period starts for everyone still out there; when it runs out the race is called and the stragglers are classified. Win it, and the purse is real money.

You can start from the menu, or just drive up to a track's marker in the world and take the race at your feet. You can restart the same field, drop yourself back onto the road if you end up in a hedge, or sit the race out and watch the field go round — with corner markers and track-edge chevrons on screen if you want to see the line they are taking. And ambient traffic is suppressed while a track is loaded, so the race is yours.

**What you notice:** the board stops moving the moment you cross the line, and the race keeps going behind you until the clock runs out.

## The tracks

ARS ships with more than a dozen routes: short airport sprints, a horsetrack, a lake loop, long road circuits and an offroad stage. Any route can be raced backwards. A track file dropped into the `Tracks` folder joins the list on the next load; a route you save in the creator joins it straight away.

**What you notice:** no route has to be learned first. The read comes from the geometry rather than from experience, so a circuit nobody has raced before gets the same first lap as a familiar one.

## The Track Creator

You can build a route without ever leaving the game. The creator takes over the free camera: place your start line, then draw the route as you fly it. The weapon-cycle inputs widen or tighten the section you are laying, attack applies it, and aim deletes a node you would rather not have.

The corner table is rebuilt each time you apply a section or delete a node, so the AI's reading of your track is ready by the time you save it. That save goes into `Tracks\` and joins the menu list straight away.

**What you notice:** the field takes your corner the way the table promised, on the first lap, with no practice.

## Making the field yours

The grid draws from a text file of model names in `Vehicles\` — the default list runs to hundreds of cars, and there is a muscle list, a lemons list and room for yours. The menu lists whatever files are in the folder, re-read each time it opens, so adding a roster is dropping in a file. If your list is shorter than the grid you asked for, Repeat Cars To Fill Grid tops it up, and your own car is added last: the back of the grid on a circuit, the front on a point-to-point.

Rivals get their names from `Drivers\names.txt`, reshuffled for every grid, and the player keeps their own. Switch rival lap times on in Interface and those names come with them.

Auto Styling dresses the field before a race: it reads each car's own livery names, picks a style from what they say — rust, camouflage, racing, stripes, flames, tuner, plain — fits cosmetic parts to match and paints last, one car per tick during the countdown, so it costs you no loading time. Your car is never touched, and a car that is already tuned is left alone; a car whose liveries carry no names still gets the parts-and-paint half. With Menyoo installed a matching saved appearance can dress a grid car instead, and when one is applied Auto Styling leaves that car alone.

**What you notice:** a field that looks like a field rather than twelve identical loaners, and not a fingerprint on your own build.

## Tuning and setup

The menu has a Race section (track, laps, grid size and the performance index the field is drawn from), a Rivals section (who races you, the grid sorting, plus their pace, traction and steering dials), Race Rules (timing, damage, DNFs, collision, nitrous, rubberbanding), Interface, Controls (the menu chord and the `arsmenu` fallback), and the Track Creator entry. The Rivals dials are the ones that change character: corner and road-curvature speed offsets, brake learning on or off, crest lift, slope loss, overspeed correction, TCS and ABS and how soft their cuts are, steering damping, and the steering limit at speed. Between them you can make a field braver, tidier or sloppier corner by corner. Rubberbanding is worth its own line: off by default, and when you turn it up it keeps the field packed — the leaders are slowed, and at the artificial setting the backmarkers also get a boost.

Settings live in four ini files under `Settings\` (Race, Settings, Debug, Development), and ARS maintains them: retired keys are removed, a number that is no longer offered snaps to the nearest one that is, a name that is no longer offered goes back to its default, and missing keys are recreated on load — so a hand-edited file repairs itself and a fresh install has nothing to set up. There is also a Debug submenu for tinkerers — visual overlays, the log level, and the radius at which the detector stops treating the road as a corner region — but nothing in it is needed to race.

**What you notice:** a field you can tune to your taste without ever opening a file by hand.

## What ARS does not have yet

- **Bikes.** The grid skips them.
- **Editing or deleting a track.** The creator saves new routes; you cannot reopen one in-game to change it.
- **Personalities, saved drivers, per-car setups and disciplines.** They existed before the rework and are not back yet.
- **Grid pace is model-level.** Cars are matched by their model's pace, not the upgrades on a particular vehicle, so a tuned example of a slow car is still selected as a slow car.
- **The player's own nitrous bottle.** Coming soon — the rivals' bottles are live.
- **The rivals' driver aids do not reach your car.** No ARS traction control, launch shaping or crest discipline on your pedals: your car drives exactly the way the game drives it.
- **No gamepad route into the menu.** It opens on the keyboard chord or the `arsmenu` command, and neither has a controller binding.
- **No flames or nitro engine note.** The burn is a straight-line power boost, and the theatre is not there.
- **FiveM.** Single-player, for now.
