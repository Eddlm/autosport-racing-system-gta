# Autosport Racing System — what you get

ARS turns GTA V into a race series. Pick a track, pick a grid size, pick how many laps, and a field of AI drivers turns up in real cars — drawn from the roster you choose — and races you: braking for corners, holding a line, defending a position, taking a chance down the inside, and getting it wrong often enough to read as a driver rather than a script.

It is not traffic AI wearing a race skin. The cars are given a plan instead of a path: before the lights, the whole route has been read and every corner in it measured, and from there each car works out its own braking, its own line and its own speed, for its own car, and then deals with the others doing the same thing.

This file is the tour, written for players: what you get and what it feels like, not which number does what.

## What you need

- **ScriptHookVDotNet — the nightly, not the stable release.** The stable bridge cannot start on current game builds. Copy the `.asi` and the API dll from the same download: they are a matched pair.
- **ScriptHookV**, **.NET Framework 4.8**, and the current **Visual C++ redistributable**.
- Drop the ARS folder into the game's `Scripts` folder. Tracks, car rosters and settings live inside it, and a first run writes the settings files for you — there is nothing to hand-edit before your first race.
- If ARS does not appear at all, `Scripts\AutosportRacingSystem\Log.log` names the reason.

## The rivals play by your rules

The first question anyone asks about a racing AI is what it is allowed to cheat with. Here: nothing.

ARS never takes your wheel. Your car answers your inputs and nothing else — no hidden assist, no steering help, no throttle shaping on your pedals. The rivals are on the same physics you are: their own car's real handling values, read from the game, with no grip multiplier and no invisible downforce bolted on. They are quick because of how they drive, not because they are allowed more. The one lever ARS does write on every car, yours included, is the game's own power multiplier: it reads 1.0 in normal driving, it keeps a car from being faded early as it approaches its own top speed, and it is what a nitrous burn rides on.

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

## They do not drive like traffic

A rival's lap is two jobs held apart: where the car is pointed, and how fast it is going. Both are worked out every tick from that car's own numbers, so the same corner does not look the same in a Comet as it does in a Dominator.

The wheel: the car aims at a point up the road and steers toward it, a damper takes the wobble out of the rotation, and when the back steps out the countersteer is handed over inside the window where the tyre still has grip to give. It is bounded by the tyre the whole way — a car cannot ask for more steering than its wheels can use at the speed it is doing, so you will not see a rival turn in at an angle no car could take.

The pedals: traction control holds the driven wheels just under the slip where they make the most force, ABS does the mirror of that on the brake side, and throttle is taken away when the car is off the road, unstable, or being pulled harder than its wheels can explain. The road's shape feeds both — a climb and a drop both cost grip, and a crest that would unload the car is a slow-down instead of a launch.

**What you notice:** they slide, catch it, and carry on. Nobody spins their wheels out of a hairpin or locks up into one like a car on rails.

## They race you

Rivals notice you. Sit in someone's mirrors and they start to feel you there, and aggression is handed out by grid slot so the field is not twelve copies of one driver: the front of the grid the coolest, the back of it the most desperate, you in the middle.

They have moves, and you can see them coming. A rival covers the inside of a corner it knows you are quick through. A faster car alongside gets let by rather than fought. Someone dives for the inside and brakes deeper than their own plan wanted. Someone else sits behind you and waits for the field to spread out.

Racing side by side is its own problem, handled separately: the cars match headings to hold a line together, steer away from contact when the two of you are actually closing rather than merely near, and hold steady when they are alongside you instead of swerving into you. And they race each other, not only you — the field has its own order to settle, and you can have every rival's lap time read out if you want to watch it happen.

**What you notice:** you get defended. The rival that has been following you for two laps will eventually have a look down the inside.

## Nitrous

Nitrous is a race rule, on or off. With it on, every rival gets a fresh bottle each lap, and they spend it themselves: on a straight, in a fight, or on the last lap. It is not a free pass — a rival will not fire mid-corner, off the road, at a big steering angle, or from too low a gear for the car it is in.

AI Nitrous Power sets their bottle as a percentage of yours — half, parity, or twice — which makes it the cleanest difficulty dial in the file: it changes what they can do on a straight without touching how they drive a corner.

Your own bottle is coming: one shot a lap, fired by hand, on the same terms the rivals play by. Theirs is live now.

**What you notice:** a rival pulls a gap on the straight that it cannot hold anywhere else.

## When it goes wrong

Spins, offs and wrecks happen. A rival that ends up stationary with its own plan still asking to move reverses out; one that has left the road drives back on; and neither is allowed to arm in the opening seconds of a race, when a car crawling off the line is not stuck and the pack around it is not a place to reverse into.

Cars die too. By default a rival's car does not take damage — the field is there to race, not to be eliminated by one stray kerb — but damage can be set to resistant or fully on. A car whose engine is gone leaves the race properly: parked on the shoulder, or dropped out of sight and ignored by the AI, depending on the rule you picked.

**What you notice:** a spun rival rejoins instead of sitting there, and you are not left weaving through wreckage on lap two.

## Race day

Pick a track, a lap count and a grid size, and start — or just drive up to a track's marker in the world and take the race at your feet. ARS loads the route, builds the grid around your pace, places everyone, dresses the cars if you asked for it, and counts you down.

Out on track you get a board: position, name, lap, performance index, best lap. Positions lock the moment a car crosses the line, so the order stops shuffling under you, and the winner's purse is real money. When the first car finishes, a grace period starts for everyone still out there; when it runs out the race is called and the stragglers are classified. Ambient traffic is suppressed while a track is loaded, so the race is yours.

You can restart the same field, drop yourself back onto the road if you end up in a hedge, or sit the race out and watch the field go round — with corner markers and track-edge chevrons on screen if you want to see the line they are taking.

**What you notice:** the board stops moving the moment you cross the line, and the race keeps going behind you until the clock runs out.

## The tracks

ARS ships with sixteen routes: short airport sprints, a horsetrack, a lake loop, long road circuits and an offroad stage. Any route can be raced backwards, and a track file dropped into the `Tracks` folder turns up in the list beside them.

**What you notice:** no route has to be learned first — a circuit the field has never raced is read exactly the way it reads one you have both done fifty laps of.

## The Track Creator

You can build a route without ever leaving the game. The creator takes over the free camera: place your start line, then draw the route as you fly it. The weapon wheel widens or tightens the section you are laying, attack applies it, and aim deletes a node you would rather not have.

The corner table re-derives while you draw, so you can see the corners the AI will read out of a track that does not exist yet. Save it and it races like any other.

**What you notice:** the field takes your corner the way the table promised, on the first lap, with no practice.

## Making the field yours

The grid draws from a text file of model names in `Vehicles\` — the default list runs to hundreds of cars, and there is a muscle list, a lemons list and room for yours. The menu lists whatever files are in the folder, re-read each time it opens, so adding a roster is dropping in a file. If your list is shorter than the grid you asked for, Repeat Cars To Fill Grid tops it up, and your own car is added last: the back of the grid on a circuit, the front on a point-to-point.

Rivals get their names from `sillynames.txt`, reshuffled for every grid, and the player keeps their own.

Auto Styling dresses the field before a race — a livery picked from the car's own livery names by what they say (rusty, camo, racing, striped, tuner, clean), cosmetic parts to match, paint last, one car per tick during the countdown so it costs you no loading time. Your car is never touched, and a car that already has a mod kit fitted is left alone. With Menyoo installed, its saved vehicle appearances can dress the grid instead — again, never yours.

**What you notice:** a field that looks like a field rather than twelve identical loaners, and not a fingerprint on your own build.

## Tuning and setup

The menu has a Race section (track, laps, grid size and the performance index the field is drawn from), a Rivals section (who races you, plus their pace, traction and steering dials), Race Rules (timing, damage, DNFs, collision, nitrous, rubberbanding), Interface, Controls, and the Track Creator entry. The Rivals dials are the ones that change character: corner and road-curvature speed offsets, brake learning on or off, crest lift, slope loss, overspeed correction, TCS and ABS and how soft their cuts are, steering damping, and the steering limit at speed. Between them you can make a field braver, tidier or sloppier corner by corner. Rubberbanding is worth its own line: off by default, and when you turn it up it keeps the field packed — the leaders are slowed, and at the artificial setting the backmarkers also get a boost.

Settings live in three ini files under `Settings\`, and ARS maintains them: retired keys are removed, a value that is no longer offered snaps to the nearest one that is, and missing keys are recreated on load, so a hand-edited file repairs itself and a fresh install has nothing to set up. There is also a Debug submenu for tinkerers — visual overlays, the log level, and the widest radius the track table still keeps as a corner — but nothing in it is needed to race.

**What you notice:** a field you can tune to your taste without ever opening a file by hand.

## What ARS does not do yet

- **Bikes.** The grid skips them.
- **Editing or deleting a track.** The creator saves new routes; you cannot reopen one in-game to change it.
- **Personalities, saved drivers, per-car setups and disciplines.** They existed before the rework and are not back yet.
- **Grid pace is model-level.** Cars are matched by their model's pace, not the upgrades on a particular vehicle, so a tuned example of a slow car is still selected as a slow car.
- **The player's own nitrous bottle.** Coming soon — the rivals' bottles are live.
- **Flames and engine noise on a burn.** The power is real; the theatre is not there yet.
- **FiveM.** Single-player, for now.
