# Before First WIP Release

Release target: **10 September**

This file lists what must be done, verified, or documented before the first WIP can be released. The goal is a build that is playable without issues, has enough options for common player needs, and gives players tools to diagnose and visualize problems.

## 1. Must Work (No Release Blockers)

### Core loop
- [ ] Start a race from the menu, finish or abandon it, and return to free roam without crashing.
- [ ] Track loading works for all tracks in the default `Tracks\` folder (circuit + point-to-point).
- [ ] AI grid spawns and drives the full race distance without pile-ups at the start or mid-race.
- [ ] Player can join the grid and race; player car is never overwritten or teleported unexpectedly.
- [ ] Race cleanup removes all AI cars, blips, and menu state when the race ends or is aborted.

### Steering / speed pipeline
- [ ] Cars stay on track at low, medium, and high speed on the test tracks.
- [ ] Braking is sufficient for tight corners but not overly conservative on sweepers.
- [ ] AI does not lock up or spin repeatedly under normal conditions.
- [ ] Stuck recovery handles being off-track or upside down without infinite loops.

### Maneuvers / avoidance
- [ ] AI overtakes cleanly when a rival is slower.
- [ ] AI does not drive through rivals or walls.
- [ ] Divebomb / DefendLane maneuvers arm and disarm correctly and do not cause crashes.
- [ ] Yield maneuver works when trailing a faster rival near a corner.

## 2. Player Options (Common Needs)

### Race setup
- [ ] Track selection lists all available tracks and selects a sensible default.
- [ ] Grid size can be set from small to large and is respected.
- [ ] Pace target + pace bracket produce a field that matches the requested difficulty/speed.
- [ ] Point-to-point vs circuit races are selectable where applicable.
- [x] Players can enter the ARS menu through an immersive world interaction (no hotkey required).

### Menu organization
- [ ] Menu categories are logically grouped (race setup, debug, options, etc.).
- [ ] Root menu surfaces the most common actions first (Start Race, Freecam).
- [ ] Debug toggles live in a clearly named submenu, not mixed with race setup.
- [ ] Pace target and bracket are explained with in-menu text or labels.
- [ ] Better communication of the pace system: show what the selected pace means in plain terms (e.g., expected top car speed / class).

### Debug / tuning toggles
- [ ] ShowInputs displays useful live data for the nearest AI racer.
- [ ] ShowTrackAnalysis visualizes lane aim, wall lines, and corner chevrons.
- [ ] ShowAggro displays pressure and short-term projections.
- [ ] Gs-aware preview toggle and blend are exposed in the Debug menu.
- [ ] Options.ini changes persist and apply on reload.

## 3. Diagnostic / Visualization Tools

- [ ] Log file writes startup and error messages to `Log.log` without crashing the game.
- [ ] Debug markers are visible when enabled and show the right information.
- [ ] Player can see the intended speed vs current speed of an AI car.
- [ ] Player can see which track node / corner the AI is targeting.
- [ ] Player can see pressure, aggression, and current maneuver state.

## 4. Known Bugs to Verify or Fix Before Release

- [ ] Reverse throttle path behavior after the `ConvertSpeedToPedals` refactor.
- [ ] Two closely spaced corners where the second is slower (exit-zone rule).
- [ ] Corner hugging through apex.
- [ ] Steering-limit cut rate feels right across car classes.
- [ ] Hill/crest grip floor values do not zero out corner speed unexpectedly.

## 5. Documentation / Release Notes

- [ ] README or release notes mention how to install, start a race, and open the menu.
- [ ] Default keybinds / chat commands are documented.
- [ ] Known limitations list (e.g., Add-On model compatibility, untested reverse path).
