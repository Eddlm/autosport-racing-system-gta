# Autosport Racing System 0.9.0 — first WIP

The first public WIP of the reworked mod: the playable race loop, the bundled tracks, the flat car roster, the in-game track creator, Auto Styling and the reworked AI.

## Required

- **ScriptHookVDotNet nightly**, not stable — the stable bridge cannot start on current game builds. Verified against `v3.7.0-nightly.188`: copy `ScriptHookVDotNet.asi`, `ScriptHookVDotNet2.dll` and `ScriptHookVDotNet3.dll` from that same download into the game folder.
- ScriptHookV, .NET Framework 4.8 and the latest Visual C++ Redistributable.
- `ScriptHookVDotNet2.dll` is **not** bundled — the asi and the API dll must be a matched pair.
- If ARS does not appear, `Scripts\AutosportRacingSystem\Log.log` names the reason; a build the mod cannot drive on logs a forced error rather than crashing.

## In this release

- **Steering and slides reworked as one control chain**: pure pursuit on the aim point, the yaw damper against the yaw the aim requires, a slide blend that hands the countersteer to the car inside the tyre's slip band, and a steer limiter whose ceiling reads the slip axis rather than a fixed angle.
- **Pedals as reasons**: TCS and ABS restated as shares of the peak slip, an overspeed correction that lifts throttle when the road pulls harder than the wheels explain, and a stability cut that closes a quarter of the throttle per wheel out of contact and gives it back at half the rate.
- **AI races the field, not a script**: overtaking pressure, divebomb / defend-lane / yield / chill-out maneuvers, one bottle of nitrous per lap fired by the AI itself, and a stuck-recovery cycle that reverses, drives out and only teleports as a last resort.
- **Tracks**: the in-game creator is live (record a route, save a track, the derived table rebuilds as you draw); bundled tracks; a corner is anything under 80 m on its tightest node by default (adjustable 20–200 m in the Debug menu, live mid-race); apex speeds read the smoothed radius, so cars brake later.
- **Settings**: one declared ini shape with a repair pass — retired files are removed, off-ladder values snap to the nearest offer, and legacy keys migrate. The AI's nitrous is a percentage of the player's (AI Nitrous Power, 50–200%, default 100), and the log ships at Error level so a fresh install still records why something failed.
- **Grid and board**: pace-ranked car selection for the grid, PI and best-lap columns on the frozen leaderboard.
- **Auto Styling**: the cosmetic auto-tuner (optional Menyoo appearance support).

## Known limitations

- **Bikes are out of scope** — the loader drops every bike line.
- **No track editing or deleting** — a created track can be saved, not updated.
- No per-car XML saves, saved drivers, personalities or discipline filtering.
- The grid uses **model-level pace**, not the installed upgrades of a particular vehicle.
- The beater/rust matte paint rule ships unverified in game.
- Low-grip versus high-grip behaviour across the fleet is a non-blocking check; report a car that turns in lazily or washes wide at the edge of the grip range.
