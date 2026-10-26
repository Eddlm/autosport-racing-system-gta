# Autosport Racing System

AI racing for GTA V. Point it at a route, pick your opponents, and the field drives itself: braking points, lines, overtakes, and a leaderboard at the end.

Status: **WIP, 0.9.0**. Nothing here is final.

## Requirements

| What | Why |
|---|---|
| **ScriptHookV** ([Alexander Blade](http://www.dev-c.com/gtav/scripthookv/)) | loads native plugins. Not bundled. |
| **.NET Framework 4.8** + latest **Visual C++ Redistributable** | what the script runs on. |
| **ScriptHookVDotNet Enhanced** ([SHVDNE](https://github.com/Chiheb-Bacha/ScriptHookVDotNetEnhanced/releases)) | the .NET bridge. It ships the **v2 API** this mod is built against: `ScriptHookVDotNet2.dll` **2.11.6**. |
| **LemonUI.SHVDN2 1.5.2** | menus. Bundled, MIT, by [JustALemon](https://github.com/LemonUIbyLemon/LemonUI). |

Two things that cost people an evening, so they are here rather than in a footnote:

- **Do not install SHVDN and SHVDNE side by side.** One bridge, one `.asi`. Two of them fight.
- **Copy the `.asi` and the `2.dll` from the same download.** A script that pairs a new `.asi` with an old API dll fails *silently* — nothing else in the game looks wrong. The 2022 stable release's v2 dll (2.10.13) is too old and takes ARS down without a message.

## Install

1. Install ScriptHookV, .NET 4.8, the VC++ redistributable and SHVDNE.
2. Extract the `AutosportRacingSystem` folder into the game's `Scripts` folder, overwriting when updating.
3. In game, press **Left Shift + E**.

Tracks, car rosters and settings live inside that folder. The menu key is rebindable in `Settings\Menu-Settings.ini`.

## Car rosters

`Vehicles\` holds plain text files, one model name per line. The Rivals menu picks which one the grid draws from. Two rows are enough to see the shape — add your own the same way:

| File | What it is for |
|---|---|
| `Vehicles\cars.txt` | The full list. Every four-wheeled model the game knows, minus bikes, boats, wings and trains. |
| `Vehicles\tests.txt` | A scratch list for testing one car at a time; drop a single model name in it and the grid is that car. |

**Every four-wheeled class races.** Off-road, industrial, utility, vans, service, emergency and military are all in the pool now; only motorcycles, bicycles, boats, helicopters, planes and trains stay out. The pace band decides who races whom, not the body style — so a grid can mix types it never mixed before.

## What works

**Racing**
- Race on the bundled routes, or create any route you want and save it.
- Race against AI opponents, with a pace-ranked grid drawn from your roster.
- Live leaderboard: position, PI, best lap, and the peak stats each car actually pulled.
- TCS, ABS, nitrous, rubberbanding, per-corner brake learning, and a steering/pedal guard that keeps the AI out of the tyre's cliff. All of it AI-only — the player's controls are never written.

**Tracks**
- The in-game creator is live: record a route, save it, and the derived corner table rebuilds as you draw.
- Routes live in `Tracks\` as plain XML.

**Settings**
- One declared ini shape per menu, with a repair pass: retired files are removed, off-ladder values snap to the ladder, missing keys are written back.

## Not back yet

- Racer creation.
- Track editing — creating and saving a route works; reopening a saved track to change it does not.
- Personalities.

## Contact & Feedback

- I'm in [Vanillaworks](https://discord.gg/hPUEChBCeV), channel [#ars-project](https://discord.com/channels/837447158197518376/985845358900940800). Alternatively the [GTA5Mods discord](https://discord.gg/2PR7aMzD4U), `@eddlm`. No DMs.
- Or open an issue here.

## The plan

There is no plan. VI is out in like a month and I don't expect anybody to care about ARS by now, much less when *that* steals the spotlight. ARS is working and up to date, reduced to the core, and missing systems come back over time if they earn it.

| Cadence | What to expect |
|---|---|
| Weekly updates | I'll keep the GTA5Mods release up to date weekly at best, monthly at worst. |
| Issues | Opened on GitHub are read; fixes land in the next build, not in a hotfix. |
