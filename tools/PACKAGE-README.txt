Autosport Racing System (ARS) - WIP build

Install: extract the AutosportRacingSystem folder into
Grand Theft Auto V\Scripts\ (overwrite if updating).

In game, press Left Shift + E (the chord: Left Shift is the modifier, E the
second key). Rebind either half under Controls (Menu Modifier, Menu Key), or edit
MenuModifierKey / MenuKey in Settings\Menu-Settings.ini. You can also open it
by typing arsmenu in the chat console.

Included: LemonUI.SHVDN2 1.5.2, used under the MIT License, by JustALemon
(https://github.com/LemonUIbyLemon/LemonUI).

Required (install separately, we do not bundle them):

  - ScriptHookV (Alexander Blade): http://www.dev-c.com/gtav/scripthookv/

  - .NET Framework 4.8 and the latest Visual C++ Redistributable.

  - ScriptHookVDotNet Enhanced (SHVDNE), which ships the v2 API this mod is
    built against: ScriptHookVDotNet2.dll 2.11.6.
      https://github.com/Chiheb-Bacha/ScriptHookVDotNetEnhanced/releases
    Copy ScriptHookVDotNet.asi and ScriptHookVDotNet2.dll from the SAME
    install into the game folder. ARS runs on the v2 API, so an asi paired
    with an older API dll makes it fail silently.

Tracks, car rosters and drivers live inside AutosportRacingSystem\.
Nothing here needs a settings file: the mod writes the missing keys on its
first load.

If the mod does not appear, read ARS.log in the game folder, beside GTA5.exe - the first lines
say what it found and what it refused.
