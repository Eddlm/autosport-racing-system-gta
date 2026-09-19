# AGENTS-SHVDN.md — SHVDN build compatibility (the asi ↔ API-dll pairing)

Companion to the "SHVDN build compatibility (release-critical)" block in `AGENTS.md`. That block is the summary; this is the evidence, the mechanism and the lab.

**Read this when**: the work touches **SHVDN / ScriptHookVDotNet, install or compatibility, "the mod doesn't load", the asi or API dll, version matching, the game build, or release requirements** — or **SHVDN Enhanced / SHVDNE / the Enhanced edition / "would that other SHVDN work with ARS"**.

## The question this answers

ARS is compiled against the ScriptHookVDotNet **v2 API** (`ScriptHookVDotNet2.dll`). Users install SHVDN themselves, so the release question is: *which SHVDN installs actually run ARS?* Answered in-game on game build `VER_1_0_3889_0` by swapping components one at a time (matrix below).

## Why build matching is mandatory — the two-assembly design

- SHVDN ships an **ASI loader** (`ScriptHookVDotNet.asi`) plus one or more **API dlls** beside it: `ScriptHookVDotNet2.dll` (v2 API), `ScriptHookVDotNet3.dll` (v3 API). The asi's filename is identical across builds and channels.
- The **native bridge** lives in the asi's own managed assembly, named `ScriptHookVDotNet` (assembly version `3.6.0.0` in every build examined). Its types — namespace `SHVDN.*`, e.g. `NativeMemory`, `NativeFunc` — are what the API dlls call to reach natives. The API dll references the bridge; the bridge never references the API dll.
- So **the asi and the API dll are a matched pair**. A script binds only to the API assembly *name* (`ScriptHookVDotNet2`), so a foreign dll loads fine, resolves fine, and throws at the first bridge call.
- The asi enumerates `ScriptHookVDotNet<digit>.dll` and resolves per script; it can host **both APIs at once** (verified: two `Loading API from …` lines with scripts still resolving to `API 2.11.6`). The v3 dll also implements the console — its absence produces the "ScriptHookVDotNet3.dll is not loaded" warning and a console that can only display logs. A leftover v3 dll next to a working v2 install is **harmless** (it stayed dormant in the verified run).

## The version-label trap

Both the widely distributed bundle's v2 dll and the nightly's report **assembly version `2.11.6.0`** with no public key token; they differ only in *file* version (`2.10.13` vs `2.11.6`). The CLR resolves by simple name, so the mismatch is invisible to every version check — including SHVDN's own line `Found 1 script(s) in ARS.dll resolved to API version 2.11.6`, which reports the assembly the script *references*, not the one it got.

**Consequence:** never diagnose this from version numbers, and never trust "it resolved". Only a runtime probe can distinguish the builds.

## Verified matrix (game build `VER_1_0_3889_0`)

**The "nightly" in every cell below is `v3.7.0-nightly.188`** — commit `0d61afa8d46f5a80297a716cfb2ed04f186c57cd`, whose `ScriptHookVDotNet2.dll` reports `ProductVersion 2.11.6+0d61afa8…`, which is how the installed pair was identified. "bundle 2022" is the widely mirrored stable **v3.6.0** zip. Naming the nightly matters: the matrix said only "nightly" for a year, and that is not reproducible — a later nightly is a different bridge.

| asi | `ScriptHookVDotNet2.dll` | `ScriptHookVDotNet3.dll` | Result |
|---|---|---|---|
| nightly (asi 239104 B) | nightly (984576 B, file 2.11.6) | — | **works** — race ran |
| nightly | nightly | bundle 2022 (1055744 B) | **works** — stray v3 dll dormant |
| nightly | bundle 2022 (1033728 B, file 2.10.13) | — | **ARS dies silently** — `MissingMethodException: UInt32 SHVDN.NativeMemory.GetHashKey(String)` |
| bundle 2022 (asi 152064 B) | bundle 2022 | bundle 2022 | **every script dies** — `SHVDN.NativeMemory..cctor()` `NullReferenceException` |
| bundle 2022 | bundle 2022 | — | **same failure** ⇒ it is the game build, not multi-API loading |
| nightly | *absent* | bundle 2022 | untested; predicted dead (no `ScriptHookVDotNet2` to resolve) |

The 2022 bundle fails identically with and without the v3 dll, and it kills `HandlingIsolationDemo` the same way — so its bridge simply cannot initialise on a current game build. That is a SHVDN-vs-game-version limit, **not** an ARS defect and not something ARS can fix.

## ARS's own fragility on this path (fixed in `353d65f`)

- ARS's **type initializer** used to call a native: the static field `RacerModels` converted two strings through `GTA.Model.op_Implicit` → `Game.GenerateHash` → `SHVDN.NativeMemory.GetHashKey`. On a mismatched bridge the type initializer threw, so the constructor never ran and **`Log.log` was never even opened** — total, silent failure with no ARS-side diagnostics.
- That field was **dead state** (read by nothing; the live driver list is `StreetRacerModels`, a `PedHash` enum array that needs no native call) → it was pruned rather than relocated.
- `VerifyScriptBridge()` runs in the constructor immediately after the log banner and before `LoadSettings()`; it probes `Game.GenerateHash("ars")`, logs the cause with `forced: true` (so `LogLevel` cannot hide it) and **rethrows**, leaving behaviour unchanged (ARS still does not run on a bad install) but making it diagnosable.
- **Order is load-bearing**: no native call may precede the probe, and **no native call may sit in a static field initializer**. A static initializer cannot be reported by a guard that has not run yet.
- Verified both directions: healthy install → probe silent, race runs; mismatched install → `Log.log` holds the banner plus `(Error): ScriptHookVDotNet2.dll is not usable with this ScriptHookVDotNet.asi build (MissingMethodException) …`, and the SHVDN stack shows `ARS.ARS.VerifyScriptBridge` instead of `ARS.ARS..cctor`.

## Release requirement (user-facing)

1. **A SHVDN build that supports the user's game build.** The stable **v3.6.0** served as the public download cannot run on current game builds at all (SHVDN's own release notes say to use nightly.89 or later from game `v1.0.3258.0`). Send users to the **nightly** releases — a separate repo, and no GitHub account is needed:
   - page: `https://github.com/scripthookvdotnet/scripthookvdotnet-nightly/releases`
   - the verified build: `https://github.com/scripthookvdotnet/scripthookvdotnet-nightly/releases/tag/v3.7.0-nightly.188` (asset `ScriptHookVDotNet-v3.7.0-nightly.188.zip`)
   - these links ship to users in the generated `README.txt` (`deploy.yml`); keep the two in sync.
2. **The asi and `ScriptHookVDotNet2.dll` from the same build** — not merely "a v2 dll present". Verified pair, byte-for-byte: asi `239104` + `ScriptHookVDotNet2.dll` `984576`.
3. Diagnosis to quote: `MissingMethodException: Method not found: 'UInt32 SHVDN.NativeMemory.GetHashKey(System.String)'` (or any other `SHVDN.*` member) in `ScriptHookVDotNet.log`; since `353d65f` also named in ARS's own `Log.log`.
4. **Never ship a pinned `ScriptHookVDotNet2.dll` with ARS** — it would manufacture exactly this mismatch for every user whose asi is a different build. Considered and rejected on this evidence. **LemonUI is the deliberate opposite case and *is* bundled** (`libs\LemonUI.SHVDN2.dll`, staged by name in `deploy.yml`, MIT with the author's permission, and the last release supporting the v2 API): it is our own script-side dependency, whereas the API dll belongs to the user's SHVDN install. The `deploy.yml` staging line must therefore name the file — a `bin\Release\*` wildcard would sweep `ScriptHookVDotNet2.dll` in and undo this.

## A second distribution: SHVDN Enhanced (SHVDNE)

`https://github.com/Chiheb-Bacha/ScriptHookVDotNetEnhanced` — a fork of upstream SHVDN that adds **GTA V Enhanced** support and claims to be a drop-in replacement on Legacy. Analysed at release **v1.1.0.6** (`d756d47`); upstream baseline `scripthookvdotnet/scripthookvdotnet` **`main`** (not `master`) @ `e98796d`. **Static analysis only — never run in game.** Full evidence, method and caveats: `Util\shvdne-lab\FINDINGS.md`; the check is re-runnable with `Util\shvdne-lab\SurfaceProbe\`.

Why it is worth keeping in mind:

- **It satisfies the matched-pair rule by construction.** One release ships the asi plus `ScriptHookVDotNet2.dll` and `ScriptHookVDotNet3.dll` from the same build, so the mismatch that kills a foreign v2 dll cannot occur *inside its own zip* — the failure mode this whole file is about. The probe confirms every bridge member its v2 dll calls resolves in its own asi, signature-exact.
- **ARS is clear of it.** `ARS.dll` binds only `ScriptHookVDotNet2` (never the bridge `ScriptHookVDotNet` directly; `--refs`), and its entire v2 surface — **17 types (mostly enums), 230 methods, 5 fields** — resolves signature-exact against SHVDNE's v2 dll. `src\` also compiles against it, exit 0.
- **Its v2 API *is* upstream's v2 API.** The whole `source/scripting_v2` delta is the removal of the **200 `[Obsolete("The v2 API is deprecated…")]` attributes** plus `using`/casing cleanup. Concretely: ARS builds with **10 warnings** against SHVDNE's v2 dll and **1595** against the current nightly. **The v2 deprecation wall is gone here** — which is the one thing the "v3 migration is the exit" line above costs us.
- **The Legacy path is intact.** Edition is read off the process image name (`GTA5_Enhanced.exe`), so every `s_isEnhanced` branch is inert on Legacy, and `DllMain.cpp` differs from upstream by **37 lines that do not touch `ClrThreadProc` / `ScriptHookVDotNet_ManagedTick` / the main-thread TLS swap**. The README's "scripts relying on the main thread may not work" caveat is **inherited upstream behaviour, not a new SHVDNE risk** — ARS already lives with it on the nightly.
- **Its large bridge rewrite cannot reach ARS's memory work.** `VehicleMemory.ScanModule` walks `Process.GetCurrentProcess().MainModule` with raw pointers itself, so ARS's AOB patterns, control writes and handling reads are independent of SHVDN's memory layer (`NativeMemory.cs` is ±13.5k lines and a new 769-line `Hooking.cs` — none of it in ARS's path).
- One member-level API difference from the nightly: SHVDNE's `GTA.GameVersion` stops at `VER_1_0_3570_0` (`VER_1_0_3586_0/3717/3725/3751/3788` absent). **Harmless** — ARS reads no `GameVersion` member (its two `Game.Version` locals are dead), and neither enum defines `VER_1_0_3889_0`, which upstream documents as still comparing correctly.

Unverified, and the reasons not to adopt it blindly:

- **No runtime evidence at all.** No game session was run. Static resolution is not a working mod, and this file's own lesson is that compile-clean proves nothing about a native bridge.
- **Single-maintainer fork with a heavily rewritten bridge.** The Legacy branches are *gated off*, not proven untouched; a rewrite of that size is risk a metadata probe cannot see. It also adds a **MinHook** dependency upstream does not have.
- **`sdk/` is empty** — the Script Hook V SDK is not vendored, so the asi cannot be built from a fresh clone. The release artifact must be trusted.
- **Recommending it would falsify our published install instructions**: SHVDNE needs **its own** `ScriptHookVDotNet.ini` ("does not add missing settings"), its default raw-script API is v3, and `WarnOfDeprecatedScriptsWithTicker=true` would show ARS users a **deprecation ticker** because ARS is a v2-built script. The `README.txt` links and the release requirement above would both need revisiting.

## The lab (reproduce any cell)

All under the gitignored `Util\shvdn-compat\`:

- `apply-shvdn-profile.ps1 -Profile <cell>` — places one combination in the game root from a clean slate (always clears a stale `ScriptHookVDotNet3.dll`); never touches `ARS.dll` or its data. Profiles: `good-2026v2`, `mixed-2026asi-2022v2`, `2026asi-both`, `bundle-2022`, `2022asi-v2only`, `2022asi-v3only`, `v3only-2026asi`, `v3only-2022asi`.
- `collect-shvdn-evidence.ps1 -Profile <name>` — run after the game exits; files hashes, all SHVDN logs, `ScriptHookV.log` and ARS's `Log.log` under `runs\<name>\` with a `verdict.txt` of the deciding lines. Use a distinct name when re-testing a cell (e.g. `…-postfix`) or the earlier evidence is overwritten.
- `restore-2026-shvdn.ps1` — one-command rollback to the working install.
- `bundle-2022\`, `backup-2026-build\`, `baseline\` — the components and the pre-test hashes.
- `build-2022api\` — compiles `src\` against the bundle's 2022 v2 dll with the game-copy targets redirected away.
- `runs\<profile>\` — per-cell evidence.

**Separate lab, different question** — `Util\shvdne-lab\` (gitignored, same as above) holds the SHVDNE work: `SurfaceProbe\` (the signature-exact metadata comparator, which also reads the mixed-mode asi), `release\` (the two SHVDNE artifacts), `build-shvdne\` (compiles the **live** `src\` against SHVDNE's v2 dll — deliberately no `src\` copy, so it cannot go stale), and `FINDINGS.md`. Recreating the trimmed clone and release is documented there.

## Lesson worth keeping

`build-2022api` compiled **clean, zero errors** against the 2022 v2 dll, and that run still died. Compile-clean proves only that the *API assembly's* surface is satisfied; the failure lives in the asi's **bridge**, which no compiler sees. Do not treat a clean compile as evidence of runtime compatibility when the dependency is a native bridge.

## Open

- **SHVDNE is statically analysed, never run** — the missing evidence is one Legacy game session on it (see the section above). Its Enhanced-edition claim is also untested here and out of scope for ARS.
- **v3-only** install (modern asi, no `ScriptHookVDotNet2.dll`): predicted dead, never run — no modern v3 dll exists on this machine (the only one is the bundle's 2022 build, itself broken on current game builds). Run `v3only-2026asi` if the signature is needed.
- The modern **v3 channel's** own asi/dll pairing is unknown.
- **The v3 migration is the exit from this trap**: the nightly v2 dll marks its own API `[Obsolete] … use the v3 API instead`, SHVDN's log calls it deprecated and warns it "may completely stop being supported", and depending on the v2 bridge is exactly what makes ARS hostage to build matching. `LemonUI.SHVDN3.dll` is already on disk (`Downloads\LemonUI\SHVDN3\`). Migration not yet costed.
