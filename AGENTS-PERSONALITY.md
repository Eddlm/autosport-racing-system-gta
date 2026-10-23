# ARS — Personality: giving the field a character each

**Read this when**: a request or an idea touches **personality, character, archetypes, per-racer behaviour, "this car drives like it knows what it's doing", "that one is timid / reckless", aggression as identity, per-car tuning of lane changes, brake confidence, pressure, frustration, rivalry, "give the AI some intent"**.

Status: **designed, not built.** Nothing here exists in `src` yet. Read it as a spec, not a description.

## The idea in one sentence

A personality is **three numbers that never change and two that do**: the constants say who the car is, the variables say how its race is going right now.

## The finding that shapes the design

Most of what this needs **already exists in the code, unnamed**:

| Already there | Where | What it is |
|---|---|---|
| `Aggression` 0-100 | `Racer.cs` | Moves the avoidance buffer and the pressure target |
| `_brakeConfidenceBias` | `Racer.cs`, drawn at init and logged | A permanent courage offset, one per racer |
| `Pressure` | rival model | Grows with proximity × aggression, falls fast; its only consumer is Yield |
| Per-apex `BrakeFactor` | corner context | Grows and shrinks as the learner scores corners |

So the work is not a new subsystem. It is **naming these together** and adding the two or three that are missing.

## The elements, simplest first

Cost is the reason for the order: the first three are one multiplier on an existing constant — no new state, nothing to desynchronise, revertible with a number. The last two need memory, decay and proof that they read.

| # | Element | Kind | What it does on track | Cost |
|---|---|---|---|---|
| 1 | **Lane-change rate** | constant | Multiplies `LaneMoverAccelMps2` and `LaneMoverSpeedCapMps`. Slow reads as doubt, normal as competence, fast as risk | one line |
| 2 | **Apex commitment** | constant | Multiplies `IdealLineLeadSeconds` / `IdealLineLeadMultiple`. Low enters late and safe, high attacks the vertex | one line |
| 3 | **Braking anticipation** | constant | Multiplies `BrakeHorizonMargin`. High brakes early (prudent), low leaves it late | one line |
| 4 | **Confidence** | constant | The existing `_brakeConfidenceBias` — the bias the car is born with | none |
| 5 | **Learning rate** | constant | `BrakeLearningGain` per car: learns a corner in two passes, or repeats the mistake all race | none |
| 6 | **Aggression** | constant | The existing 0-100: avoidance buffer and pressure | none |
| 7 | **Pressure** | variable | The existing one: rises near a rival, decays when clear | none |
| 8 | **Corner confidence** | variable | The existing per-apex `BrakeFactor`, raised and lowered by the learner | none |
| 9 | **Frustration** | variable | Grows while stuck behind a slower car, cools in clean air; temporarily raises aggression | new state |
| 10 | **Rivalry** | variable | Memory of who passed you; raises risk when that car is near | new state |

## Archetypes are how this becomes legible

Do not hand-tune per car. **Three archetypes and a jitter**:

| Archetype | Lane | Apex | Braking | Reads as |
|---|---|---|---|---|
| **Timid** | 0.6 | 0.8 | 1.2 | "That one is not going to risk it" |
| **Competent** | 1.0 | 1.0 | 1.0 | "That one knows what it is doing" |
| **Reckless** | 1.4 | 1.2 | 0.85 | "That one is going to break something" |

Plus ±10 % per car. Nine cars then read as nine characters out of three archetypes, and nothing else has to move.

## The rules that keep it from becoming noise

1. **Personality may not touch safety.** Avoidance and contact keep a global floor. A "reckless" car takes risks; it does not ram.
2. **Pace is not personality.** With `PaceIndex` equalising lap times, a trait that only changes speed is invisible. Every element above changes **where the car puts itself and when it decides**, not how fast it runs.
3. **Hysteresis or it reads as broken.** A car that dithers between two decisions every corner looks defective, not decisive. A trait must bias one way and stay there.
4. **The legibility test**: if the difference cannot be described in **one word** after two corners, it is noise, not character.

## The one worth adding first, beyond the table

**How a car leaves an error.** Today `OffTrackRecoverThrottle` is one value for the whole field. Made a trait, it gives a very visible signature — one car rejoins with rage, another accepts the mistake and settles — and it costs almost nothing, because the system already exists.

## Where the state, when it comes, should live

One `Personality` object per racer, constructed once with the archetype plus its jitter, holding the constants, and exposing the variables as fields the race loop reads and writes:

- constants: lane rate, apex commitment, braking anticipation, confidence, learning rate;
- variables: pressure (existing), corner confidence (existing, per corner), frustration and rivalry (new), each with a growth rate and a decay rate rather than a fixed value.

The test of the design is not that a car has twelve numbers. It is that a spectator can say **"that one is the one that brakes late"** without being told which car is which.
