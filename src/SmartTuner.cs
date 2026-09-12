using GTA;
using GTA.Native;
using System;
using System.Collections.Generic;

namespace ARS
{
    // Smart tuning: reads a car's own livery names, picks a style from those keywords, applies the livery,
    // fits the body parts that suit that style, and paints last (a livery can dirty the appearance otherwise).
    // Replaces the old random-index tuning. Runs post-spawn from a per-tick queue - one car per tick - so the
    // race start is not extended and the mod-part streaming is spread over frames instead of landing in a burst.
    // Enum member names here were taken from GTA.VehicleMod / GTA.VehicleColor by reflection on SHVDN2, not guessed.
    internal static class SmartTuner
    {
        enum Style { Clean, Racing, Stripes, Muscle, Offroad, Tuner, Beater }

        // Per-name keyword match; first hit wins, so the order is deliberate (Rusty before Racing, etc).
        static readonly KeyValuePair<string, Style>[] Keywords =
        {
            new KeyValuePair<string, Style>("rusty", Style.Beater),
            new KeyValuePair<string, Style>("rat look", Style.Beater),
            new KeyValuePair<string, Style>("ratlook", Style.Beater),
            new KeyValuePair<string, Style>("junkyard", Style.Beater),
            new KeyValuePair<string, Style>("primed", Style.Beater),
            new KeyValuePair<string, Style>("camo", Style.Offroad),
            new KeyValuePair<string, Style>("safari", Style.Offroad),
            new KeyValuePair<string, Style>("naval", Style.Offroad),
            new KeyValuePair<string, Style>("forest", Style.Offroad),
            new KeyValuePair<string, Style>("desert", Style.Offroad),
            new KeyValuePair<string, Style>("expedition", Style.Offroad),
            new KeyValuePair<string, Style>("rally", Style.Racing),
            new KeyValuePair<string, Style>("racing", Style.Racing),
            new KeyValuePair<string, Style>("race", Style.Racing),
            new KeyValuePair<string, Style>("works", Style.Racing),
            new KeyValuePair<string, Style>("redwood", Style.Racing),
            new KeyValuePair<string, Style>("fukaru", Style.Racing),
            new KeyValuePair<string, Style>("xero gas", Style.Racing),
            new KeyValuePair<string, Style>("sprunk", Style.Racing),
            new KeyValuePair<string, Style>("ecola", Style.Racing),
            new KeyValuePair<string, Style>("pisswasser", Style.Racing),
            new KeyValuePair<string, Style>("meinmacht", Style.Racing),
            new KeyValuePair<string, Style>("sessanta", Style.Racing),
            new KeyValuePair<string, Style>("jackal", Style.Racing),
            new KeyValuePair<string, Style>("globe oil", Style.Racing),
            new KeyValuePair<string, Style>("ragga rum", Style.Racing),
            new KeyValuePair<string, Style>("atomic", Style.Racing),
            new KeyValuePair<string, Style>("tenshun", Style.Racing),
            new KeyValuePair<string, Style>("kabel", Style.Racing),
            new KeyValuePair<string, Style>("hyper function", Style.Racing),
            new KeyValuePair<string, Style>("stripe", Style.Stripes),
            new KeyValuePair<string, Style>("pinstripe", Style.Stripes),
            new KeyValuePair<string, Style>("flame", Style.Muscle),
            new KeyValuePair<string, Style>("scallop", Style.Muscle),
            new KeyValuePair<string, Style>("tribal", Style.Muscle),
            new KeyValuePair<string, Style>("graffiti", Style.Tuner),
            new KeyValuePair<string, Style>("tagged", Style.Tuner),
            new KeyValuePair<string, Style>("abstraction", Style.Tuner),
            new KeyValuePair<string, Style>("geometric", Style.Tuner),
            new KeyValuePair<string, Style>("halftone", Style.Tuner),
        };

        // Cosmetic slots per style. Performance slots (Engine, Brakes, Transmission, Suspension) and their
        // toggles are deliberately absent: this pass is appearance only.
        static readonly Dictionary<Style, VehicleMod[]> Parts = new Dictionary<Style, VehicleMod[]>
        {
            { Style.Racing, new[] { VehicleMod.Spoilers, VehicleMod.FrontBumper, VehicleMod.RearBumper, VehicleMod.SideSkirt, VehicleMod.Exhaust, VehicleMod.Frame, VehicleMod.Roof, VehicleMod.PlateHolder, VehicleMod.TrimDesign, VehicleMod.Windows } },
            { Style.Stripes, new[] { VehicleMod.SideSkirt, VehicleMod.Exhaust, VehicleMod.PlateHolder, VehicleMod.TrimDesign, VehicleMod.Ornaments, VehicleMod.Windows } },
            { Style.Muscle, new[] { VehicleMod.Hood, VehicleMod.Exhaust, VehicleMod.Frame, VehicleMod.Hydraulics, VehicleMod.Seats, VehicleMod.SteeringWheels, VehicleMod.TrimDesign } },
            { Style.Offroad, new[] { VehicleMod.Roof, VehicleMod.Frame, VehicleMod.FrontBumper, VehicleMod.RearBumper, VehicleMod.Trunk, VehicleMod.Aerials } },
            { Style.Tuner, new[] { VehicleMod.SideSkirt, VehicleMod.TrimDesign, VehicleMod.Seats, VehicleMod.Dashboard, VehicleMod.DialDesign, VehicleMod.Speakers, VehicleMod.SteeringWheels, VehicleMod.ColumnShifterLevers } },
            { Style.Clean, new[] { VehicleMod.SideSkirt, VehicleMod.PlateHolder, VehicleMod.Windows } },
            { Style.Beater, new[] { VehicleMod.Exhaust, VehicleMod.PlateHolder, VehicleMod.Ornaments, VehicleMod.Trunk, VehicleMod.Hydraulics, VehicleMod.VanityPlates } },
        };

        // Body paints, grouped so a livery that names a colour can veto its own family (white stripes on a
        // near-white body would not read). Everything else stays in play on purpose: variety over matching.
        static readonly KeyValuePair<VehicleColor, string>[] Palette =
        {
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicWhite, "white"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicFrostWhite, "white"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSilver, "white"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicCream, "white"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.WornWhite, "white"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MatteWhite, "white"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBlack, "black"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicGraphiteBlack, "black"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBlackSteel, "black"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MatteBlack, "black"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.WornBlack, "black"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicFormulaRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicTorinoRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicCabernetRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicCandyRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicLavaRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MatteRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.WornRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSunriseOrange, "orange"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicOrange, "orange"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MatteOrange, "orange"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.WornOrange, "orange"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicTaxiYellow, "yellow"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicRaceYellow, "yellow"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MatteYellow, "yellow"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicLime, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicDarkGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicRacingGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSeaGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicOliveGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MatteGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MatteForestGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.HunterGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.WornGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicDarkBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicMidnightBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicUltraBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSaxonyBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicMarinerBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MatteBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.WornBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicPurple, "purple"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBlackPurple, "purple"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MattePurple, "purple"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicChocoBrown, "brown"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicLightBrown, "brown"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicGoldenBrown, "brown"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MatteBrown, "brown"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicClassicGold, "gold"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.PureGold, "gold"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.BrushedGold, "gold"),
        };

        static readonly string[] ColourWords = { "white", "black", "red", "blue", "green", "yellow", "orange", "silver", "gold", "purple", "brown", "cream", "grey", "gray" };

        static readonly Queue<Vehicle> Pending = new Queue<Vehicle>();

        internal static void Enqueue(IEnumerable<Racer> racers)
        {
            Pending.Clear();
            if (!ARS.SmartTuning) return;
            foreach (Racer r in racers)
            {
                if (Game.Player.Character.IsInVehicle(r.Car)) continue;   // never restyle the player's own car
                if (r.Car.GetMod(VehicleMod.Engine) != -1) continue;      // leave cars that are already tuned alone
                if (ARS.CanWeUse(r.Car)) Pending.Enqueue(r.Car);
            }
        }

        internal static void Clear()
        {
            Pending.Clear();
        }

        // One car per tick: bounded work, no Script.Wait, and the streaming cost stays spread out.
        internal static void Tick(Func<int, int, int> random)
        {
            if (Pending.Count == 0) return;
            Vehicle veh = Pending.Dequeue();
            if (!ARS.SmartTuning || !ARS.CanWeUse(veh) || !veh.Exists()) return;
            Apply(veh, random);
        }

        static void Apply(Vehicle veh, Func<int, int, int> random)
        {
            veh.InstallModKit();

            List<string> names = new List<string>();
            for (int i = 0; i < veh.LiveryCount; i++) names.Add(LiveryName(veh, i));

            int livery;
            Style style = PickStyle(names, random, out livery);

            if (livery >= 0) veh.Livery = livery;
            ARS.Log(ARS.LogImportance.Info, "Smart tune " + veh.DisplayName + ": " + style + ", livery " + (livery >= 0 ? names[livery] : "(none)"));
            ApplyParts(veh, style, random);
            ApplyPaint(veh, livery >= 0 ? names[livery] : null, random);
        }

        // Groups the car's livery names by style and takes a random non-empty group, so a car with both rally
        // and camo liveries can come out either way. Unmatched names land in Clean rather than being dropped.
        static Style PickStyle(List<string> names, Func<int, int, int> random, out int liveryIndex)
        {
            liveryIndex = -1;
            Dictionary<Style, List<int>> groups = new Dictionary<Style, List<int>>();
            for (int i = 0; i < names.Count; i++)
            {
                if (string.IsNullOrEmpty(names[i])) continue;
                Style s = MatchStyle(names[i]);
                if (!groups.ContainsKey(s)) groups.Add(s, new List<int>());
                groups[s].Add(i);
            }
            if (groups.Count == 0) return Style.Clean;

            List<Style> styles = new List<Style>(groups.Keys);
            Style picked = styles[random(0, styles.Count - 1)];
            List<int> options = groups[picked];
            liveryIndex = options[random(0, options.Count - 1)];
            return picked;
        }

        static Style MatchStyle(string name)
        {
            string lower = name.ToLowerInvariant();
            foreach (KeyValuePair<string, Style> pair in Keywords)
            {
                if (lower.Contains(pair.Key)) return pair.Value;
            }
            return Style.Clean;
        }

        // Most slots of the style's set get filled, but not all of them - two cars of the same style should not
        // come out identical.
        static void ApplyParts(Vehicle veh, Style style, Func<int, int, int> random)
        {
            foreach (VehicleMod slot in Parts[style])
            {
                int count = veh.GetModCount(slot);
                if (count <= 0) continue;
                if (random(0, 99) >= 75) continue;
                veh.SetMod(slot, random(0, count - 1), false);
            }
        }

        static void ApplyPaint(Vehicle veh, string liveryName, Func<int, int, int> random)
        {
            string stated = StatedColour(liveryName);

            List<VehicleColor> body = new List<VehicleColor>();
            foreach (KeyValuePair<VehicleColor, string> entry in Palette)
            {
                if (stated != null && entry.Value == stated) continue;
                body.Add(entry.Key);
            }
            if (body.Count == 0) body.Add(VehicleColor.MetallicBlack);

            veh.PrimaryColor = body[random(0, body.Count - 1)];
            veh.SecondaryColor = stated != null ? ColourOf(stated) : Palette[random(0, Palette.Length - 1)].Key;
            veh.PearlescentColor = body[random(0, body.Count - 1)];
            veh.RimColor = veh.SecondaryColor;
        }

        // The colour named by the livery, if it names one. Only used as a veto on the body and as the accent -
        // never as a mandate, so "White Stripes" can land on any body colour that is not near-white.
        static string StatedColour(string liveryName)
        {
            if (string.IsNullOrEmpty(liveryName)) return null;
            string lower = liveryName.ToLowerInvariant();
            foreach (string word in ColourWords)
            {
                if (lower.Contains(word)) return word == "grey" || word == "gray" ? "white" : word;
            }
            return null;
        }

        static VehicleColor ColourOf(string family)
        {
            foreach (KeyValuePair<VehicleColor, string> entry in Palette)
            {
                if (entry.Value == family) return entry.Key;
            }
            return VehicleColor.MetallicBlack;
        }

        // GET_LIVERY_NAME gives the GXT label; _GET_LABEL_TEXT turns it into the game's own text.
        static string LiveryName(Vehicle veh, int index)
        {
            string label = Function.Call<string>(Hash.GET_LIVERY_NAME, veh, index);
            if (string.IsNullOrEmpty(label)) return null;
            string text = Function.Call<string>((Hash)0x7B5280EBA9840C72, label);
            return string.IsNullOrEmpty(text) || text == "NULL" ? null : text;
        }
    }
}
