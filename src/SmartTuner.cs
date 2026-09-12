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

        // Brand liveries imply their own colours: the artwork is the brand's, so the paint should be too. The
        // value is (preferred body families, the colour the livery artwork carries). Redwood's artwork is red
        // and reads best on white (user, 2026-10). Add brands here as they come up - the keyword table already
        // sends them to a Racing build, so the parts follow for free.
        static readonly Dictionary<string, KeyValuePair<string[], string>> Brands = new Dictionary<string, KeyValuePair<string[], string>>
        {
            { "redwood", new KeyValuePair<string[], string>(new[] { "white" }, "red") },
        };

        // Body paints, grouped so a livery that names a colour can pull from its whitelist. **Metallic only, by
        // request** - no matte/worn/util/chrome variants. Every member was checked against GTA.VehicleColor by
        // reflection; keep that rule when adding colours, and keep at least one member per family.
        static readonly KeyValuePair<VehicleColor, string>[] Palette =
        {
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicWhite, "white"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicFrostWhite, "white"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBlack, "black"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicGraphiteBlack, "black"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBlackSteel, "black"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicAnthraciteGray, "black"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicFormulaRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicTorinoRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicCabernetRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicCandyRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicLavaRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBlazeRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicGracefulRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicGarnetRed, "red"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSunriseOrange, "orange"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicOrange, "orange"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicTaxiYellow, "yellow"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicRaceYellow, "yellow"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicYellowBird, "yellow"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicLime, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicDarkGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicRacingGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSeaGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicOliveGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicGasolineBlueGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSecuricorGreen, "green"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicDarkBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicMidnightBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicUltraBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSaxonyBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicMarinerBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicHarborBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicDiamondBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSurfBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicNauticalBlue, "blue"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicPurple, "purple"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBlackPurple, "purple"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicChocoBrown, "brown"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicLightBrown, "brown"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicGoldenBrown, "brown"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicMossBrown, "brown"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBistonBrown, "brown"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBeechwood, "brown"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicClassicGold, "gold"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBronze, "gold"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSilver, "silver"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicSteelGray, "silver"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicBlueSilver, "silver"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicStoneSilver, "silver"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicShadowSilver, "silver"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicCream, "cream"),
            new KeyValuePair<VehicleColor, string>(VehicleColor.MetallicChampagne, "cream"),
        };

        static readonly string[] ColourWords = { "white", "black", "red", "blue", "green", "yellow", "orange", "silver", "gold", "purple", "brown", "cream", "grey", "gray" };
        static readonly string[] AllFamilies = { "white", "black", "red", "blue", "green", "yellow", "orange", "silver", "gold", "purple", "brown", "cream" };
        static readonly string[] Neutrals = { "black", "white", "silver" };

        // WHITELIST: the body colours that sit well with a livery of this colour. Anything unlisted is out - a
        // blacklist ("anything but white") plus a uniform draw is what produced muddy pairings, while these sets
        // stay broad enough (4+ families) that a pick is still a surprise.
        static readonly Dictionary<string, string[]> BodyWith = new Dictionary<string, string[]>
        {
            { "white", new[] { "red", "black", "blue", "orange", "green", "purple", "gold" } },
            { "black", new[] { "white", "silver", "red", "yellow", "blue", "orange", "green" } },
            { "red", new[] { "black", "white", "silver", "blue", "gold", "cream" } },
            { "blue", new[] { "white", "silver", "orange", "yellow", "red", "black", "gold" } },
            { "green", new[] { "black", "white", "cream", "gold", "brown", "silver" } },
            { "yellow", new[] { "black", "red", "blue", "green", "silver", "orange" } },
            { "orange", new[] { "black", "blue", "white", "brown", "silver", "green" } },
            { "silver", new[] { "black", "red", "blue", "green", "purple", "orange" } },
            { "gold", new[] { "black", "white", "green", "blue", "brown", "red" } },
            { "purple", new[] { "black", "silver", "white", "yellow", "gold" } },
            { "brown", new[] { "cream", "white", "black", "orange", "silver", "gold" } },
            { "cream", new[] { "black", "brown", "red", "green", "gold", "blue" } },
        };

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

        // A livery the car actually offers, and which space it came from: modern cars carry liveries as mod slot
        // 48 (GET_MOD_TEXT_LABEL, applied with SET_VEHICLE_MOD), older ones in the livery list (GET_LIVERY_NAME,
        // applied with SET_VEHICLE_LIVERY). Enumerating only the latter is why the first test found no names.
        struct LiveryOption
        {
            public int Index;
            public string Name;
            public bool IsMod;
        }

        static List<LiveryOption> Options(Vehicle veh)
        {
            List<LiveryOption> options = new List<LiveryOption>();
            int modCount = veh.GetModCount(VehicleMod.Livery);
            for (int i = 0; i < modCount; i++)
            {
                string label = Function.Call<string>(Hash.GET_MOD_TEXT_LABEL, veh, (int)VehicleMod.Livery, i);
                options.Add(new LiveryOption { Index = i, Name = LabelText(label), IsMod = true });
            }
            if (options.Count == 0)
            {
                for (int i = 0; i < veh.LiveryCount; i++)
                {
                    string label = Function.Call<string>(Hash.GET_LIVERY_NAME, veh, i);
                    options.Add(new LiveryOption { Index = i, Name = LabelText(label), IsMod = false });
                }
            }
            return options;
        }

        static void Apply(Vehicle veh, Func<int, int, int> random)
        {
            veh.InstallModKit();

            List<LiveryOption> options = Options(veh);
            List<string> names = new List<string>();
            foreach (LiveryOption option in options) names.Add(option.Name);

            int livery;
            Style style = PickStyle(names, random, out livery);

            if (livery >= 0)
            {
                if (options[livery].IsMod) veh.SetMod(VehicleMod.Livery, options[livery].Index, false);
                else veh.Livery = options[livery].Index;
            }

            ARS.Log(ARS.LogImportance.Info, "Smart tune " + veh.DisplayName + ": " + style + ", " + options.Count
                + " liveries, " + (livery >= 0 ? names[livery] : "none named"));
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
            List<string> named = ColoursIn(liveryName);
            string brand = BrandIn(liveryName);

            VehicleColor body;
            VehicleColor accent;
            if (brand != null)
            {
                // A brand livery dictates the paint outright: body from the brand's preferred families, accent
                // from the colour its artwork carries.
                KeyValuePair<string[], string> rule = Brands[brand];
                body = PickPaint(rule.Key, random);
                accent = ColourOf(rule.Value);
            }
            else if (named.Count >= 2)
            {
                // A name like "Black Pfister White Stripe" states its own base and its own accent - honour it.
                body = ColourOf(named[0]);
                accent = ColourOf(named[named.Count - 1]);
            }
            else if (named.Count == 1)
            {
                body = PickPaint(FamiliesFor(named[0]), random);
                accent = ColourOf(named[0]);
            }
            else
            {
                // No livery, or a livery that names no colour: body stays a free pick, accent goes neutral so the
                // two draws cannot clash, and the rims never follow the accent.
                body = PickPaint(AllFamilies, random);
                accent = PickPaint(Neutrals, random);
            }

            veh.PrimaryColor = body;
            veh.SecondaryColor = accent;
            veh.PearlescentColor = VehicleColor.MetallicBlack;   // black pearl = no pearl tint, by request for now
            veh.RimColor = PickPaint(Neutrals, random);
        }

        static string BrandIn(string liveryName)
        {
            if (string.IsNullOrEmpty(liveryName)) return null;
            string lower = liveryName.ToLowerInvariant();
            foreach (string brand in Brands.Keys)
            {
                if (lower.Contains(brand)) return brand;
            }
            return null;
        }

        static string[] FamiliesFor(string family)
        {
            if (family == null) return AllFamilies;
            string[] families;
            return BodyWith.TryGetValue(family, out families) ? families : AllFamilies;
        }

        static VehicleColor PickPaint(string[] families, Func<int, int, int> random)
        {
            List<VehicleColor> options = new List<VehicleColor>();
            foreach (KeyValuePair<VehicleColor, string> entry in Palette)
            {
                if (Array.IndexOf(families, entry.Value) >= 0) options.Add(entry.Key);
            }
            if (options.Count == 0) return VehicleColor.MetallicBlack;
            return options[random(0, options.Count - 1)];
        }

        // Every colour the name mentions, in the order the name mentions them (the word list's own order is
        // irrelevant here - "Black Pfister White Stripe" must read as black-then-white, not white-first).
        static List<string> ColoursIn(string liveryName)
        {
            List<string> found = new List<string>();
            if (string.IsNullOrEmpty(liveryName)) return found;
            string lower = liveryName.ToLowerInvariant();
            int at = 0;
            while (at < lower.Length)
            {
                int best = -1;
                string bestWord = null;
                foreach (string word in ColourWords)
                {
                    int hit = lower.IndexOf(word, at);
                    if (hit < 0) continue;
                    if (best < 0 || hit < best || (hit == best && word.Length > bestWord.Length))
                    {
                        best = hit;
                        bestWord = word;
                    }
                }
                if (best < 0) break;
                string family = bestWord == "grey" || bestWord == "gray" ? "white" : bestWord;
                if (!found.Contains(family)) found.Add(family);
                at = best + bestWord.Length;
            }
            return found;
        }

        static VehicleColor ColourOf(string family)
        {
            foreach (KeyValuePair<VehicleColor, string> entry in Palette)
            {
                if (entry.Value == family) return entry.Key;
            }
            return VehicleColor.MetallicBlack;
        }

        // The natives hand back a GXT label; _GET_LABEL_TEXT turns it into the game's own text.
        static string LabelText(string label)
        {
            if (string.IsNullOrEmpty(label)) return null;
            string text = Function.Call<string>((Hash)0x7B5280EBA9840C72, label);
            return string.IsNullOrEmpty(text) || text == "NULL" ? null : text;
        }
    }
}
