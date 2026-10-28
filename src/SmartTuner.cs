// Per-car pace and grip tuning from completed laps. Writes results back as settings, never mid-race.
using GTA;
using GTA.Native;
using System;
using System.Collections.Generic;

namespace ARS
{

    internal static class SmartTuner
    {
        enum Style { Clean, Racing, Stripes, Muscle, Offroad, Tuner, Beater }

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
            new KeyValuePair<string, Style>("globe oil", Style.Racing),
            new KeyValuePair<string, Style>("ragga rum", Style.Racing),
            new KeyValuePair<string, Style>("atomic", Style.Racing),
            new KeyValuePair<string, Style>("stripe", Style.Stripes),
            new KeyValuePair<string, Style>("flame", Style.Muscle),
            new KeyValuePair<string, Style>("scallop", Style.Muscle),
            new KeyValuePair<string, Style>("tribal", Style.Muscle),
            new KeyValuePair<string, Style>("graffiti", Style.Tuner),
            new KeyValuePair<string, Style>("tagged", Style.Tuner),
            new KeyValuePair<string, Style>("abstraction", Style.Tuner),
            new KeyValuePair<string, Style>("geometric", Style.Tuner),
            new KeyValuePair<string, Style>("halftone", Style.Tuner),

            new KeyValuePair<string, Style>("yogarishima", Style.Tuner),
            new KeyValuePair<string, Style>("teast", Style.Tuner),
            new KeyValuePair<string, Style>("tenshun", Style.Tuner),
            new KeyValuePair<string, Style>("kabel", Style.Tuner),
            new KeyValuePair<string, Style>("hyper function", Style.Tuner),
            new KeyValuePair<string, Style>("jackal", Style.Tuner),
            new KeyValuePair<string, Style>("wasabi", Style.Tuner),
            new KeyValuePair<string, Style>("dense", Style.Tuner),
            new KeyValuePair<string, Style>("hardstand", Style.Tuner),
            new KeyValuePair<string, Style>("stance andreas", Style.Tuner),
            new KeyValuePair<string, Style>("haiso", Style.Tuner),
            new KeyValuePair<string, Style>("higashi", Style.Tuner),
            new KeyValuePair<string, Style>("minami", Style.Tuner),
            new KeyValuePair<string, Style>("nishi", Style.Tuner),
            new KeyValuePair<string, Style>("kita", Style.Tuner),
            new KeyValuePair<string, Style>("missile", Style.Tuner),
            new KeyValuePair<string, Style>("prolaps", Style.Tuner),
            new KeyValuePair<string, Style>("neds", Style.Tuner),
        };

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

        static readonly Dictionary<string, KeyValuePair<string[], string[]>> Brands = new Dictionary<string, KeyValuePair<string[], string[]>>
        {
            { "redwood", new KeyValuePair<string[], string[]>(new[] { "white" }, new[] { "red", "yellow" }) },
            { "sprunk", new KeyValuePair<string[], string[]>(new[] { "white", "green" }, new[] { "green" }) },
            { "ecola", new KeyValuePair<string[], string[]>(new[] { "white", "red" }, new[] { "red" }) },
            { "pisswasser", new KeyValuePair<string[], string[]>(new[] { "black", "white" }, new[] { "red", "white", "gold" }) },
            { "globe oil", new KeyValuePair<string[], string[]>(new[] { "white", "orange" }, new[] { "orange", "blue" }) },
            { "xero gas", new KeyValuePair<string[], string[]>(new[] { "white", "black" }, new[] { "red", "white", "blue" }) },
            { "fukaru", new KeyValuePair<string[], string[]>(new[] { "white", "red" }, new[] { "red", "white" }) },
            { "atomic", new KeyValuePair<string[], string[]>(new[] { "blue", "yellow", "white" }, new[] { "yellow", "blue" }) },
            { "ron", new KeyValuePair<string[], string[]>(new[] { "blue", "orange" }, new[] { "orange", "yellow", "blue" }) },
            { "ltd", new KeyValuePair<string[], string[]>(new[] { "white", "blue" }, new[] { "blue", "white" }) },
            { "prolaps", new KeyValuePair<string[], string[]>(new[] { "black", "white" }, new[] { "orange", "white" }) },
            { "sessanta", new KeyValuePair<string[], string[]>(new[] { "black", "cream" }, new[] { "brown", "gold" }) },
            { "meinmacht", new KeyValuePair<string[], string[]>(new[] { "blue", "white" }, new[] { "blue", "white" }) },
            { "hardstand", new KeyValuePair<string[], string[]>(new[] { "black", "white" }, new[] { "yellow", "orange" }) },
            { "yogarishima", new KeyValuePair<string[], string[]>(new[] { "blue", "white" }, new[] { "blue", "white" }) },
            { "grotti", new KeyValuePair<string[], string[]>(new[] { "red", "white" }, new[] { "red", "white" }) },
            { "ocelot", new KeyValuePair<string[], string[]>(new[] { "white", "blue" }, new[] { "blue", "red", "white" }) },
            { "dewbauchee", new KeyValuePair<string[], string[]>(new[] { "white", "black" }, new[] { "yellow", "black", "white" }) },
            { "karin", new KeyValuePair<string[], string[]>(new[] { "white", "blue" }, new[] { "blue", "gold" }) },
            { "maibatsu", new KeyValuePair<string[], string[]>(new[] { "white", "black", "silver" }, new[] { "red" }) },
            { "vulcar", new KeyValuePair<string[], string[]>(new[] { "blue", "black" }, new[] { "orange" }) },
            { "annis", new KeyValuePair<string[], string[]>(new[] { "white", "silver", "black" }, new[] { "blue" }) },
            { "vapid", new KeyValuePair<string[], string[]>(new[] { "white", "black", "silver" }, new[] { "red" }) },
        };

        static readonly string[] Marques = { "ubermacht", "benefactor", "pegassi", "truffade", "coil", "willard", "dinka", "obey", "bravado", "declasse", "fathom" };

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

        static readonly Dictionary<string, VehicleColor[]> MatteByFamily = new Dictionary<string, VehicleColor[]>
        {
            { "white", new[] { VehicleColor.MatteWhite } },
            { "black", new[] { VehicleColor.MatteBlack } },
            { "red", new[] { VehicleColor.MatteRed, VehicleColor.MatteDarkRed } },
            { "orange", new[] { VehicleColor.MatteOrange } },
            { "yellow", new[] { VehicleColor.MatteYellow } },
            { "green", new[] { VehicleColor.MatteGreen, VehicleColor.MatteForestGreen, VehicleColor.MatteOliveDrab, VehicleColor.MatteFoliageGreen } },
            { "blue", new[] { VehicleColor.MatteBlue, VehicleColor.MatteDarkBlue, VehicleColor.MatteMidnightBlue } },
            { "purple", new[] { VehicleColor.MattePurple, VehicleColor.MatteDarkPurple } },
            { "brown", new[] { VehicleColor.MatteBrown, VehicleColor.MatteDesertBrown } },
            { "gold", new[] { VehicleColor.MatteDesertTan } },
            { "silver", new[] { VehicleColor.MatteGray, VehicleColor.MatteLightGray } },
            { "cream", new[] { VehicleColor.MatteDesertTan, VehicleColor.MatteWhite } },
        };

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
                if (Game.Player.Character.IsInVehicle(r.Car)) continue;
                if (r.Car.GetMod(VehicleMod.Engine) != -1) continue;
                if (ARS.CanWeUse(r.Car)) Pending.Enqueue(r.Car);
            }
        }

        internal static void Clear()
        {
            Pending.Clear();
        }

        internal static void Tick(Func<int, int, int> random)
        {
            if (Pending.Count == 0) return;
            Vehicle veh = Pending.Dequeue();
            if (!ARS.SmartTuning || !ARS.CanWeUse(veh) || !veh.Exists()) return;

            if (Game.Player.Character.IsInVehicle(veh)) return;
            Apply(veh, random);
        }

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
            ApplyPaint(veh, style, livery >= 0 ? names[livery] : null, random);
        }

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
            Style picked = styles[random(0, styles.Count)];
            List<int> options = groups[picked];
            liveryIndex = options[random(0, options.Count)];
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

        static void ApplyParts(Vehicle veh, Style style, Func<int, int, int> random)
        {
            foreach (VehicleMod slot in Parts[style])
            {
                int count = veh.GetModCount(slot);
                if (count <= 0) continue;
                if (random(0, 100) >= 75) continue;
                veh.SetMod(slot, random(0, count), false);
            }
        }

        static void ApplyPaint(Vehicle veh, Style style, string liveryName, Func<int, int, int> random)
        {
            List<string> named = ColoursIn(liveryName);
            string brand = BrandIn(liveryName);

            VehicleColor rims = PickPaint(Neutrals, random);
            VehicleColor body;
            VehicleColor accent;
            if (brand != null)
            {

                KeyValuePair<string[], string[]> rule = Brands[brand];
                body = PreferStated(rule.Key, named, random);
                accent = PreferStated(rule.Value, named, random);
                rims = PickPaint(rule.Value, random);
            }
            else if (IsMarque(liveryName))
            {

                body = PickPaint(Neutrals, random);
                accent = named.Count > 0 ? ColourOf(named[named.Count - 1]) : PickPaint(Neutrals, random);
            }
            else if (named.Count >= 2)
            {

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

                body = PickPaint(AllFamilies, random);
                accent = PickPaint(Neutrals, random);
            }

            if (style == Style.Beater)
            {
                body = MatteFor(body, random);
                accent = MatteFor(accent, random);
            }

            veh.PrimaryColor = body;
            veh.SecondaryColor = accent;
            veh.PearlescentColor = VehicleColor.MetallicBlack;
            veh.RimColor = rims;
        }

        static string BrandIn(string liveryName)
        {
            if (string.IsNullOrEmpty(liveryName)) return null;
            string lower = liveryName.ToLowerInvariant();
            foreach (string brand in Brands.Keys)
            {
                if (ContainsWord(lower, brand)) return brand;
            }
            return null;
        }

        static bool IsMarque(string liveryName)
        {
            if (string.IsNullOrEmpty(liveryName)) return false;
            string lower = liveryName.ToLowerInvariant();
            foreach (string marque in Marques)
            {
                if (ContainsWord(lower, marque)) return true;
            }
            return false;
        }

        static bool ContainsWord(string haystack, string word)
        {
            int at = haystack.IndexOf(word, StringComparison.Ordinal);
            while (at >= 0)
            {
                bool leftOk = at == 0 || !char.IsLetter(haystack[at - 1]);
                int after = at + word.Length;
                bool rightOk = after >= haystack.Length || !char.IsLetter(haystack[after]);
                if (leftOk && rightOk) return true;
                at = haystack.IndexOf(word, at + 1, StringComparison.Ordinal);
            }
            return false;
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
            return options[random(0, options.Count)];
        }

        static VehicleColor PreferStated(string[] families, List<string> named, Func<int, int, int> random)
        {
            foreach (string colour in named)
            {
                if (Array.IndexOf(families, colour) >= 0) return PickPaint(new[] { colour }, random);
            }
            return PickPaint(families, random);
        }

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

        static VehicleColor MatteFor(VehicleColor colour, Func<int, int, int> random)
        {
            foreach (KeyValuePair<VehicleColor, string> entry in Palette)
            {
                if (entry.Key != colour) continue;
                VehicleColor[] matte;
                if (MatteByFamily.TryGetValue(entry.Value, out matte)) return matte[random(0, matte.Length)];
                break;
            }
            return VehicleColor.MatteGray;
        }

        static string LabelText(string label)
        {
            if (string.IsNullOrEmpty(label)) return null;
            string text = Function.Call<string>((Hash)0x7B5280EBA9840C72, label);
            return string.IsNullOrEmpty(text) || text == "NULL" ? null : text;
        }
    }
}
