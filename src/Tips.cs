// The tip lines the HUD rotates through.
using System;
using System.Collections.Generic;
using System.IO;

namespace ARS
{

    public enum TipFrequency
    {
        High,
        Medium,
        Low
    }

    public static class Tips
    {

        static readonly string[] BuiltInTips =
        {
            "[circuit] ARS hands you a fresh nitrous bottle every lap - spend the one you have.",
            "[nitro] Racers can fire nitrous too - one shot a lap, on the straights.",
            "A racer hounding you will eventually dive down the inside. Cover the line or leave room.",
            "The grid is pace-matched to your car. Grid PI Mode > Relative uses your PI plus the offset.",
            "When the winner crosses the line, the Timeout grace period starts - finish before it runs out.",
            "Best laps are read out when the race ends.",
            "Rivals > Car Roster swaps which Vehicles roster the grid is drawn from.",
            "You can record your own route: ARS > Track Creator, then Save Track.",
            "[tuning-off] Auto Styling picks liveries, body kits and paint for the grid before a race.",
            "Every racer gets its own aggression from its grid slot, so the field never drives as one."
        };

        class Tip
        {
            public string Text;
            public Func<bool> Applies;
        }

        class TipTag
        {
            public string Tag;
            public Func<bool> Applies;
        }

        static readonly TipTag[] Tags =
        {
            new TipTag { Tag = "[circuit]", Applies = () => !ARS.IsPointToPoint },
            new TipTag { Tag = "[nitro]", Applies = () => ARS.NitrousEnabled },
            new TipTag { Tag = "[tuning-off]", Applies = () => !ARS.SmartTuning }
        };

        static readonly List<Tip> _all = new List<Tip>();
        static readonly List<Tip> _deck = new List<Tip>();
        static readonly Random _random = new Random();

        public static void Reset()
        {
            _all.Clear();
            foreach (string line in TipLines())
            {
                Tip tip = Parse(line);
                if (tip != null) _all.Add(tip);
            }
            Refill();
        }

        static string[] TipLines()
        {
            try
            {

                string path = Path.Combine(ARS.ScriptsFolder, "Settings", "tips.txt");
                if (!File.Exists(path)) path = Path.Combine(ARS.ScriptsFolder, "tips.txt");
                if (File.Exists(path)) return File.ReadAllLines(path);
            }
            catch (Exception) { }
            return BuiltInTips;
        }

        static Tip Parse(string line)
        {
            string text = (line ?? "").Trim();
            if (text.Length == 0 || text[0] == '#') return null;
            Func<bool> applies = null;
            foreach (TipTag tag in Tags)
            {
                if (!text.StartsWith(tag.Tag, StringComparison.OrdinalIgnoreCase)) continue;
                applies = tag.Applies;
                text = text.Substring(tag.Tag.Length).Trim();
                break;
            }
            return text.Length == 0 ? null : new Tip { Text = text, Applies = applies };
        }

        static void Refill()
        {
            _deck.Clear();
            for (int i = _all.Count - 1; i >= 0; i--)
            {
                if (_all[i].Applies == null || _all[i].Applies()) _deck.Add(_all[i]);
            }
            for (int i = _deck.Count - 1; i > 0; i--)
            {
                int j = _random.Next(i + 1);
                Tip swap = _deck[i];
                _deck[i] = _deck[j];
                _deck[j] = swap;
            }
        }

        static int Denominator
        {
            get
            {
                switch (ARS.TipRate)
                {
                    case TipFrequency.High: return 10;
                    case TipFrequency.Low: return 30;
                    default: return 20;
                }
            }
        }

        public static void ApexPassed(int passedApexes)
        {
            if (ARS.RaceStatus != RaceState.InProgress) return;

            if (ARS.LeaderboardFinish.Count > 0) return;

            for (int i = 0; i < passedApexes; i++)
            {

                if (ARS.GetRandomInt(1, Denominator + 1) != 1) continue;
                string tip = NextTip();
                if (tip == null) return;
                ARS.HelpMessages.Add(tip);
                return;
            }
        }

        static string NextTip()
        {
            if (_deck.Count == 0) Refill();
            if (_deck.Count == 0) return null;
            Tip tip = _deck[_deck.Count - 1];
            _deck.RemoveAt(_deck.Count - 1);
            return tip.Text;
        }
    }
}
