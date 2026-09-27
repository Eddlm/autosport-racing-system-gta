using System;
using System.Collections.Generic;
using System.IO;

namespace ARS
{
    // How often a passed apex rolls for a tip.
    public enum TipFrequency
    {
        High,
        Medium,
        Low
    }

    // One roll per apex the player passes; the tip goes out through the HelpMessages queue.
    public static class Tips
    {
        // The fallback list, written as the same tagged lines tips.txt ships with.
        static readonly string[] BuiltInTips =
        {
            "[circuit] ARS hands you a fresh nitrous bottle every lap - spend the one you have.",
            "[nitro] Racers can fire nitrous too - one shot a lap, on the straights.",
            "A racer hounding you will eventually dive down the inside. Cover the line or leave room.",
            "The grid is pace-matched to your car. Grid PI Mode > Relative uses your PI plus the offset.",
            "When the winner crosses the line, the Timeout grace period starts - finish before it runs out.",
            "Best laps are read out when the race ends.",
            "General Settings > Car Pool swaps which Vehicles roster the grid is drawn from.",
            "You can record your own route: ARS > Track Creator, then Save Track.",
            "[tuning-off] Smart Tuning picks liveries, body kits and paint for the grid before a race.",
            "Every racer gets its own aggression from its grid slot, so the field never drives as one."
        };

        class Tip
        {
            public string Text;
            public Func<bool> Applies;
        }

        // An optional leading tag limits when a tip may appear; no tag means any race.
        class TipTag
        {
            public string Tag;
            public Func<bool> Applies;
        }

        static readonly TipTag[] Tags =
        {
            new TipTag { Tag = "[circuit]", Applies = () => !ARS.IsPointToPoint },
            new TipTag { Tag = "[nitro]", Applies = () => ARS.AiNitro != TriState.Never },
            new TipTag { Tag = "[tuning-off]", Applies = () => !ARS.SmartTuning }
        };

        static readonly List<Tip> _all = new List<Tip>();
        static readonly List<Tip> _deck = new List<Tip>();
        static readonly Random _random = new Random();

        // Reloaded per grid, so an edited tips.txt applies to the next race without a restart.
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

        // The file wins when it is there; the shipped list keeps tips alive when it is not.
        static string[] TipLines()
        {
            try
            {
                string path = Path.Combine(ARS.ScriptsFolder, "tips.txt");
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

        // Only the tips that apply to this race, shuffled, popped without replacement.
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
                    case TipFrequency.High: return 20;
                    case TipFrequency.Low: return 100;
                    default: return 50;
                }
            }
        }

        public static void ApexPassed(int passedApexes)
        {
            if (ARS.RaceStatus != RaceState.InProgress) return;
            // Once the winner is in, the help box belongs to the timeout countdown.
            if (ARS.LeaderboardFinish.Count > 0) return;
            // A tip must never sit in front of a real message.
            if (ARS.HelpMessages.Count > 0) return;
            for (int i = 0; i < passedApexes; i++)
            {
                if (ARS.GetRandomInt(1, Denominator) != 1) continue;
                string tip = NextTip();
                if (tip == null) return;
                ARS.HelpMessages.Add(tip);
                return;
            }
        }

        // A repeat waits for the whole applicable list to be shown once.
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
