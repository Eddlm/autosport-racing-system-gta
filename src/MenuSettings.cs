using GTA;
using System.Globalization;

namespace ARS
{
    // One ini per menu: a single ScriptSettings object for Settings\Menu-<Name>.ini,
    // loaded lazily, written on every change. Never Load() the same file twice.
    public class MenuSettings
    {
        readonly string _path;
        ScriptSettings _file;

        public MenuSettings(string path)
        {
            _path = path;
        }

        ScriptSettings File
        {
            get { return _file ?? (_file = ScriptSettings.Load(_path)); }
        }

        public string Get(string key, string fallback)
        {
            SeedIfMissing(key, fallback);
            return File.GetValue<string>("MENU", key, fallback);
        }

        public bool GetBool(string key, bool fallback)
        {
            SeedIfMissing(key, fallback.ToString());
            return File.GetValue<bool>("MENU", key, fallback);
        }

        public int GetInt(string key, int fallback)
        {
            SeedIfMissing(key, fallback.ToString(CultureInfo.InvariantCulture));
            return File.GetValue<int>("MENU", key, fallback);
        }

        public float GetFloat(string key, float fallback)
        {
            SeedIfMissing(key, fallback.ToString(CultureInfo.InvariantCulture));
            return File.GetValue<float>("MENU", key, fallback);
        }

        // Seed-on-read: the ini mirrors the live key set after the first load.
        void SeedIfMissing(string key, string value)
        {
            if (value == null || File.GetValue<string>("MENU", key, null) != null) return;
            File.SetValue("MENU", key, value);
            File.Save();
        }

        public void Set(string key, string value)
        {
            File.SetValue("MENU", key, value);
            File.Save();
        }

        public void Set(string key, int value)
        {
            Set(key, value.ToString());
        }

        public void Set(string key, float value)
        {
            Set(key, value.ToString(CultureInfo.InvariantCulture));
        }

        // One-time seed: if the per-menu file lacks the key, carry the legacy value over.
        public void Migrate(string key, string legacyValue)
        {
            if (legacyValue == null) return;
            if (Get(key, null) == null) Set(key, legacyValue);
        }
    }
}