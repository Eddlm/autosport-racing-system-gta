using System;
using System.Linq;
using System.Reflection;

namespace ARS
{
    // Optional N.O.S. 1.9 API v2 adapter. No hard reference; ARS still loads without N.O.S.
    // The 1.9 contract offers per-car FlameStyle and Audio, NOT RGB or gradient assignment.
    internal static class NosAiNitro
    {
        static Type api, mode, options;
        static MethodInfo register, hold, release;
        static bool probed;
        static readonly string[] Styles = {
            "BlueNitrous", "BackfireOverdrive", "CinematicHybrid", "LegacyBlue", "NveOriginal"
        };
        static readonly Random rng = new Random();
        static readonly System.Collections.Generic.Dictionary<int, string> carStyles =
            new System.Collections.Generic.Dictionary<int, string>();

        static bool Probe()
        {
            if (probed) return api != null;
            try
            {
                Assembly assembly = AppDomain.CurrentDomain.GetAssemblies()
                    .FirstOrDefault(asm => asm.GetName().Name == "N.O.S - Nitro Overdrive System.API");
                // Assembly.Load by name only if already installed; failure is harmless.
                if (assembly == null) assembly = Assembly.Load("N.O.S - Nitro Overdrive System.API");
                if (assembly == null) return false;
                Type a = assembly.GetType("NitroOverdriveSystem.API.NOSApi");
                Type m = assembly.GetType("NitroOverdriveSystem.API.NOSAiMode");
                Type o = assembly.GetType("NitroOverdriveSystem.API.NOSAiOptions");
                if (a == null || m == null || o == null) return false;
                FieldInfo version = a.GetField("API_VERSION");
                if (version == null || Convert.ToInt32(version.GetValue(null)) < 2) return false;
                register = a.GetMethod("RegisterAiVehicle");
                hold = a.GetMethod("SetAiNitro");
                release = a.GetMethod("ReleaseAiVehicle");
                if (register == null || hold == null || release == null) return false;
                api = a; mode = m; options = o; probed = true;
                return true;
            }
            catch { return false; }
        }

        static bool Available()
        {
            try { return Probe() && (bool)api.GetProperty("SupportsAiNitro").GetValue(null, null); }
            catch { return false; }
        }

        public static bool Register(int vehicleHandle)
        {
            if (vehicleHandle == 0 || !Available()) return false;
            try
            {
                string style;
                if (!carStyles.TryGetValue(vehicleHandle, out style))
                {
                    style = Styles[rng.Next(Styles.Length)];
                    carStyles[vehicleHandle] = style;
                }
                object opts = Activator.CreateInstance(options);
                options.GetProperty("FlameStyle").SetValue(opts, style, null);
                options.GetProperty("Audio").SetValue(opts, true, null);
                object regen = Enum.ToObject(mode, 1);
                bool ok = (bool)register.Invoke(null, new[] { (object)vehicleHandle, regen, opts });
                if (!ok) carStyles.Remove(vehicleHandle);
                return ok;
            }
            catch { carStyles.Remove(vehicleHandle); return false; }
        }

        public static void Hold(int vehicleHandle, bool pressed)
        {
            try { if (api != null) hold.Invoke(null, new object[] { vehicleHandle, pressed }); }
            catch { }
        }

        public static void Release(int vehicleHandle)
        {
            try { if (api != null) release.Invoke(null, new object[] { vehicleHandle }); }
            catch { }
            carStyles.Remove(vehicleHandle);
        }
    }
}
