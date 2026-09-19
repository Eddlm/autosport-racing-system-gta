using GTA;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace ARS
{
    public static unsafe class VehicleMemory
    {
        static ulong _steerOffset;
        static ulong _steerAngleOffset;
        static ulong _throttleOffset;
        static ulong _brakeOffset;
        static ulong _handlingOffset;
        static readonly Dictionary<string, IntPtr> _patterns = new Dictionary<string, IntPtr>();
        static bool _controlPatternReported;
        static bool _handlingPatternReported;

        public static void SetSteerInput(Vehicle vehicle, float value) => WriteInput(vehicle, value, ref _steerOffset, 0, "steer");
        public static void SetSteerAngle(Vehicle vehicle, float value) => WriteInput(vehicle, value, ref _steerAngleOffset, 8, "steer angle");
        public static void SetThrottle(Vehicle vehicle, float value) => WriteInput(vehicle, value, ref _throttleOffset, 0x10, "throttle");
        public static void SetBrakes(Vehicle vehicle, float value) => WriteInput(vehicle, value, ref _brakeOffset, 0x14, "brake");

        // What the last frame actually left in the control field; an unknown offset reads as uncut.
        public static float GetThrottle(Vehicle vehicle)
        {
            if (!ARS.CanWeUse(vehicle) || _throttleOffset == 0) return 1f;
            return *((float*)((ulong)vehicle.MemoryAddress + _throttleOffset));
        }

        public static float GetLateralTraction(Vehicle vehicle) => ReadHandlingFloat(vehicle, 0x0098);
        public static float GetSteerLock(Vehicle vehicle) => ReadHandlingFloat(vehicle, 0x0080);
        public static float GetDriveBiasFront(Vehicle vehicle) => ReadHandlingFloat(vehicle, 0x0048);
        public static float GetDownforce(Vehicle vehicle) => ReadHandlingFloat(vehicle, 0x0014);
        public static int GetHandlingFlags(Vehicle vehicle)
        {
            ulong address = GetHandlingAddress(vehicle);
            return address == 0 ? 0 : *(int*)(address + 0x128);
        }

        static void WriteInput(Vehicle vehicle, float value, ref ulong offset, int delta, string name)
        {
            if (!ARS.CanWeUse(vehicle)) return;
            if (offset == 0)
            {
                IntPtr pattern = (IntPtr)FindPattern("\x74\x0A\xF3\x0F\x11\xB3\x1C\x09\x00\x00\xEB\x25", "xxxxx?????xx");
                if (pattern == IntPtr.Zero)
                {
                    ReportMissingControlPattern();
                    return;
                }
                offset = *(uint*)(pattern + 6) + (ulong)delta;
                ARS.Log(ARS.LogImportance.Info, "[MEMORY] Learned the " + name + " offset: " + offset);
                return;
            }
            *((float*)((ulong)vehicle.MemoryAddress + offset)) = value;
        }

        // Once per session, forced: at Log Level None this is the only way a user learns why the cars never move.
        static void ReportMissingControlPattern()
        {
            if (_controlPatternReported) return;
            _controlPatternReported = true;
            ARS.Log(ARS.LogImportance.Error, "[MEMORY] The vehicle control pattern was not found on game build " + Game.Version + "; ARS cannot drive the cars.", true);
        }

        static void ReportMissingHandlingPattern()
        {
            if (_handlingPatternReported) return;
            _handlingPatternReported = true;
            ARS.Log(ARS.LogImportance.Error, "[MEMORY] The handling-pointer pattern was not found on game build " + Game.Version + "; grip, steering lock, downforce and handling flags all read as zero.", true);
        }

        static float ReadHandlingFloat(Vehicle vehicle, ulong offset)
        {
            ulong address = GetHandlingAddress(vehicle);
            return address == 0 ? 0f : *(float*)(address + offset);
        }

        static ulong GetHandlingAddress(Vehicle vehicle)
        {
            if (!ARS.CanWeUse(vehicle)) return 0;
            if (_handlingOffset == 0)
            {
                IntPtr pattern = (IntPtr)FindPattern("\x3C\x03\x0F\x85\x00\x00\x00\x00\x48\x8B\x41\x20\x48\x8B\x88", "xxxx????xxxxxxx");
                if (pattern == IntPtr.Zero)
                {
                    ReportMissingHandlingPattern();
                    return 0;
                }
                _handlingOffset = *(uint*)(pattern + 0x16);
            }
            return _handlingOffset == 0 ? 0 : *((ulong*)((ulong)vehicle.MemoryAddress + _handlingOffset));
        }

        // Memoized: a pattern miss used to buy a full module walk on every single call, from every
        // per-tick consumer (control writes, grip, steering lock, downforce, flags, wheel pointers).
        public static byte* FindPattern(string pattern, string mask)
        {
            string key = pattern + "|" + mask;
            IntPtr cached;
            if (_patterns.TryGetValue(key, out cached)) return (byte*)cached;
            byte* found = ScanModule(pattern, mask);
            _patterns[key] = (IntPtr)found;
            if (found == null) ARS.Log(ARS.LogImportance.Info, "[MEMORY] Pattern not found on this build: " + key);
            return found;
        }

        static byte* ScanModule(string pattern, string mask)
        {
            ProcessModule module = Process.GetCurrentProcess().MainModule;
            ulong address = (ulong)module.BaseAddress.ToInt64();
            ulong end = address + (ulong)module.ModuleMemorySize;
            for (; address < end; address++)
                for (int i = 0; i < pattern.Length; i++)
                    if (mask[i] == '?' || ((byte*)address)[i] == pattern[i])
                    {
                        if (i + 1 == pattern.Length) return (byte*)address;
                    }
                    else break;
            return null;
        }
    }
}
