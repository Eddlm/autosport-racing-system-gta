using GTA;
using System;
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

        public static void SetSteerInput(Vehicle vehicle, float value) => WriteInput(vehicle, value, ref _steerOffset, 0, "steer");
        public static void SetSteerAngle(Vehicle vehicle, float value) => WriteInput(vehicle, value, ref _steerAngleOffset, 8, "steer angle");
        public static void SetThrottle(Vehicle vehicle, float value) => WriteInput(vehicle, value, ref _throttleOffset, 0x10, "throttle");
        public static void SetBrakes(Vehicle vehicle, float value) => WriteInput(vehicle, value, ref _brakeOffset, 0x14, "brake");

        public static float GetLateralTraction(Vehicle vehicle) => ReadHandlingFloat(vehicle, 0x0098);
        public static float GetSteerLock(Vehicle vehicle) => ReadHandlingFloat(vehicle, 0x0080);
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
                if (pattern != IntPtr.Zero)
                {
                    offset = *(uint*)(pattern + 6) + (ulong)delta;
                    ARS.Log(ARS.LogImportance.Info, "[MEMORY] Learned the " + name + " offset: " + offset);
                }
                return;
            }
            *((float*)((ulong)vehicle.MemoryAddress + offset)) = value;
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
                if (pattern != IntPtr.Zero) _handlingOffset = *(uint*)(pattern + 0x16);
            }
            return _handlingOffset == 0 ? 0 : *((ulong*)((ulong)vehicle.MemoryAddress + _handlingOffset));
        }

        static byte* FindPattern(string pattern, string mask)
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
