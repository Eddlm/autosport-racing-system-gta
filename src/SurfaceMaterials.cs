using System.Collections.Generic;

namespace ARS
{
    // Tyre grip by the material hash a shape test returns, keyed off the four lists ARS already carries
    // (Road, Dirt, Sand, Other). The hash is the collision material's, not a materials.dat index and not
    // joaat of a material name: probed live on Offroad Hardcore v2, the apexes returned 282940568 on the
    // paved stretches and -1907520769 / -1942898710 over dirt. A hash in none of the lists reads 1.0, so an
    // unknown material is treated as road and never silently loses the outside approach.
    static class SurfaceMaterials
    {
        static readonly Dictionary<int, float> Grips = Build();

        static Dictionary<int, float> Build()
        {
            Dictionary<int, float> map = new Dictionary<int, float>();
            foreach (int hash in ARS.Road) map[hash] = 1f;
            foreach (int hash in ARS.Dirt) map[hash] = 0.75f;
            foreach (int hash in ARS.Sand) map[hash] = 0.70f;
            foreach (int hash in ARS.Other) map[hash] = 0.85f;
            return map;
        }

        public static bool TryGrip(int materialHash, out float grip)
        {
            return Grips.TryGetValue(materialHash, out grip);
        }

        public static float Grip(int materialHash)
        {
            float grip;
            return Grips.TryGetValue(materialHash, out grip) ? grip : 1f;
        }
    }
}
