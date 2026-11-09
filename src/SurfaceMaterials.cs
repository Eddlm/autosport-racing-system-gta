namespace ARS
{
    // The game's authored tyre grip for each material, in materials.dat order. The shape test's material
    // result IS this index - phMaterialMgrGta::GetMaterialId returns the entry's offset from the array start
    // (gtaMaterialManager.cpp:415) - so no name hashing is involved. Values come from the ars-fivem table,
    // whose layout the live manager confirms in SP Physics Tweaks: 213 entries, stride 112 bytes, grip first.
    // An index off the end reads 1.0, so an unknown material never counts as loose ground.
    static class SurfaceMaterials
    {
        static readonly float[] Grips =
        {
            1.00f, 1.00f, 0.90f, 1.00f, 1.00f, 0.90f, 0.90f, 0.92f,
            1.00f, 1.00f, 0.80f, 1.00f, 1.00f, 1.00f, 0.90f, 1.00f,
            1.00f, 1.00f, 0.75f, 0.80f, 0.75f, 0.90f, 0.90f, 0.75f,
            0.75f, 0.45f, 0.65f, 0.40f, 0.65f, 0.40f, 1.00f, 0.75f,
            0.85f, 0.75f, 0.75f, 0.90f, 0.90f, 0.90f, 0.70f, 0.70f,
            0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.90f,
            0.85f, 0.70f, 0.75f, 0.80f, 0.75f, 0.85f, 0.85f, 1.00f,
            1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f,
            1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 0.80f, 0.80f, 0.80f,
            0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f,
            0.80f, 0.90f, 0.90f, 0.80f, 0.90f, 0.80f, 0.80f, 0.85f,
            0.80f, 0.80f, 0.85f, 0.80f, 0.90f, 1.00f, 1.00f, 0.90f,
            0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f,
            0.85f, 0.80f, 0.80f, 0.80f, 1.00f, 1.00f, 0.90f, 0.90f,
            0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f,
            0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f, 0.90f,
            0.90f, 1.00f, 1.00f, 0.90f, 0.80f, 1.00f, 1.00f, 1.00f,
            1.00f, 0.80f, 0.80f, 1.00f, 0.50f, 0.50f, 1.00f, 0.70f,
            0.70f, 0.70f, 0.70f, 0.90f, 0.90f, 0.90f, 0.70f, 0.70f,
            0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f,
            0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f, 0.70f,
            0.70f, 0.70f, 0.70f, 0.70f, 0.90f, 0.90f, 1.00f, 1.00f,
            0.80f, 0.80f, 0.80f, 1.00f, 0.75f, 1.00f, 1.00f, 0.75f,
            0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f,
            0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f,
            0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f,
            0.75f, 0.75f, 0.75f, 0.75f, 0.75f,
        };

        public static bool TryGrip(int materialIndex, out float grip)
        {
            if (materialIndex >= 0 && materialIndex < Grips.Length)
            {
                grip = Grips[materialIndex];
                return true;
            }
            grip = 1f;
            return false;
        }

        public static float Grip(int materialIndex)
        {
            float grip;
            return TryGrip(materialIndex, out grip) ? grip : 1f;
        }
    }
}
