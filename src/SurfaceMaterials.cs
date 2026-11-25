// Surface material ids mapped onto the road test the route laws read.
namespace ARS
{

    static class SurfaceMaterials
    {
        public static bool IsRoad(int materialHash)
        {
            return ARS.Road.Contains(materialHash);
        }
    }
}
