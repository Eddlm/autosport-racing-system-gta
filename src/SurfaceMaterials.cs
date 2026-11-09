namespace ARS
{
    // Road or not road, from the collision material hash a shape test returns. Only the identity is relied
    // on: a modded materials.dat can change every grip multiplier in the game, but it cannot turn grass into
    // tarmac. Anything whose hash is not on the road list is off road, and off road is where the outside
    // approach stops. The road list is grown from evidence: the load log prints every distinct hash it saw
    // and what it decided, so a road hash that is missing shows up as an off-road stretch on a paved track.
    static class SurfaceMaterials
    {
        public static bool IsRoad(int materialHash)
        {
            return ARS.Road.Contains(materialHash);
        }
    }
}
