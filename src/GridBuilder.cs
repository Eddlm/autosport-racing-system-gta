using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.Linq;

namespace ARS
{
    public static class GridBuilder
    {
        public static void Place(List<Racer> racers, List<Vector3> positions, List<Vector3> route, bool pointToPoint, bool sortByPower)
        {
            if (sortByPower) racers.Sort((a, b) => Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, a.Car).CompareTo(Function.Call<float>(Hash.GET_VEHICLE_ACCELERATION, b.Car)));
            Racer player = racers.FirstOrDefault(r => r.Driver.IsPlayer);
            if (player != null) { racers.Remove(player); racers.Add(player); }
            if (pointToPoint) positions.Reverse();
            for (int index = 0; index < racers.Count; index++)
            {
                Racer racer = racers[index];
                racer.Initialize();
                racer.Car.Position = positions[index];
                racer.Car.Heading = pointToPoint ? (route[2] - route[0]).ToHeading() : (index > positions.Count - 2 ? (positions[index - 2] - positions[index]).Normalized : (positions[index] - positions[index + 2]).Normalized).ToHeading();
            }
        }
    }
}
