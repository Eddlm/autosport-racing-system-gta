using GTA;
using GTA.Math;
using GTA.Native;
using System;
using System.Collections.Generic;
using System.Linq;

namespace ARS
{
    public enum GridSort
    {
        Power, PowerDescendent, TopSpeed, TopSpeedDescendent, Random
    }

    public static class GridBuilder
    {
        public static void Place(List<Racer> racers, List<Vector3> positions, List<Vector3> route, bool pointToPoint, GridSort sort)
        {
            SortRacers(racers, sort);
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

        static void SortRacers(List<Racer> racers, GridSort sort)
        {
            switch (sort)
            {
                case GridSort.Power: racers.Sort((a, b) => Accel(a).CompareTo(Accel(b))); break;
                case GridSort.PowerDescendent: racers.Sort((a, b) => Accel(b).CompareTo(Accel(a))); break;
                case GridSort.TopSpeed: racers.Sort((a, b) => TopSpeed(a).CompareTo(TopSpeed(b))); break;
                case GridSort.TopSpeedDescendent: racers.Sort((a, b) => TopSpeed(b).CompareTo(TopSpeed(a))); break;
                case GridSort.Random: Shuffle(racers); break;
            }
        }

        static float Accel(Racer r) => Function.Call<float>(Hash.GET_VEHICLE_MODEL_ACCELERATION, r.Car.Model.Hash);
        static float TopSpeed(Racer r) => Function.Call<float>((Hash)0xF417C2502FFFED43, r.Car.Model.Hash);

        static void Shuffle(List<Racer> racers)
        {
            Random rng = new Random();
            for (int i = racers.Count - 1; i > 0; i--)
            {
                int j = rng.Next(i + 1);
                Racer tmp = racers[i];
                racers[i] = racers[j];
                racers[j] = tmp;
            }
        }
    }
}
