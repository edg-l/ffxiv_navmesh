using System.Collections.Generic;
using System.Numerics;

namespace Navmesh.Movement;

public static class SplineSmoothing
{
    public static List<Vector3> CatmullRom(List<Vector3> pts, int segmentsPerSpan = 8)
    {
        var result = new List<Vector3>();
        if (pts.Count < 2)
        {
            result.AddRange(pts);
            return result;
        }

        var extended = new List<Vector3>(pts.Count + 2) { pts[0] };
        extended.AddRange(pts);
        extended.Add(pts[^1]);

        for (int i = 1; i < extended.Count - 2; ++i)
        {
            var p0 = extended[i - 1];
            var p1 = extended[i];
            var p2 = extended[i + 1];
            var p3 = extended[i + 2];

            if (i == 1)
                result.Add(p1);

            for (int s = 1; s <= segmentsPerSpan; ++s)
            {
                float t = s / (float)segmentsPerSpan;
                float t2 = t * t;
                float t3 = t2 * t;
                var point = 0.5f * (
                    (2f * p1) +
                    (-p0 + p2) * t +
                    (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
                    (-p0 + 3f * p1 - 3f * p2 + p3) * t3
                );
                result.Add(point);
            }
        }

        return result;
    }
}