using System.Collections.Generic;
using System.Numerics;

namespace Navmesh.GroundGraph;

public static class FunnelStringPull
{
    public static List<Vector3> Pull(QuadGraph graph, List<(int quad, Vector3 p)> path, Vector3 fromPos, Vector3 toPos)
    {
        var result = new List<Vector3>();
        if (path.Count == 0)
        {
            result.Add(fromPos);
            result.Add(toPos);
            return result;
        }

        result.Add(fromPos);

        if (path.Count == 1)
        {
            result.Add(toPos);
            return result;
        }

        var portals = new List<(Vector3 left, Vector3 right)>();
        for (int i = 0; i < path.Count - 1; ++i)
        {
            var portal = FindPortal(graph, path[i].quad, path[i + 1].quad);
            if (portal.HasValue)
            {
                var (p1, p2) = portal.Value;
                var mid = (p1 + p2) * 0.5f;
                var dir = toPos - fromPos;
                var normal = new Vector3(-dir.Z, 0, dir.X);
                if (normal.LengthSquared() < 1e-6f)
                    normal = new Vector3(0, 0, 1);
                var proj1 = Vector3.Dot(p1 - mid, normal);
                var proj2 = Vector3.Dot(p2 - mid, normal);
                if (proj1 < proj2)
                {
                    portals.Add((p1, p2));
                }
                else
                {
                    portals.Add((p2, p1));
                }
            }
            else
            {
                var mid = path[i + 1].p;
                portals.Add((mid, mid));
            }
        }
        portals.Add((toPos, toPos));

        int apexIndex = 0;
        int leftIndex = 0;
        int rightIndex = 0;
        var apex = fromPos;
        var left = portals[0].left;
        var right = portals[0].right;

        for (int i = 1; i < portals.Count; ++i)
        {
            var (newLeft, newRight) = portals[i];

            if (TriArea2D(apex, right, newRight) <= 0)
            {
                if (apex == right || TriArea2D(apex, left, newRight) > 0)
                {
                    right = newRight;
                    rightIndex = i;
                }
                else
                {
                    apex = left;
                    apexIndex = leftIndex;
                    left = portals[apexIndex].left;
                    right = portals[apexIndex].right;
                    leftIndex = rightIndex = apexIndex;
                    i = apexIndex;
                    continue;
                }
            }

            if (TriArea2D(apex, left, newLeft) >= 0)
            {
                if (apex == left || TriArea2D(apex, right, newLeft) < 0)
                {
                    left = newLeft;
                    leftIndex = i;
                }
                else
                {
                    apex = right;
                    apexIndex = rightIndex;
                    left = portals[apexIndex].left;
                    right = portals[apexIndex].right;
                    leftIndex = rightIndex = apexIndex;
                    i = apexIndex;
                    continue;
                }
            }
        }

        if (apex != toPos)
            result.Add(apex);
        result.Add(toPos);

        return result;
    }

    private static (Vector3 p1, Vector3 p2)? FindPortal(QuadGraph graph, int fromQuad, int toQuad)
    {
        foreach (var p in graph.Portals)
        {
            if ((p.FromQuad == fromQuad && p.ToQuad == toQuad) || (p.FromQuad == toQuad && p.ToQuad == fromQuad))
            {
                var yMid = (p.YFrom + p.YTo) * 0.5f;
                var p1 = new Vector3(p.SpanMin.X, yMid, p.SpanMin.Y);
                var p2 = new Vector3(p.SpanMax.X, yMid, p.SpanMax.Y);
                return (p1, p2);
            }
        }
        return null;
    }

    private static float TriArea2D(Vector3 a, Vector3 b, Vector3 c)
    {
        var abx = b.X - a.X;
        var abz = b.Z - a.Z;
        var acx = c.X - a.X;
        var acz = c.Z - a.Z;
        return abx * acz - abz * acx;
    }
}