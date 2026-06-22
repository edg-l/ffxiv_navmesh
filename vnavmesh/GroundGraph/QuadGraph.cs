using System;
using System.Collections.Generic;
using System.Numerics;

namespace Navmesh.GroundGraph;

public class QuadGraph
{
    public const int FLAG_UNREACHABLE = 0x10;

    public List<Quad> Quads = [];
    public List<List<int>> Adjacency = [];
    public List<Portal> Portals = [];
    public int[] Flags = [];
    public Vector3 BoundsMin;
    public Vector3 BoundsMax;
    public float MaxClimb;

    public int Count => Quads.Count;

    public QuadGraph(Vector3 boundsMin, Vector3 boundsMax)
    {
        BoundsMin = boundsMin;
        BoundsMax = boundsMax;
    }

    public int AddQuad(Quad q)
    {
        var index = Quads.Count;
        Quads.Add(q);
        Adjacency.Add([]);
        return index;
    }

    public void SetArea(int quadId, Navmesh.AreaId area)
    {
        var q = Quads[quadId];
        Quads[quadId] = q with { Area = q.Area | area };
    }

    public void MarkAreaBox(Vector3 min, Vector3 max, Navmesh.AreaId area)
    {
        for (int i = 0; i < Quads.Count; ++i)
        {
            var q = Quads[i];
            if (q.MaxX >= min.X && q.MinX <= max.X && q.MaxZ >= min.Z && q.MinZ <= max.Z && q.MinY >= min.Y && q.MinY <= max.Y)
                SetArea(i, area);
        }
    }

    public int AddOffMesh(Vector3 a, Vector3 b, Navmesh.AreaId area, float radius = 0.5f, bool bidirectional = false, int userId = 0)
    {
        var quadA = NearestQuad(a, radius + 5f);
        var quadB = NearestQuad(b, radius + 5f);
        if (quadA < 0 || quadB < 0)
            return -1;

        var qa = Quads[quadA];
        var qb = Quads[quadB];
        var spanMinA = new Vector2(Math.Clamp(a.X, qa.MinX, qa.MaxX), Math.Clamp(a.Z, qa.MinZ, qa.MaxZ));
        var spanMaxA = spanMinA;
        var spanMinB = new Vector2(Math.Clamp(b.X, qb.MinX, qb.MaxX), Math.Clamp(b.Z, qb.MinZ, qb.MaxZ));
        var spanMaxB = spanMinB;

        Portals.Add(new Portal(quadA, quadB, spanMinA, spanMaxA, a.Y, b.Y, true, area | Navmesh.AreaId.Endpoint));
        Adjacency[quadA].Add(quadB);
        if (bidirectional)
        {
            Portals.Add(new Portal(quadB, quadA, spanMinB, spanMaxB, b.Y, a.Y, true, area | Navmesh.AreaId.Endpoint));
            Adjacency[quadB].Add(quadA);
        }
        return quadA;
    }

    public void BuildAdjacency(float maxClimb)
    {
        MaxClimb = maxClimb;
        for (int i = 0; i < Quads.Count; ++i)
            Adjacency[i].Clear();
        Portals.Clear();

        for (int a = 0; a < Quads.Count; ++a)
        {
            var qa = Quads[a];
            for (int b = a + 1; b < Quads.Count; ++b)
            {
                var qb = Quads[b];
                if (TryFindSharedEdge(qa, qb, maxClimb, out var spanMin, out var spanMax, out var yFrom, out var yTo))
                {
                    Adjacency[a].Add(b);
                    Adjacency[b].Add(a);
                    Portals.Add(new Portal(a, b, spanMin, spanMax, yFrom, yTo, false, Navmesh.AreaId.Default));
                }
            }
        }
    }

    private static bool TryFindSharedEdge(Quad a, Quad b, float maxClimb, out Vector2 spanMin, out Vector2 spanMax, out float yFrom, out float yTo)
    {
        spanMin = default;
        spanMax = default;
        yFrom = a.MinY;
        yTo = b.MinY;

        if (MathF.Abs(a.MinY - b.MinY) > maxClimb)
            return false;

        if (MathF.Abs(a.MaxX - b.MinX) < 0.001f)
        {
            var zMin = MathF.Max(a.MinZ, b.MinZ);
            var zMax = MathF.Min(a.MaxZ, b.MaxZ);
            if (zMin < zMax)
            {
                spanMin = new(a.MaxX, zMin);
                spanMax = new(a.MaxX, zMax);
                return true;
            }
        }
        else if (MathF.Abs(a.MinX - b.MaxX) < 0.001f)
        {
            var zMin = MathF.Max(a.MinZ, b.MinZ);
            var zMax = MathF.Min(a.MaxZ, b.MaxZ);
            if (zMin < zMax)
            {
                spanMin = new(a.MinX, zMin);
                spanMax = new(a.MinX, zMax);
                return true;
            }
        }
        else if (MathF.Abs(a.MaxZ - b.MinZ) < 0.001f)
        {
            var xMin = MathF.Max(a.MinX, b.MinX);
            var xMax = MathF.Min(a.MaxX, b.MaxX);
            if (xMin < xMax)
            {
                spanMin = new(xMin, a.MaxZ);
                spanMax = new(xMax, a.MaxZ);
                return true;
            }
        }
        else if (MathF.Abs(a.MinZ - b.MaxZ) < 0.001f)
        {
            var xMin = MathF.Max(a.MinX, b.MinX);
            var xMax = MathF.Min(a.MaxX, b.MaxX);
            if (xMin < xMax)
            {
                spanMin = new(xMin, a.MinZ);
                spanMax = new(xMax, a.MinZ);
                return true;
            }
        }

        return false;
    }

    public int NearestQuad(Vector3 p, float maxDist = float.MaxValue, bool allowUnreachable = true)
    {
        int best = -1;
        float bestDist = maxDist;
        for (int i = 0; i < Quads.Count; ++i)
        {
            if (!allowUnreachable && i < Flags.Length && (Flags[i] & FLAG_UNREACHABLE) != 0)
                continue;
            var q = Quads[i];
            if (q.ContainsXZ(p))
            {
                var dy = MathF.Abs(p.Y - q.MinY);
                if (dy < bestDist)
                {
                    bestDist = dy;
                    best = i;
                }
            }
        }
        if (best >= 0)
            return best;

        for (int i = 0; i < Quads.Count; ++i)
        {
            if (!allowUnreachable && i < Flags.Length && (Flags[i] & FLAG_UNREACHABLE) != 0)
                continue;
            var q = Quads[i];
            var center = q.Center;
            var d = (center - p).LengthSquared();
            if (d < bestDist)
            {
                bestDist = d;
                best = i;
            }
        }
        return best;
    }

    public HashSet<int> FloodReachable(IEnumerable<int> seeds)
    {
        var result = new HashSet<int>();
        var queue = new Queue<int>();
        foreach (var s in seeds)
        {
            if (s >= 0 && !result.Contains(s))
            {
                result.Add(s);
                queue.Enqueue(s);
            }
        }
        while (queue.Count > 0)
        {
            var cur = queue.Dequeue();
            foreach (var neighbor in Adjacency[cur])
            {
                if (result.Add(neighbor))
                    queue.Enqueue(neighbor);
            }
            foreach (var portal in Portals)
            {
                if (portal.IsOffMesh && portal.FromQuad == cur && result.Add(portal.ToQuad))
                    queue.Enqueue(portal.ToQuad);
            }
        }
        return result;
    }

    public void ApplyReachable(HashSet<int> reachable)
    {
        if (Flags.Length < Quads.Count)
            Flags = new int[Quads.Count];
        for (int i = 0; i < Quads.Count; ++i)
        {
            if (reachable.Contains(i))
                Flags[i] = 0;
            else
                Flags[i] = FLAG_UNREACHABLE;
        }
    }

    public void InitFlags()
    {
        Flags = new int[Quads.Count];
    }

    public List<Vector3> Pathfind(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, float range, System.Threading.CancellationToken cancel)
    {
        var fromQuad = NearestQuad(from, float.MaxValue, false);
        var toQuad = NearestQuad(to, float.MaxValue, false);
        var fromReachable = fromQuad >= 0 && fromQuad < Flags.Length && (Flags[fromQuad] & FLAG_UNREACHABLE) == 0;
        var toReachable = toQuad >= 0 && toQuad < Flags.Length && (Flags[toQuad] & FLAG_UNREACHABLE) == 0;
        Service.Log.Debug($"[pathfind] quad {fromQuad} -> {toQuad} (of {Quads.Count} quads, {Portals.Count} portals; fromReachable={fromReachable}, toReachable={toReachable})");
        if (fromQuad < 0 || toQuad < 0)
            return [];

        var pathfinder = new QuadPathfind(this);
        var path = range > 0
            ? pathfinder.FindPathWithRange(fromQuad, toQuad, from, to, range, useRaycast, cancel)
            : pathfinder.FindPath(fromQuad, toQuad, from, to, useRaycast, cancel);

        Service.Log.Debug($"[pathfind] A* returned {path.Count} nodes");
        if (path.Count == 0)
            return [];

        if (useStringPulling)
        {
            var simplified = FunnelStringPull.Pull(this, path, from, to);
            Service.Log.Debug($"[pathfind] funnel returned {simplified.Count} waypoints");
            return simplified;
        }
        else
        {
            var res = new List<Vector3> { from };
            foreach (var (_, p) in path)
                res.Add(p);
            res.Add(to);
            return res;
        }
    }
}

public readonly record struct Portal(int FromQuad, int ToQuad, Vector2 SpanMin, Vector2 SpanMax, float YFrom, float YTo, bool IsOffMesh, Navmesh.AreaId Area);