using System;
using System.Collections.Generic;
using System.Numerics;
using Navmesh.GroundGraph.Polyanya;

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

    // Cached PolyMesh derived from this graph's quads/portals/adjacency. Built
    // lazily on the first Pathfind call; invalidated whenever BuildAdjacency or
    // InitFlags runs (the mesh's face<->quad mapping would be stale). Valid as
    // long as Quads/Portals/Adjacency don't change.
    private PolyMesh? _cachedPolyMesh;

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

    public void BuildAdjacency(float maxClimb, float agentRadius = 0f)
    {
        MaxClimb = maxClimb;
        _cachedPolyMesh = null;
        for (int i = 0; i < Quads.Count; ++i)
            Adjacency[i].Clear();
        Portals.Clear();

        for (int a = 0; a < Quads.Count; ++a)
        {
            var qa = Quads[a];
            for (int b = a + 1; b < Quads.Count; ++b)
            {
                var qb = Quads[b];
                if (TryFindSharedEdge(qa, qb, maxClimb, agentRadius, out var spanMin, out var spanMax, out var yFrom, out var yTo))
                {
                    Adjacency[a].Add(b);
                    Adjacency[b].Add(a);
                    Portals.Add(new Portal(a, b, spanMin, spanMax, yFrom, yTo, false, Navmesh.AreaId.Default));
                }
            }
        }
    }

    private static bool TryFindSharedEdge(Quad a, Quad b, float maxClimb, float agentRadius, out Vector2 spanMin, out Vector2 spanMax, out float yFrom, out float yTo)
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
                (zMin, zMax) = Inset(zMin, zMax, agentRadius);
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
                (zMin, zMax) = Inset(zMin, zMax, agentRadius);
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
                (xMin, xMax) = Inset(xMin, xMax, agentRadius);
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
                (xMin, xMax) = Inset(xMin, xMax, agentRadius);
                spanMin = new(xMin, a.MinZ);
                spanMax = new(xMax, a.MinZ);
                return true;
            }
        }

        return false;
    }

    // shrink a portal edge interval inward by the agent radius on both ends so the
    // string-pulled path keeps its distance from the walls flanking the portal;
    // collapse to the midpoint if the gap is narrower than the agent (FFXIV has tight
    // doorways - keep them traversable rather than removing the portal entirely)
    private static (float lo, float hi) Inset(float lo, float hi, float r)
    {
        if (r <= 0f)
            return (lo, hi);
        if (hi - lo <= 2f * r)
        {
            var mid = (lo + hi) * 0.5f;
            return (mid, mid);
        }
        return (lo + r, hi - r);
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
        _cachedPolyMesh = null;
    }

    public List<Vector3> Pathfind(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, float range, System.Threading.CancellationToken cancel)
    {
        var fromQuad = NearestQuad(from, float.MaxValue, false);
        var toQuad = NearestQuad(to, float.MaxValue, false);
        var fromReachable = fromQuad >= 0 && fromQuad < Flags.Length && (Flags[fromQuad] & FLAG_UNREACHABLE) == 0;
        var toReachable = toQuad >= 0 && toQuad < Flags.Length && (Flags[toQuad] & FLAG_UNREACHABLE) == 0;
        Service.Log.Debug($"[pathfind] quad {fromQuad} -> {toQuad} (of {Quads.Count} quads, {Portals.Count} portals; fromReachable={fromReachable}, toReachable={toReachable})");
        if (fromQuad < 0 || toQuad < 0 || !fromReachable || !toReachable)
        {
            // TEMP-DEVLOG: remove with the rest of the TEMP-DEVLOG markers.
            Service.Telemetry?.Log($"ground EMPTY from={from:F1} to={to:F1} fromQuad={fromQuad} toQuad={toQuad} fromReach={fromReachable} toReach={toReachable}");
            return [];
        }

        // TEMP-DEVLOG: time the Polyanya search.
        var _devsw = System.Diagnostics.Stopwatch.StartNew();

        // Build (or reuse) the triangle PolyMesh derived from this graph. The
        // mesh is any-angle by construction, so `useStringPulling` is a no-op
        // (accepted for IPC compatibility; Polyanya already produces taut paths).
        // `useRaycast` is likewise accepted but ignored: any-angle search
        // supersedes raycast shortcutting. range semantics are forwarded to
        // PolyanyaSearch (range>0 terminates within range of goal; range==0
        // = exact goal).
        var mesh = _cachedPolyMesh ??= PolyMesh.FromQuadGraph(this);
        var search = new PolyanyaSearch(mesh);
        if (Flags.Length > 0)
            search.SetQuadFlags(Flags, FLAG_UNREACHABLE);
        var path = search.FindPath(from, to, fromQuad, toQuad, range, cancel);
        Service.Log.Debug($"[pathfind] polyanya returned {path.Count} waypoints");
        // TEMP-DEVLOG: remove with the rest of the TEMP-DEVLOG markers.
        _devsw.Stop();
        var _devlen = 0f;
        for (int i = 1; i < path.Count; ++i)
            _devlen += Vector3.Distance(path[i - 1], path[i]);
        var _devstraight = Vector3.Distance(from, to);
        Service.Telemetry?.Log($"ground OK from={from:F1} to={to:F1} quads={fromQuad}->{toQuad} mesh(faces={mesh.Faces.Count}) range={range:F1} wps={path.Count} len={_devlen:F1} straight={_devstraight:F1} ratio={(_devstraight > 0.01f ? _devlen / _devstraight : 1f):F2} {_devsw.Elapsed.TotalMilliseconds:F2}ms");
        return path;
    }
}

public readonly record struct Portal(int FromQuad, int ToQuad, Vector2 SpanMin, Vector2 SpanMax, float YFrom, float YTo, bool IsOffMesh, Navmesh.AreaId Area);