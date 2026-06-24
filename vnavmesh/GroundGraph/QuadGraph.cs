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

    // C3: lazily-built uniform-grid spatial index for NearestQuad. Cell size ~4
    // yalms gives a good balance for FFXIV zones (quads are typically 0.25–8 y wide).
    private const float SpatialGridCell = 4f;
    private List<int>[]? _spatialGrid;
    private int _sgCellsX, _sgCellsZ;
    private float _sgOriginX, _sgOriginZ;

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
        _spatialGrid = null; // C3: invalidate spatial index when quads/portals change

        // C1: snapshot existing off-mesh portals BEFORE clearing, so inter-layer
        // portals added by QuadMesher.MeshInto via AddOffMesh are preserved.
        var savedOffMesh = new List<Portal>();
        foreach (var p in Portals)
            if (p.IsOffMesh)
                savedOffMesh.Add(p);

        for (int i = 0; i < Quads.Count; ++i)
            Adjacency[i].Clear();
        Portals.Clear();

        // Two axis-aligned quads share an edge only if one's forward edge sits on
        // the same line as the other's backward edge AND their extents overlap on
        // the perpendicular axis. Bucket edges by quantized coordinate so only
        // quads on the same edge line are compared, then sweep the overlap axis.
        // (Was O(n^2) all-pairs: ~6.7B comparisons on a 116k-quad mesh.)
        int n = Quads.Count;
        var rightX = new Dictionary<int, List<int>>();  // quads by MaxX (right edge)
        var leftX = new Dictionary<int, List<int>>();   // quads by MinX (left edge)
        var topZ = new Dictionary<int, List<int>>();    // quads by MaxZ (far edge)
        var botZ = new Dictionary<int, List<int>>();    // quads by MinZ (near edge)
        for (int i = 0; i < n; ++i)
        {
            var q = Quads[i];
            BucketAdd(rightX, EdgeKey(q.MaxX), i);
            BucketAdd(leftX, EdgeKey(q.MinX), i);
            BucketAdd(topZ, EdgeKey(q.MaxZ), i);
            BucketAdd(botZ, EdgeKey(q.MinZ), i);
        }
        // Vertical shared edges (a.MaxX == b.MinX): overlap on Z.
        foreach (var (k, r) in rightX)
            if (leftX.TryGetValue(k, out var l))
                ConnectOverlapping(r, l, vertical: true, maxClimb, agentRadius);
        // Horizontal shared edges (a.MaxZ == b.MinZ): overlap on X.
        foreach (var (k, t) in topZ)
            if (botZ.TryGetValue(k, out var b))
                ConnectOverlapping(t, b, vertical: false, maxClimb, agentRadius);

        // C1: restore off-mesh portals and their adjacency entries.
        foreach (var p in savedOffMesh)
        {
            Portals.Add(p);
            if (p.FromQuad >= 0 && p.FromQuad < Adjacency.Count)
                Adjacency[p.FromQuad].Add(p.ToQuad);
            if (p.ToQuad >= 0 && p.ToQuad < Adjacency.Count)
                Adjacency[p.ToQuad].Add(p.FromQuad);
        }
    }

    // Quantize an edge coordinate to a bucket key. Greedy-mesh quad edges are
    // multiples of CellSize (0.25); a 0.05 quantum collides only truly-equal
    // edges (5 buckets apart per cell) while absorbing float error.
    private static int EdgeKey(float v) => (int)MathF.Round(v * 20f);

    private static void BucketAdd(Dictionary<int, List<int>> d, int key, int idx)
    {
        if (!d.TryGetValue(key, out var list))
            d[key] = list = new();
        list.Add(idx);
    }

    // setA = quads whose forward edge lies on this line; setB = quads whose
    // backward edge lies on it. Sweep the perpendicular axis to find overlapping
    // (a, b) pairs, validating each via TryFindSharedEdge. Greedy quads on one
    // side tile the axis disjointly, so the active sets stay tiny ⇒ near-linear.
    private void ConnectOverlapping(List<int> setA, List<int> setB, bool vertical, float maxClimb, float agentRadius)
    {
        var events = new List<(float coord, int kind, int idx)>((setA.Count + setB.Count) * 2);
        foreach (var a in setA)
        {
            var q = Quads[a];
            float lo = vertical ? q.MinZ : q.MinX, hi = vertical ? q.MaxZ : q.MaxX;
            events.Add((lo, 0, a));
            events.Add((hi, 2, a));
        }
        foreach (var b in setB)
        {
            var q = Quads[b];
            float lo = vertical ? q.MinZ : q.MinX, hi = vertical ? q.MaxZ : q.MaxX;
            events.Add((lo, 1, b));
            events.Add((hi, 3, b));
        }
        // Starts (0,1) before ends (2,3) at equal coord; TryFindSharedEdge filters
        // zero-overlap (touching) pairs via its strict lo<hi check.
        events.Sort((u, v) => u.coord != v.coord ? u.coord.CompareTo(v.coord) : u.kind.CompareTo(v.kind));
        var activeA = new HashSet<int>(); // C2: HashSet avoids O(N) Remove-by-value
        var activeB = new HashSet<int>();
        foreach (var (_, kind, idx) in events)
        {
            switch (kind)
            {
                case 0:
                    foreach (var b in activeB) TryConnect(idx, b, maxClimb, agentRadius);
                    activeA.Add(idx);
                    break;
                case 1:
                    foreach (var a in activeA) TryConnect(a, idx, maxClimb, agentRadius);
                    activeB.Add(idx);
                    break;
                case 2: activeA.Remove(idx); break;
                case 3: activeB.Remove(idx); break;
            }
        }
    }

    private void TryConnect(int a, int b, float maxClimb, float agentRadius)
    {
        if (a == b)
            return;
        if (TryFindSharedEdge(Quads[a], Quads[b], maxClimb, agentRadius, out var spanMin, out var spanMax, out var yFrom, out var yTo))
        {
            Adjacency[a].Add(b);
            Adjacency[b].Add(a);
            Portals.Add(new Portal(a, b, spanMin, spanMax, yFrom, yTo, false, Navmesh.AreaId.Default));
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
        // C3: use spatial grid for fast lookup when available and no strict maxDist.
        if (_spatialGrid != null && maxDist >= float.MaxValue * 0.5f && allowUnreachable)
        {
            int result = NearestQuadSpatial(p);
            if (result >= 0)
                return result;
        }

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

    // C3: build the spatial grid lazily the first time it is needed.
    private void EnsureSpatialGrid()
    {
        if (_spatialGrid != null)
            return;
        float spanX = BoundsMax.X - BoundsMin.X;
        float spanZ = BoundsMax.Z - BoundsMin.Z;
        _sgCellsX = Math.Max(1, (int)MathF.Ceiling(spanX / SpatialGridCell));
        _sgCellsZ = Math.Max(1, (int)MathF.Ceiling(spanZ / SpatialGridCell));
        _sgOriginX = BoundsMin.X;
        _sgOriginZ = BoundsMin.Z;
        _spatialGrid = new List<int>[_sgCellsX * _sgCellsZ];
        for (int i = 0; i < _spatialGrid.Length; i++)
            _spatialGrid[i] = new List<int>();
        for (int qi = 0; qi < Quads.Count; qi++)
        {
            var q = Quads[qi];
            int x0 = GridCell(q.MinX - _sgOriginX, _sgCellsX);
            int x1 = GridCell(q.MaxX - _sgOriginX, _sgCellsX);
            int z0 = GridCell(q.MinZ - _sgOriginZ, _sgCellsZ);
            int z1 = GridCell(q.MaxZ - _sgOriginZ, _sgCellsZ);
            for (int gz = z0; gz <= z1; gz++)
                for (int gx = x0; gx <= x1; gx++)
                    _spatialGrid[gz * _sgCellsX + gx].Add(qi);
        }
    }

    private int GridCell(float v, int maxCell)
        => Math.Clamp((int)(v / SpatialGridCell), 0, maxCell - 1);

    // C3: probe the grid cell at p and a small neighbourhood; return nearest XZ-
    // containing quad (nearest Y) or nearest-center quad if none contains.
    private int NearestQuadSpatial(Vector3 p)
    {
        EnsureSpatialGrid();
        int gx = GridCell(p.X - _sgOriginX, _sgCellsX);
        int gz = GridCell(p.Z - _sgOriginZ, _sgCellsZ);

        int best = -1;
        float bestDist = float.MaxValue;
        const int Radius = 2;
        for (int dz = -Radius; dz <= Radius; dz++)
        {
            for (int dx = -Radius; dx <= Radius; dx++)
            {
                int cx = gx + dx, cz = gz + dz;
                if (cx < 0 || cx >= _sgCellsX || cz < 0 || cz >= _sgCellsZ)
                    continue;
                var cell = _spatialGrid![cz * _sgCellsX + cx];
                foreach (int qi in cell)
                {
                    var q = Quads[qi];
                    if (q.ContainsXZ(p))
                    {
                        float dy = MathF.Abs(p.Y - q.MinY);
                        if (dy < bestDist) { bestDist = dy; best = qi; }
                    }
                }
            }
        }
        if (best >= 0)
            return best;

        // No XZ-containing quad in neighbourhood: fall back to nearest-center
        // in a wider search.
        bestDist = float.MaxValue;
        for (int dz = -Radius; dz <= Radius; dz++)
        {
            for (int dx = -Radius; dx <= Radius; dx++)
            {
                int cx = gx + dx, cz = gz + dz;
                if (cx < 0 || cx >= _sgCellsX || cz < 0 || cz >= _sgCellsZ)
                    continue;
                var cell = _spatialGrid![cz * _sgCellsX + cx];
                foreach (int qi in cell)
                {
                    var q = Quads[qi];
                    float d = (q.Center - p).LengthSquared();
                    if (d < bestDist) { bestDist = d; best = qi; }
                }
            }
        }
        // If still -1 (point outside grid entirely), signal fallback to full scan.
        return best;
    }

    public HashSet<int> FloodReachable(IEnumerable<int> seeds)
    {
        // C4: pre-index off-mesh portals by FromQuad to avoid O(nodes*portals) scan.
        var offMeshByFrom = new Dictionary<int, List<int>>();
        foreach (var portal in Portals)
        {
            if (!portal.IsOffMesh) continue;
            if (!offMeshByFrom.TryGetValue(portal.FromQuad, out var list))
                offMeshByFrom[portal.FromQuad] = list = new List<int>();
            list.Add(portal.ToQuad);
        }

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
            if (offMeshByFrom.TryGetValue(cur, out var toQuads))
            {
                foreach (var toQuad in toQuads)
                {
                    if (result.Add(toQuad))
                        queue.Enqueue(toQuad);
                }
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
        _spatialGrid = null; // C3: invalidate spatial index
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