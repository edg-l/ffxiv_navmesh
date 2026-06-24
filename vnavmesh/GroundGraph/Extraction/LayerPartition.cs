using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace Navmesh.GroundGraph.Extraction;

// Result of partitioning a CompactHeightfield into height-separated layers.
// Each cell (x, z, spanIndex) is assigned a layer id. Cells sharing a layer
// are 4-connected in XZ AND have floor-Y within walkableClimb of each other.
// Same-XZ cells whose floor-Y values differ by more than walkableClimb end up
// in distinct layers (overpass case).
public class LayerPartition
{
    // Layer assignment: cellLayer[z * width + x] = list of (floorY, layerId) per span.
    // layerId -1 means the span is non-walkable.
    public readonly int[,][] CellLayers; // [z, x] -> layer-id per span in column
    public readonly int NumLayers;
    public readonly int Width;
    public readonly int Height;
    public readonly CompactHeightfield Chf;

    private LayerPartition(CompactHeightfield chf, int[,][] cellLayers, int numLayers)
    {
        Chf = chf;
        CellLayers = cellLayers;
        NumLayers = numLayers;
        Width = chf.Width;
        Height = chf.Height;
    }

    // Assign height-region layers via 4-connected flood-fill. Each flood seed is
    // a walkable span not yet assigned a layer. BFS propagates to 4-connected
    // walkable neighbours whose floor-Y is within walkableClimb. When a neighbour's
    // floor-Y is more than walkableClimb away, it starts a new layer.
    public static LayerPartition Partition(CompactHeightfield chf)
    {
        int w = chf.Width;
        int h = chf.Height;
        float climbWorld = chf.WalkableClimbWorld;

        // cellLayer[z, x][spanIdx] = layer id for that span (-1 = unassigned).
        var cellLayers = new int[h, w][];
        for (int z = 0; z < h; z++)
            for (int x = 0; x < w; x++)
            {
                int n = chf.GetSpans(x, z).Count;
                var arr = new int[n];
                for (int i = 0; i < n; i++) arr[i] = -1;
                cellLayers[z, x] = arr;
            }

        int nextLayer = 0;
        var queue = new Queue<(int x, int z, int spanIdx)>();

        ReadOnlySpan<(int dx, int dz)> floodDirs = new (int, int)[]
        {
            (1, 0), (-1, 0), (0, 1), (0, -1)
        };

        for (int seedZ = 0; seedZ < h; seedZ++)
        {
            for (int seedX = 0; seedX < w; seedX++)
            {
                var seedSpans = chf.GetSpans(seedX, seedZ);
                for (int si = 0; si < seedSpans.Count; si++)
                {
                    if (seedSpans[si].Area == 0) continue;
                    if (cellLayers[seedZ, seedX][si] >= 0) continue;

                    // New layer flood-fill from this seed.
                    int layerId = nextLayer++;
                    cellLayers[seedZ, seedX][si] = layerId;
                    queue.Enqueue((seedX, seedZ, si));

                    while (queue.Count > 0)
                    {
                        var (cx, cz, csi) = queue.Dequeue();
                        float floorY = chf.GetSpans(cx, cz)[csi].FloorY;

                        // Check 4 neighbours.
                        foreach (var (dx, dz) in floodDirs)
                        {
                            int nx = cx + dx, nz = cz + dz;
                            if (nx < 0 || nx >= w || nz < 0 || nz >= h)
                                continue;
                            var nspans = chf.GetSpans(nx, nz);
                            for (int ni = 0; ni < nspans.Count; ni++)
                            {
                                if (nspans[ni].Area == 0) continue;
                                if (cellLayers[nz, nx][ni] >= 0) continue;
                                float diff = MathF.Abs(nspans[ni].FloorY - floorY);
                                if (diff <= climbWorld)
                                {
                                    // Same layer: within climb threshold.
                                    cellLayers[nz, nx][ni] = layerId;
                                    queue.Enqueue((nx, nz, ni));
                                }
                                // else: different layer (overpass) — will be seeded separately.
                            }
                        }
                    }
                }
            }
        }

        return new LayerPartition(chf, cellLayers, nextLayer);
    }

    // Get the layer id for a walkable span at (x, z, spanIndex). Returns -1 if
    // non-walkable or out of range.
    public int GetLayer(int x, int z, int spanIndex)
    {
        if (x < 0 || x >= Width || z < 0 || z >= Height) return -1;
        var arr = CellLayers[z, x];
        if (spanIndex < 0 || spanIndex >= arr.Length) return -1;
        return arr[spanIndex];
    }
}

// Contour extraction: traces the walkable/non-walkable boundary of each layer.
public static class ContourExtractor
{
    // Extract boundary contours for one layer. Returns a list of contour loops,
    // each as a list of 2D XZ positions. WALL edges (non-walkable or over-climb
    // neighbour) become contour segments. OPEN seams (different layer within climb)
    // are tracked separately for link extraction and do NOT appear in contours.
    public static List<List<Vector2>> ExtractContours(LayerPartition partition, int layerId)
    {
        var chf = partition.Chf;
        int w = chf.Width;
        int h = chf.Height;
        float climbWorld = chf.WalkableClimbWorld;

        // For each cell in this layer, collect WALL edges (boundary edges where
        // the neighbour is absent, non-walkable, or over-climb drop).
        // An edge is represented as (x1, z1, x2, z2) in XZ world coordinates.
        var wallEdges = new List<(Vector2 a, Vector2 b)>();

        for (int z = 0; z < h; z++)
        {
            for (int x = 0; x < w; x++)
            {
                var spans = chf.GetSpans(x, z);
                for (int si = 0; si < spans.Count; si++)
                {
                    if (partition.GetLayer(x, z, si) != layerId) continue;
                    float floorY = spans[si].FloorY;

                    // Check 4 edges of this cell. G1: pass layerId for layer-aware wall check.
                    CheckEdge(partition, chf, wallEdges, x, z, si, floorY, layerId, 1, 0, climbWorld);   // +X edge
                    CheckEdge(partition, chf, wallEdges, x, z, si, floorY, layerId, -1, 0, climbWorld);  // -X edge
                    CheckEdge(partition, chf, wallEdges, x, z, si, floorY, layerId, 0, 1, climbWorld);   // +Z edge
                    CheckEdge(partition, chf, wallEdges, x, z, si, floorY, layerId, 0, -1, climbWorld);  // -Z edge
                }
            }
        }

        // Simplify: remove collinear edges.
        var simplified = SimplifyEdges(wallEdges);
        return simplified;
    }

    // G1: layer-aware edge classification. An edge is a WALL for layerId unless:
    //   (a) a within-climb neighbour span in the SAME layer continues (interior), or
    //   (b) a within-climb neighbour span in a DIFFERENT layer exists (open seam).
    // A neighbour column that has NO within-climb span for layerId and only
    // out-of-climb or same-layer spans outside climb is a WALL.
    private static void CheckEdge(LayerPartition partition, CompactHeightfield chf,
        List<(Vector2 a, Vector2 b)> wallEdges,
        int x, int z, int si, float floorY, int layerId, int dx, int dz, float climbWorld)
    {
        int nx = x + dx, nz = z + dz;
        bool isWall = true;

        if (nx >= 0 && nx < chf.Width && nz >= 0 && nz < chf.Height)
        {
            var nspans = chf.GetSpans(nx, nz);
            for (int ni = 0; ni < nspans.Count; ni++)
            {
                if (nspans[ni].Area == 0) continue;
                float diff = MathF.Abs(nspans[ni].FloorY - floorY);
                if (diff <= climbWorld)
                {
                    int nLayer = partition.GetLayer(nx, nz, ni);
                    // Same layer continues (interior edge) or different layer (open seam):
                    // either way this is not a wall for layerId.
                    if (nLayer == layerId || nLayer >= 0)
                    {
                        isWall = false;
                        break;
                    }
                }
                // Over-climb or non-walkable neighbour: potential wall — keep checking.
            }
        }
        // Out of bounds: wall.

        if (!isWall) return;

        // Add the boundary edge of this cell on the side facing (dx,dz).
        // For +X edge: the edge goes from (MaxX, MinZ) to (MaxX, MaxZ).
        // For -X edge: from (MinX, MaxZ) to (MinX, MinZ).
        // For +Z edge: from (MaxX, MaxZ) to (MinX, MaxZ).
        // For -Z edge: from (MinX, MinZ) to (MaxX, MinZ).
        float minX = chf.CellMinX(x), maxX = chf.CellMaxX(x);
        float minZ = chf.CellMinZ(z), maxZ = chf.CellMaxZ(z);

        Vector2 a, b;
        if (dx == 1) { a = new(maxX, minZ); b = new(maxX, maxZ); }
        else if (dx == -1) { a = new(minX, maxZ); b = new(minX, minZ); }
        else if (dz == 1) { a = new(maxX, maxZ); b = new(minX, maxZ); }
        else { a = new(minX, minZ); b = new(maxX, minZ); }

        wallEdges.Add((a, b));
    }

    // Simplify: chain adjacent edge segments into polylines, then merge collinear
    // consecutive vertices within each chain. Returns a list of simplified polylines;
    // Phase 4 CDT assembles these into proper polygon loops.
    private static List<List<Vector2>> SimplifyEdges(List<(Vector2 a, Vector2 b)> edges)
    {
        if (edges.Count == 0)
            return new List<List<Vector2>>();

        // Step 1: chain segments by matching endpoints into polylines.
        // G2: build a point→segment-index map for O(1) next/prev lookups.
        // Each segment (ea, eb) contributes to buckets keyed by ea and eb.
        // We use a simple Dictionary<Vector2, List<int>> for the endpoints.
        var byA = new Dictionary<Vector2, List<int>>();
        var byB = new Dictionary<Vector2, List<int>>();
        for (int i = 0; i < edges.Count; i++)
        {
            var (ea, eb) = edges[i];
            if (!byA.TryGetValue(ea, out var la)) byA[ea] = la = new List<int>();
            la.Add(i);
            if (!byB.TryGetValue(eb, out var lb)) byB[eb] = lb = new List<int>();
            lb.Add(i);
        }

        var remaining = new HashSet<int>(Enumerable.Range(0, edges.Count));
        var chains = new List<List<Vector2>>();

        while (remaining.Count > 0)
        {
            // Start a new chain from the first remaining segment.
            int startIdx = -1;
            foreach (var i in remaining) { startIdx = i; break; }
            RemoveFromIndex(byA, edges[startIdx].a, startIdx);
            RemoveFromIndex(byB, edges[startIdx].b, startIdx);
            remaining.Remove(startIdx);

            // G3: accumulate backward extension in a separate list, prepend once.
            var forward = new List<Vector2> { edges[startIdx].a, edges[startIdx].b };
            var backward = new List<Vector2>(); // points to prepend (in reverse order)

            // Extend forward via byA[tail] or byB[tail].
            bool extended = true;
            while (extended)
            {
                extended = false;
                var tail = forward[^1];
                int found = -1;
                bool flipFound = false;
                if (byA.TryGetValue(tail, out var matchA))
                {
                    foreach (int i in matchA) { if (remaining.Contains(i)) { found = i; flipFound = false; break; } }
                }
                if (found < 0 && byB.TryGetValue(tail, out var matchB))
                {
                    foreach (int i in matchB) { if (remaining.Contains(i)) { found = i; flipFound = true; break; } }
                }
                if (found >= 0)
                {
                    RemoveFromIndex(byA, edges[found].a, found);
                    RemoveFromIndex(byB, edges[found].b, found);
                    remaining.Remove(found);
                    forward.Add(flipFound ? edges[found].a : edges[found].b);
                    extended = true;
                }
            }

            // Extend backward via byB[head] or byA[head].
            extended = true;
            while (extended)
            {
                extended = false;
                var head = forward[0];
                // Also consider already-prepended backward points:
                if (backward.Count > 0) head = backward[^1];
                else head = forward[0];
                int found = -1;
                bool flipFound = false;
                if (byB.TryGetValue(head, out var matchB))
                {
                    foreach (int i in matchB) { if (remaining.Contains(i)) { found = i; flipFound = false; break; } }
                }
                if (found < 0 && byA.TryGetValue(head, out var matchA))
                {
                    foreach (int i in matchA) { if (remaining.Contains(i)) { found = i; flipFound = true; break; } }
                }
                if (found >= 0)
                {
                    RemoveFromIndex(byA, edges[found].a, found);
                    RemoveFromIndex(byB, edges[found].b, found);
                    remaining.Remove(found);
                    // G3: accumulate in backward list (we'll prepend all at once).
                    backward.Add(flipFound ? edges[found].b : edges[found].a);
                    extended = true;
                }
            }

            // G3: prepend all backward points at once.
            List<Vector2> chain;
            if (backward.Count > 0)
            {
                backward.Reverse();
                chain = new List<Vector2>(backward.Count + forward.Count);
                chain.AddRange(backward);
                chain.AddRange(forward);
            }
            else
            {
                chain = forward;
            }

            if (chain.Count >= 2)
                chains.Add(chain);
        }

        // Step 2: within each chain, remove collinear middle vertices.
        // G4: use chain[i-1] (original neighbour) for the collinearity test, not
        // simplified[^1] (last kept point), to avoid corner errors on non-axis-aligned contours.
        var result = new List<List<Vector2>>(chains.Count);
        foreach (var chain in chains)
        {
            var simplified = new List<Vector2> { chain[0] };
            for (int i = 1; i < chain.Count - 1; i++)
            {
                var prev = chain[i - 1]; // G4: original neighbour, not simplified[^1]
                var curr = chain[i];
                var next = chain[i + 1];
                float cross = (curr.X - prev.X) * (next.Y - prev.Y)
                            - (curr.Y - prev.Y) * (next.X - prev.X);
                if (MathF.Abs(cross) > 1e-6f)
                    simplified.Add(curr);
                // else: collinear — skip this vertex
            }
            simplified.Add(chain[^1]);
            if (simplified.Count >= 2)
                result.Add(simplified);
        }

        return result;
    }

    private static void RemoveFromIndex(Dictionary<Vector2, List<int>> index, Vector2 key, int idx)
    {
        if (index.TryGetValue(key, out var list))
            list.Remove(idx);
    }
}

// Inter-layer link extraction: finds within-climb seam segments between distinct
// layers. Each seam becomes an off-mesh link stub (OffMeshLink with world positions).
public static class LinkExtractor
{
    public readonly record struct LayerLink(
        int LayerA, int LayerB,
        Vector3 PosA, Vector3 PosB);

    // Extract all within-climb seam links between distinct layers.
    public static List<LayerLink> ExtractInterLayerLinks(LayerPartition partition)
    {
        var chf = partition.Chf;
        int w = chf.Width;
        int h = chf.Height;
        float climbWorld = partition.Chf.WalkableClimbWorld;
        var links = new List<LayerLink>();
        var seenPairs = new HashSet<(int, int, int, int, int, int)>(); // dedup (layA,layB,x,z,nx,nz)
        ReadOnlySpan<(int dx, int dz)> linkDirs = new (int, int)[] { (1, 0), (0, 1) };

        for (int z = 0; z < h; z++)
        {
            for (int x = 0; x < w; x++)
            {
                var spans = chf.GetSpans(x, z);
                for (int si = 0; si < spans.Count; si++)
                {
                    int layA = partition.GetLayer(x, z, si);
                    if (layA < 0) continue;
                    float floorA = spans[si].FloorY;

                    // Check +X and +Z neighbours only (to avoid duplicate links).
                    foreach (var (dx, dz) in linkDirs)
                    {
                        int nx = x + dx, nz = z + dz;
                        if (nx >= w || nz >= h) continue;
                        var nspans = chf.GetSpans(nx, nz);
                        for (int ni = 0; ni < nspans.Count; ni++)
                        {
                            int layB = partition.GetLayer(nx, nz, ni);
                            if (layB < 0) continue;
                            if (layA == layB) continue;
                            float floorB = nspans[ni].FloorY;
                            float diff = MathF.Abs(floorA - floorB);
                            if (diff > climbWorld) continue; // over-climb: wall, not link

                            // Within-climb seam between different layers: emit link.
                            int la = Math.Min(layA, layB), lb = Math.Max(layA, layB);
                            var key = (la, lb, Math.Min(x, nx), Math.Min(z, nz), Math.Max(x, nx), Math.Max(z, nz));
                            if (!seenPairs.Add(key)) continue;

                            // G5: posA at center of cell (x,z), posB at center of
                            // neighbour cell (nx,nz), so the link crosses the boundary.
                            var posA = chf.CellCenter(x, z, floorA);
                            var posB = chf.CellCenter(nx, nz, floorB);
                            links.Add(new LayerLink(layA, layB, posA, posB));
                        }
                    }
                }
            }
        }

        return links;
    }
}
