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

                    // Check 4 edges of this cell.
                    CheckEdge(partition, chf, wallEdges, x, z, si, floorY, 1, 0, climbWorld);   // +X edge
                    CheckEdge(partition, chf, wallEdges, x, z, si, floorY, -1, 0, climbWorld);  // -X edge
                    CheckEdge(partition, chf, wallEdges, x, z, si, floorY, 0, 1, climbWorld);   // +Z edge
                    CheckEdge(partition, chf, wallEdges, x, z, si, floorY, 0, -1, climbWorld);  // -Z edge
                }
            }
        }

        // Simplify: remove collinear edges.
        var simplified = SimplifyEdges(wallEdges);
        return simplified;
    }

    private static void CheckEdge(LayerPartition partition, CompactHeightfield chf,
        List<(Vector2 a, Vector2 b)> wallEdges,
        int x, int z, int si, float floorY, int dx, int dz, float climbWorld)
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
                    // Neighbour within climb. Could be same layer (interior edge) or
                    // different layer (open seam). Not a wall.
                    isWall = false;
                    break;
                }
                // Over-climb neighbour: wall (ledge).
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
        // Build adjacency map: point → list of segment indices that start or end there.
        var remaining = new HashSet<int>(Enumerable.Range(0, edges.Count));
        var chains = new List<List<Vector2>>();

        while (remaining.Count > 0)
        {
            // Start a new chain from the first remaining segment.
            int startIdx = -1;
            foreach (var i in remaining) { startIdx = i; break; }
            remaining.Remove(startIdx);

            var chain = new List<Vector2> { edges[startIdx].a, edges[startIdx].b };

            // Try to extend the chain forward (matching chain tail to a segment's start).
            bool extended = true;
            while (extended)
            {
                extended = false;
                var tail = chain[^1];
                foreach (var i in remaining)
                {
                    var (ea, eb) = edges[i];
                    if (Vector2.DistanceSquared(ea, tail) < 1e-6f)
                    {
                        remaining.Remove(i);
                        chain.Add(eb);
                        extended = true;
                        break;
                    }
                    if (Vector2.DistanceSquared(eb, tail) < 1e-6f)
                    {
                        remaining.Remove(i);
                        chain.Add(ea);
                        extended = true;
                        break;
                    }
                }
            }

            // Try to extend the chain backward (matching chain head to a segment's end).
            extended = true;
            while (extended)
            {
                extended = false;
                var head = chain[0];
                foreach (var i in remaining)
                {
                    var (ea, eb) = edges[i];
                    if (Vector2.DistanceSquared(eb, head) < 1e-6f)
                    {
                        remaining.Remove(i);
                        chain.Insert(0, ea);
                        extended = true;
                        break;
                    }
                    if (Vector2.DistanceSquared(ea, head) < 1e-6f)
                    {
                        remaining.Remove(i);
                        chain.Insert(0, eb);
                        extended = true;
                        break;
                    }
                }
            }

            if (chain.Count >= 2)
                chains.Add(chain);
        }

        // Step 2: within each chain, remove collinear middle vertices.
        // Three consecutive points A, B, C are collinear if the cross product
        // (B-A) x (C-A) ≈ 0 (in 2D: (B.X-A.X)*(C.Y-A.Y) - (B.Y-A.Y)*(C.X-A.X)).
        var result = new List<List<Vector2>>(chains.Count);
        foreach (var chain in chains)
        {
            var simplified = new List<Vector2> { chain[0] };
            for (int i = 1; i < chain.Count - 1; i++)
            {
                var prev = simplified[^1];
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

                            float cx = chf.CellMinX(x) + (dx == 1 ? chf.CellSize : chf.CellSize * 0.5f);
                            float cz = chf.CellMinZ(z) + (dz == 1 ? chf.CellSize : chf.CellSize * 0.5f);
                            var posA = new Vector3(cx, floorA, cz);
                            var posB = new Vector3(cx, floorB, cz);
                            links.Add(new LayerLink(layA, layB, posA, posB));
                        }
                    }
                }
            }
        }

        return links;
    }
}
