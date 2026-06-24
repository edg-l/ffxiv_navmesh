using Navmesh.GroundGraph.Extraction;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace Navmesh.GroundGraph;

public readonly record struct Quad(float MinX, float MinY, float MinZ, float MaxX, float MaxY, float MaxZ, Navmesh.AreaId Area)
{
    public Vector3 Center => new((MinX + MaxX) * 0.5f, (MinY + MaxY) * 0.5f, (MinZ + MaxZ) * 0.5f);
    public Vector2 MinXZ => new(MinX, MinZ);
    public Vector2 MaxXZ => new(MaxX, MaxZ);

    public bool ContainsXZ(Vector3 p) => p.X >= MinX && p.X <= MaxX && p.Z >= MinZ && p.Z <= MaxZ;
}

public static class QuadMesher
{
    private const float SurfaceYEps = 0.05f;

    // Build a QuadGraph from a merged CompactHeightfield (fine 0.25y data).
    // Each layer is meshed independently; within-climb seam boundaries between
    // distinct layers become off-mesh links in the graph.
    public static QuadGraph GreedyMesh(CompactHeightfield chf)
    {
        // World bounds of the CHF grid.
        var boundsMin = chf.BoundsMin;
        var boundsMax = new Vector3(
            boundsMin.X + chf.Width * chf.CellSize,
            boundsMin.Y + 2048,  // Y bounds are for search; use full range
            boundsMin.Z + chf.Height * chf.CellSize);
        var graph = new QuadGraph(boundsMin, boundsMax);

        int w = chf.Width;
        int h = chf.Height;

        // Partition into layers.
        var partition = LayerPartition.Partition(chf);

        // Per-layer arrays: visited[z*w+x] = span index already meshed (-1 if not).
        // We process each layer independently.
        for (int layerId = 0; layerId < partition.NumLayers; layerId++)
        {
            // Build flat walkable arrays for this layer.
            var visited = new bool[w * h];
            var surfaceY = new float[w * h];
            var walkable = new bool[w * h];

            for (int z = 0; z < h; z++)
            {
                for (int x = 0; x < w; x++)
                {
                    var spans = chf.GetSpans(x, z);
                    for (int si = 0; si < spans.Count; si++)
                    {
                        if (partition.GetLayer(x, z, si) != layerId) continue;
                        var span = spans[si];
                        if (span.Area == 0) continue;
                        int idx = z * w + x;
                        // If multiple spans map to this layer (shouldn't happen in a
                        // correct partition, but guard anyway), take the highest Y.
                        if (!walkable[idx] || span.FloorY > surfaceY[idx])
                        {
                            walkable[idx] = true;
                            surfaceY[idx] = span.FloorY;
                        }
                    }
                }
            }

            // Greedy meshing: extend strips in X, then expand in Z.
            for (int z = 0; z < h; z++)
            {
                for (int x = 0; x < w; x++)
                {
                    int idx = z * w + x;
                    if (visited[idx] || !walkable[idx])
                        continue;

                    float surfY = surfaceY[idx];

                    // Extend strip in X.
                    int xEnd = x;
                    while (xEnd + 1 < w
                        && !visited[z * w + (xEnd + 1)]
                        && walkable[z * w + (xEnd + 1)]
                        && MathF.Abs(surfaceY[z * w + (xEnd + 1)] - surfY) <= SurfaceYEps
                        && IsSameLayer(partition, chf, xEnd, z, xEnd + 1, z, layerId))
                        ++xEnd;

                    int stripStartX = x;
                    int stripEndX = xEnd;
                    int zEnd = z;
                    while (zEnd + 1 < h && CanExtendStrip(w, visited, walkable, surfaceY, partition, chf, stripStartX, stripEndX, zEnd, zEnd + 1, surfY, layerId))
                        ++zEnd;

                    for (int markZ = z; markZ <= zEnd; ++markZ)
                        for (int markX = stripStartX; markX <= stripEndX; ++markX)
                            visited[markZ * w + markX] = true;

                    var worldMin = new Vector3(chf.CellMinX(stripStartX), surfY, chf.CellMinZ(z));
                    var worldMax = new Vector3(chf.CellMaxX(stripEndX), surfY, chf.CellMaxZ(zEnd));
                    var quad = new Quad(worldMin.X, surfY, worldMin.Z, worldMax.X, surfY, worldMax.Z, Navmesh.AreaId.Default);
                    graph.AddQuad(quad);
                }
            }
        }

        // Add inter-layer links as off-mesh portals.
        var links = LinkExtractor.ExtractInterLayerLinks(partition);
        foreach (var link in links)
        {
            // Find nearest quads for each link endpoint and add a bidirectional
            // off-mesh portal (AddOffMesh handles the portal + adjacency bookkeeping).
            graph.AddOffMesh(link.PosA, link.PosB, Navmesh.AreaId.Shortcut, bidirectional: true);
        }

        Service.Log.Debug($"[ground] quad graph: {graph.Count} quads, {graph.Portals.Count} portals ({partition.NumLayers} layers, cellSize {chf.CellSize})");
        return graph;
    }

    // Check whether two adjacent cells belong to the same layer (no wall between them).
    private static bool IsSameLayer(LayerPartition partition, CompactHeightfield chf,
        int x1, int z1, int x2, int z2, int layerId)
    {
        // Find the span in the target cell that belongs to this layer.
        var spans2 = chf.GetSpans(x2, z2);
        for (int ni = 0; ni < spans2.Count; ni++)
        {
            if (partition.GetLayer(x2, z2, ni) == layerId)
                return true; // same layer → no wall
        }
        return false; // target cell not in this layer → wall
    }

    private static bool CanExtendStrip(int cellsX, bool[] visited, bool[] walkable, float[] surfaceY,
        LayerPartition partition, CompactHeightfield chf,
        int xStart, int xEnd, int zFrom, int zTo, float y, int layerId)
    {
        for (int x = xStart; x <= xEnd; ++x)
        {
            int idx = zTo * cellsX + x;
            if (visited[idx] || !walkable[idx] || MathF.Abs(surfaceY[idx] - y) > SurfaceYEps)
                return false;
            if (!IsSameLayer(partition, chf, x, zFrom, x, zTo, layerId))
                return false;
        }
        for (int x = xStart; x < xEnd; ++x)
        {
            if (!IsSameLayer(partition, chf, x, zTo, x + 1, zTo, layerId))
                return false;
        }
        return true;
    }
}
