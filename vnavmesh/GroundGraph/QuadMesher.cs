using Navmesh.NavVolume;
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
    public static QuadGraph GreedyMesh(VoxelMap volume, Vector3 boundsMin, Vector3 boundsMax)
    {
        var graph = new QuadGraph(boundsMin, boundsMax);

        var leafLevel = volume.Levels[^1];
        var cellSize = leafLevel.CellSize;
        var origin = volume.RootTile.BoundsMin;

        var totalCellsX = 1;
        var totalCellsY = 1;
        var totalCellsZ = 1;
        foreach (var lvl in volume.Levels)
        {
            totalCellsX *= lvl.NumCellsX;
            totalCellsY *= lvl.NumCellsY;
            totalCellsZ *= lvl.NumCellsZ;
        }

        var (minX, minY, minZ) = WorldToLeafIndex(boundsMin, origin, cellSize, totalCellsX, totalCellsY, totalCellsZ);
        var (maxX, maxY, maxZ) = WorldToLeafIndex(boundsMax, origin, cellSize, totalCellsX, totalCellsY, totalCellsZ);

        var visited = new bool[totalCellsX * totalCellsZ];
        var surfaceY = new float[totalCellsX * totalCellsZ];
        var walkable = new bool[totalCellsX * totalCellsZ];

        for (int y = minY; y <= maxY; ++y)
        {
            Array.Fill(visited, false);
            Array.Fill(walkable, false);

            for (int z = minZ; z <= maxZ; ++z)
            {
                for (int x = minX; x <= maxX; ++x)
                {
                    var idx = z * totalCellsX + x;
                    var abovePos = LeafIndexToWorld(x, y, z, origin, cellSize);
                    var aboveVoxel = volume.FindLeafVoxel(abovePos);
                    if (aboveVoxel.empty)
                    {
                        var belowPos = LeafIndexToWorld(x, y - 1, z, origin, cellSize);
                        var belowVoxel = volume.FindLeafVoxel(belowPos);
                        if (!belowVoxel.empty)
                        {
                            walkable[idx] = true;
                            var solidTopY = belowPos.Y + cellSize.Y * 0.5f;
                            surfaceY[idx] = solidTopY;
                        }
                    }
                }
            }

            for (int z = minZ; z <= maxZ; ++z)
            {
                for (int x = minX; x <= maxX; ++x)
                {
                    var idx = z * totalCellsX + x;
                    if (visited[idx] || !walkable[idx])
                        continue;

                    var surfY = surfaceY[idx];
                    var xEnd = x;
                    while (xEnd + 1 <= maxX && !visited[(z * totalCellsX) + (xEnd + 1)] && walkable[(z * totalCellsX) + (xEnd + 1)] && surfaceY[(z * totalCellsX) + (xEnd + 1)] == surfY)
                        ++xEnd;

                    int stripStartX = x;
                    int stripEndX = xEnd;
                    int zEnd = z;
                    while (zEnd + 1 <= maxZ && CanExtendStrip(totalCellsX, visited, walkable, surfaceY, stripStartX, stripEndX, zEnd + 1, surfY))
                        ++zEnd;

                    for (int markZ = z; markZ <= zEnd; ++markZ)
                        for (int markX = stripStartX; markX <= stripEndX; ++markX)
                            visited[markZ * totalCellsX + markX] = true;

                    var worldMin = LeafIndexToWorld(stripStartX, y, z, origin, cellSize);
                    var worldMax = LeafIndexToWorld(stripEndX, y, zEnd, origin, cellSize);
                    var quad = new Quad(worldMin.X, surfY, worldMin.Z, worldMax.X + cellSize.X, surfY, worldMax.Z + cellSize.Z, Navmesh.AreaId.Default);
                    graph.AddQuad(quad);
                }
            }
        }

        Service.Log.Debug($"[ground] quad graph: {graph.Count} quads (leaf grid {totalCellsX}x{totalCellsY}x{totalCellsZ}, cellSize {cellSize})");
        return graph;
    }

    private static bool CanExtendStrip(int cellsX, bool[] visited, bool[] walkable, float[] surfaceY, int xStart, int xEnd, int z, float y)
    {
        for (int x = xStart; x <= xEnd; ++x)
        {
            var idx = z * cellsX + x;
            if (visited[idx] || !walkable[idx] || surfaceY[idx] != y)
                return false;
        }
        return true;
    }

    private static (int x, int y, int z) WorldToLeafIndex(Vector3 p, Vector3 origin, Vector3 cellSize, int totalX, int totalY, int totalZ)
    {
        var frac = (p - origin) / cellSize;
        return (Math.Clamp((int)frac.X, 0, totalX - 1), Math.Clamp((int)frac.Y, 0, totalY - 1), Math.Clamp((int)frac.Z, 0, totalZ - 1));
    }

    private static Vector3 LeafIndexToWorld(int x, int y, int z, Vector3 origin, Vector3 cellSize)
    {
        return origin + new Vector3((x + 0.5f) * cellSize.X, (y + 0.5f) * cellSize.Y, (z + 0.5f) * cellSize.Z);
    }
}