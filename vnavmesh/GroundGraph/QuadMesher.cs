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
        var cellsX = leafLevel.NumCellsX;
        var cellsZ = leafLevel.NumCellsZ;
        var cellsY = leafLevel.NumCellsY;

        Service.Log.Debug($"[ground] leaf level: cells={cellsX}x{cellsY}x{cellsZ}, cellSize={cellSize}, levels={volume.Levels.Length}");

        var (minX, minY, minZ) = WorldToLeafIndex(volume, boundsMin, leafLevel);
        var (maxX, maxY, maxZ) = WorldToLeafIndex(volume, boundsMax, leafLevel);
        minX = Math.Max(0, minX);
        minY = Math.Max(0, minY);
        minZ = Math.Max(0, minZ);
        maxX = Math.Min(cellsX - 1, maxX);
        maxY = Math.Min(cellsY - 1, maxY);
        maxZ = Math.Min(cellsZ - 1, maxZ);

        Service.Log.Debug($"[ground] scan range: x=[{minX},{maxX}] y=[{minY},{maxY}] z=[{minZ},{maxZ}]");

        int walkableCount = 0;
        int occupiedCount = 0;
        int emptyCount = 0;

        var visited = new bool[cellsX * cellsZ];
        var surfaceY = new float[cellsX * cellsZ];
        var walkable = new bool[cellsX * cellsZ];

        for (int y = minY; y <= maxY; ++y)
        {
            Array.Fill(visited, false);
            Array.Fill(walkable, false);

            for (int z = minZ; z <= maxZ; ++z)
            {
                for (int x = minX; x <= maxX; ++x)
                {
                    var idx = z * cellsX + x;
                    var abovePos = LeafIndexToWorld(volume, x, y, z, leafLevel);
                    var aboveVoxel = volume.FindLeafVoxel(abovePos);
                    if (aboveVoxel.empty)
                        emptyCount++;
                    else
                        occupiedCount++;

                    if (aboveVoxel.empty)
                    {
                        var belowPos = LeafIndexToWorld(volume, x, y - 1, z, leafLevel);
                        var belowVoxel = volume.FindLeafVoxel(belowPos);
                        if (!belowVoxel.empty)
                        {
                            walkable[idx] = true;
                            walkableCount++;
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
                    var idx = z * cellsX + x;
                    if (visited[idx] || !walkable[idx])
                        continue;

                    var surfY = surfaceY[idx];
                    var xEnd = x;
                    while (xEnd + 1 <= maxX && !visited[(z * cellsX) + (xEnd + 1)] && walkable[(z * cellsX) + (xEnd + 1)] && surfaceY[(z * cellsX) + (xEnd + 1)] == surfY)
                        ++xEnd;

                    int stripStartX = x;
                    int stripEndX = xEnd;
                    int zEnd = z;
                    while (zEnd + 1 <= maxZ && CanExtendStrip(cellsX, visited, walkable, surfaceY, stripStartX, stripEndX, zEnd + 1, surfY))
                        ++zEnd;

                    for (int markZ = z; markZ <= zEnd; ++markZ)
                        for (int markX = stripStartX; markX <= stripEndX; ++markX)
                            visited[markZ * cellsX + markX] = true;

                    var worldMin = LeafIndexToWorld(volume, stripStartX, y, z, leafLevel);
                    var worldMax = LeafIndexToWorld(volume, stripEndX, y, zEnd, leafLevel);
                    var quad = new Quad(worldMin.X, surfY, worldMin.Z, worldMax.X + cellSize.X, surfY, worldMax.Z + cellSize.Z, Navmesh.AreaId.Default);
                    graph.AddQuad(quad);
                }
            }
        }

        Service.Log.Debug($"[ground] voxel stats: {walkableCount} walkable, {occupiedCount} occupied, {emptyCount} empty, {graph.Count} quads");
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

    private static (int x, int y, int z) WorldToLeafIndex(VoxelMap volume, Vector3 p, VoxelMap.Level leafLevel)
    {
        var frac = (p - volume.RootTile.BoundsMin) / leafLevel.CellSize;
        return ((int)frac.X, (int)frac.Y, (int)frac.Z);
    }

    private static Vector3 LeafIndexToWorld(VoxelMap volume, int x, int y, int z, VoxelMap.Level leafLevel)
    {
        var basePos = volume.RootTile.BoundsMin + new Vector3((x + 0.5f) * leafLevel.CellSize.X, (y + 0.5f) * leafLevel.CellSize.Y, (z + 0.5f) * leafLevel.CellSize.Z);
        return basePos;
    }
}