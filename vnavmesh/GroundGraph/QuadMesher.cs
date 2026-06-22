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
        var totalCellsZ = 1;
        foreach (var lvl in volume.Levels)
        {
            totalCellsX *= lvl.NumCellsX;
            totalCellsZ *= lvl.NumCellsZ;
        }

        var visited = new bool[totalCellsX * totalCellsZ];
        var surfaceY = new float[totalCellsX * totalCellsZ];
        var walkable = new bool[totalCellsX * totalCellsZ];

        ScanTile(volume.RootTile, volume, origin, cellSize, totalCellsX, totalCellsZ, visited, surfaceY, walkable);

        for (int z = 0; z < totalCellsZ; ++z)
        {
            for (int x = 0; x < totalCellsX; ++x)
            {
                var idx = z * totalCellsX + x;
                if (visited[idx] || !walkable[idx])
                    continue;

                var surfY = surfaceY[idx];
                var xEnd = x;
                while (xEnd + 1 < totalCellsX && !visited[(z * totalCellsX) + (xEnd + 1)] && walkable[(z * totalCellsX) + (xEnd + 1)] && surfaceY[(z * totalCellsX) + (xEnd + 1)] == surfY)
                    ++xEnd;

                int stripStartX = x;
                int stripEndX = xEnd;
                int zEnd = z;
                while (zEnd + 1 < totalCellsZ && CanExtendStrip(totalCellsX, visited, walkable, surfaceY, stripStartX, stripEndX, zEnd + 1, surfY))
                    ++zEnd;

                for (int markZ = z; markZ <= zEnd; ++markZ)
                    for (int markX = stripStartX; markX <= stripEndX; ++markX)
                        visited[markZ * totalCellsX + markX] = true;

                var worldMin = origin + new Vector3(stripStartX * cellSize.X, 0, z * cellSize.Z);
                var worldMax = origin + new Vector3((stripEndX + 1) * cellSize.X, 0, (zEnd + 1) * cellSize.Z);
                var quad = new Quad(worldMin.X, surfY, worldMin.Z, worldMax.X, surfY, worldMax.Z, Navmesh.AreaId.Default);
                graph.AddQuad(quad);
            }
        }

        Service.Log.Debug($"[ground] quad graph: {graph.Count} quads (leaf grid {totalCellsX}x{totalCellsZ}, cellSize {cellSize})");
        return graph;
    }

    private static void ScanTile(VoxelMap.Tile tile, VoxelMap volume, Vector3 origin, Vector3 leafCellSize, int totalCellsX, int totalCellsZ, bool[] visited, float[] surfaceY, bool[] walkable)
    {
        var ld = tile.LevelDesc;
        for (int i = 0; i < tile.Contents.Length; ++i)
        {
            var data = tile.Contents[i];
            if ((data & VoxelMap.VoxelOccupiedBit) == 0)
                continue;

            data &= VoxelMap.VoxelIdMask;
            if (data == VoxelMap.VoxelIdMask)
                continue;

            var (cx, cy, cz) = ld.IndexToVoxel((ushort)i);
            if (tile.Level < volume.Levels.Length - 1)
            {
                ScanTile(tile.Subdivision[(int)data], volume, origin, leafCellSize, totalCellsX, totalCellsZ, visited, surfaceY, walkable);
            }
            else
            {
                var worldPos = tile.VoxelToWorld(cx, cy, cz);
                var aboveVoxel = volume.FindLeafVoxel(worldPos);
                if (!aboveVoxel.empty)
                    continue;

                var belowPos = worldPos - new Vector3(0, leafCellSize.Y, 0);
                var belowVoxel = volume.FindLeafVoxel(belowPos);
                if (belowVoxel.empty)
                    continue;

                var leafIdxX = (int)((worldPos.X - origin.X) / leafCellSize.X);
                var leafIdxZ = (int)((worldPos.Z - origin.Z) / leafCellSize.Z);
                if (leafIdxX < 0 || leafIdxX >= totalCellsX || leafIdxZ < 0 || leafIdxZ >= totalCellsZ)
                    continue;

                var idx = leafIdxZ * totalCellsX + leafIdxX;
                var solidTopY = belowPos.Y + leafCellSize.Y * 0.5f;
                if (!walkable[idx] || solidTopY > surfaceY[idx])
                {
                    walkable[idx] = true;
                    surfaceY[idx] = solidTopY;
                }
            }
        }
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
}