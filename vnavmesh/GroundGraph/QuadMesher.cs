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
        var leafCellSize = leafLevel.CellSize;
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

        var visited = new bool[totalCellsX * totalCellsZ];
        var surfaceY = new float[totalCellsX * totalCellsZ];
        var walkable = new bool[totalCellsX * totalCellsZ];

        var rootLevel = volume.Levels[0];
        var l0Nx = rootLevel.NumCellsX;
        var l0Ny = rootLevel.NumCellsY;
        var l0Nz = rootLevel.NumCellsZ;
        var l0CellSize = rootLevel.CellSize;

        for (int l0z = 0; l0z < l0Nz; ++l0z)
        {
            for (int l0x = 0; l0x < l0Nx; ++l0x)
            {
                for (int l0y = l0Ny - 1; l0y >= 0; --l0y)
                {
                    var l0Idx = rootLevel.VoxelToIndex(l0x, l0y, l0z);
                    var l0Data = volume.RootTile.Contents[l0Idx];
                    if ((l0Data & VoxelMap.VoxelOccupiedBit) == 0)
                        continue;
                    var l0Sub = l0Data & VoxelMap.VoxelIdMask;
                    if (l0Sub == VoxelMap.VoxelIdMask)
                        continue;

                    ScanMixedTileForSurfaces(volume.RootTile.Subdivision[l0Sub], volume, origin, leafCellSize, totalCellsX, totalCellsZ, surfaceY, walkable);
                }
            }
        }

        for (int l0z = 0; l0z < l0Nz; ++l0z)
        {
            for (int l0x = 0; l0x < l0Nx; ++l0x)
            {
                var colFound = false;
                for (int l0y = l0Ny - 1; l0y >= 0 && !colFound; --l0y)
                {
                    var l0Idx = rootLevel.VoxelToIndex(l0x, l0y, l0z);
                    var l0Data = volume.RootTile.Contents[l0Idx];
                    if ((l0Data & VoxelMap.VoxelOccupiedBit) != 0)
                    {
                        colFound = true;
                        if ((l0Data & VoxelMap.VoxelIdMask) != VoxelMap.VoxelIdMask)
                        {
                            // mixed - already scanned above
                        }
                        else
                        {
                            // fully solid L0 cell - surface is at its top
                            var topY = origin.Y + (l0y + 1) * l0CellSize.Y;
                            MarkColumnRange(volume, origin, leafCellSize, l0x, l0z, l0CellSize, totalCellsX, totalCellsZ, surfaceY, walkable, topY, true);
                        }
                    }
                    else
                    {
                        // empty L0 cell - check if L0 cell below is solid
                        if (l0y > 0)
                        {
                            var belowIdx = rootLevel.VoxelToIndex(l0x, l0y - 1, l0z);
                            var belowData = volume.RootTile.Contents[belowIdx];
                            if ((belowData & VoxelMap.VoxelOccupiedBit) != 0)
                            {
                                colFound = true;
                                if ((belowData & VoxelMap.VoxelIdMask) == VoxelMap.VoxelIdMask)
                                {
                                    // solid below - surface is at top of solid cell
                                    var topY = origin.Y + l0y * l0CellSize.Y;
                                    MarkColumnRange(volume, origin, leafCellSize, l0x, l0z, l0CellSize, totalCellsX, totalCellsZ, surfaceY, walkable, topY, false);
                                }
                                // if below is mixed, the mixed scan already found the surface
                            }
                        }
                    }
                }
            }
        }

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

                var worldMin = origin + new Vector3(stripStartX * leafCellSize.X, 0, z * leafCellSize.Z);
                var worldMax = origin + new Vector3((stripEndX + 1) * leafCellSize.X, 0, (zEnd + 1) * leafCellSize.Z);
                var quad = new Quad(worldMin.X, surfY, worldMin.Z, worldMax.X, surfY, worldMax.Z, Navmesh.AreaId.Default);
                graph.AddQuad(quad);
            }
        }

        Service.Log.Debug($"[ground] quad graph: {graph.Count} quads, {graph.Portals.Count} portals (leaf grid {totalCellsX}x{totalCellsZ}, cellSize {leafCellSize})");
        return graph;
    }

    private static void ScanMixedTileForSurfaces(VoxelMap.Tile tile, VoxelMap volume, Vector3 origin, Vector3 leafCellSize, int totalCellsX, int totalCellsZ, float[] surfaceY, bool[] walkable)
    {
        var ld = tile.LevelDesc;
        var isLeaf = tile.Level == volume.Levels.Length - 1;

        for (int i = 0; i < tile.Contents.Length; ++i)
        {
            var data = tile.Contents[i];
            var occupied = (data & VoxelMap.VoxelOccupiedBit) != 0;
            var subIdx = data & VoxelMap.VoxelIdMask;

            if (!occupied)
            {
                if (isLeaf)
                {
                    var (cx, cy, cz) = ld.IndexToVoxel((ushort)i);
                    var worldPos = tile.VoxelToWorld(cx, cy, cz);
                    var aboveVoxel = volume.FindLeafVoxel(worldPos);
                    if (aboveVoxel.empty)
                    {
                        var belowPos = worldPos - new Vector3(0, leafCellSize.Y, 0);
                        var belowVoxel = volume.FindLeafVoxel(belowPos);
                        if (!belowVoxel.empty)
                        {
                            var leafIdxX = (int)((worldPos.X - origin.X) / leafCellSize.X);
                            var leafIdxZ = (int)((worldPos.Z - origin.Z) / leafCellSize.Z);
                            if (leafIdxX >= 0 && leafIdxX < totalCellsX && leafIdxZ >= 0 && leafIdxZ < totalCellsZ)
                            {
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
                }
                continue;
            }

            if (subIdx == VoxelMap.VoxelIdMask)
                continue;

            if (isLeaf)
            {
                var (cx, cy, cz) = ld.IndexToVoxel((ushort)i);
                var worldPos = tile.VoxelToWorld(cx, cy, cz);
                var aboveVoxel = volume.FindLeafVoxel(worldPos);
                if (aboveVoxel.empty)
                {
                    var belowPos = worldPos - new Vector3(0, leafCellSize.Y, 0);
                    var belowVoxel = volume.FindLeafVoxel(belowPos);
                    if (!belowVoxel.empty)
                    {
                        var leafIdxX = (int)((worldPos.X - origin.X) / leafCellSize.X);
                        var leafIdxZ = (int)((worldPos.Z - origin.Z) / leafCellSize.Z);
                        if (leafIdxX >= 0 && leafIdxX < totalCellsX && leafIdxZ >= 0 && leafIdxZ < totalCellsZ)
                        {
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
            }
            else
            {
                ScanMixedTileForSurfaces(tile.Subdivision[subIdx], volume, origin, leafCellSize, totalCellsX, totalCellsZ, surfaceY, walkable);
            }
        }
    }

    private static void MarkColumnRange(VoxelMap volume, Vector3 origin, Vector3 leafCellSize, int l0x, int l0z, Vector3 l0CellSize, int totalCellsX, int totalCellsZ, float[] surfaceY, bool[] walkable, float approxTopY, bool isSolidTile)
    {
        var xStart = (int)((origin.X + l0x * l0CellSize.X - origin.X) / leafCellSize.X);
        var xEnd = (int)((origin.X + (l0x + 1) * l0CellSize.X - origin.X) / leafCellSize.X);
        var zStart = (int)((origin.Z + l0z * l0CellSize.Z - origin.Z) / leafCellSize.Z);
        var zEnd = (int)((origin.Z + (l0z + 1) * l0CellSize.Z - origin.Z) / leafCellSize.Z);

        for (int lz = zStart; lz < zEnd; ++lz)
        {
            for (int lx = xStart; lx < xEnd; ++lx)
            {
                if (lx < 0 || lx >= totalCellsX || lz < 0 || lz >= totalCellsZ)
                    continue;

                var probeY = approxTopY;
                var probePos = origin + new Vector3((lx + 0.5f) * leafCellSize.X, probeY, (lz + 0.5f) * leafCellSize.Z);
                var aboveVoxel = volume.FindLeafVoxel(probePos);
                if (!aboveVoxel.empty)
                {
                    for (int dy = 0; dy < 64; ++dy)
                    {
                        probePos.Y += leafCellSize.Y;
                        aboveVoxel = volume.FindLeafVoxel(probePos);
                        if (aboveVoxel.empty)
                            break;
                    }
                }

                if (!aboveVoxel.empty)
                    continue;

                var belowPos = probePos - new Vector3(0, leafCellSize.Y, 0);
                var belowVoxel = volume.FindLeafVoxel(belowPos);
                if (belowVoxel.empty)
                {
                    for (int dy = 0; dy < 64; ++dy)
                    {
                        belowPos.Y -= leafCellSize.Y;
                        belowVoxel = volume.FindLeafVoxel(belowPos);
                        if (!belowVoxel.empty)
                            break;
                    }
                }

                if (belowVoxel.empty)
                    continue;

                var idx = lz * totalCellsX + lx;
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