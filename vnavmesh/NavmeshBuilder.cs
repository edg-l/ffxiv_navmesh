using Navmesh.GroundGraph;
using Navmesh.GroundGraph.Extraction;
using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;

namespace Navmesh;

// utility for building a navmesh from scene data
// individual tiles can be built concurrently
public class NavmeshBuilder
{
    public NavmeshSettings Settings;
    public SceneExtractor Scene;
    public Vector3 BoundsMin;
    public Vector3 BoundsMax;
    public int NumTilesX;
    public int NumTilesZ;
    public Navmesh Navmesh; // should not be accessed while building tiles

    private NavmeshCustomization customization;

    private int _walkableClimbVoxels;
    private int _walkableHeightVoxels;
    private int _walkableRadiusVoxels;
    private float _walkableNormalThreshold;
    private int _borderSizeVoxels;
    private float _borderSizeWorld;
    private int _tileSizeXVoxels;
    private int _tileSizeZVoxels;
    private int _voxelizerNumX = 1;
    private int _voxelizerNumY = 1;
    private int _voxelizerNumZ = 1;

    public NavmeshBuilder(SceneDefinition scene, NavmeshCustomization customization)
    {
        Settings = customization.Settings;
        this.customization = customization;

        // load all meshes
        Scene = new(scene);
        customization.CustomizeScene(Scene);

        BoundsMin = new(-1024);
        BoundsMax = new(1024);
        NumTilesX = NumTilesZ = Settings.NumTiles[0];
        Service.Log.Debug($"starting building {NumTilesX}x{NumTilesZ} navmesh, customization = {customization.GetType()} v{customization.Version}");

        // create empty navmesh
        var volume = new VoxelMap(BoundsMin, BoundsMax, Settings.NumTiles);
        Navmesh = new(customization.Version, null, volume);

        // calculate derived parameters
        _walkableClimbVoxels = (int)MathF.Floor(Settings.AgentMaxClimb / Settings.CellHeight);
        _walkableHeightVoxels = (int)MathF.Ceiling(Settings.AgentHeight / Settings.CellHeight);
        _walkableRadiusVoxels = (int)MathF.Ceiling(Settings.AgentRadius / Settings.CellSize);
        _walkableNormalThreshold = Settings.AgentMaxSlopeDeg.Degrees().Cos();
        _borderSizeVoxels = 3 + _walkableRadiusVoxels;
        _borderSizeWorld = _borderSizeVoxels * Settings.CellSize;
        float tileWidthWorld = (BoundsMax.X - BoundsMin.X) / NumTilesX;
        float tileHeightWorld = (BoundsMax.Z - BoundsMin.Z) / NumTilesZ;
        _tileSizeXVoxels = (int)MathF.Ceiling(tileWidthWorld / Settings.CellSize) + 2 * _borderSizeVoxels;
        _tileSizeZVoxels = (int)MathF.Ceiling(tileHeightWorld / Settings.CellSize) + 2 * _borderSizeVoxels;
        if (volume != null)
        {
            _voxelizerNumY = Settings.NumTiles[0];
            for (int i = 1; i < Settings.NumTiles.Length; ++i)
            {
                var n = Settings.NumTiles[i];
                _voxelizerNumX *= n;
                _voxelizerNumY *= n;
                _voxelizerNumZ *= n;
            }
        }
    }

    public void BuildTiles(Action? onTileFinished = null)
    {
        var tasks = new List<Task<(VoxelMap? tileVolume, CompactHeightfield? tileCHF, int tileX, int tileZ)>>();

        int threadCount;

        var maxThreads = Environment.ProcessorCount;
        var wantedThreads = Service.Config.BuildMaxCores;
        if (wantedThreads <= 0)
            threadCount = maxThreads + wantedThreads;
        else
            threadCount = wantedThreads;
        threadCount = Math.Clamp(threadCount, 1, maxThreads);

        var sem = new SemaphoreSlim(threadCount, threadCount);

        for (var z = 0; z < NumTilesZ; z++)
        {
            for (var x = 0; x < NumTilesX; x++)
            {
                var z0 = z;
                var x0 = x;
                tasks.Add(Task.Run(async () =>
                {
                    await sem.WaitAsync();
                    try
                    {
                        var (vox, chf, tileX, tileZ) = BuildTile(x0, z0);

                        VoxelMap? tileVolume = null;
                        if (vox != null)
                        {
                            tileVolume = new VoxelMap(BoundsMin, BoundsMax, Settings.NumTiles);
                            tileVolume.Build(vox, x0, z0);
                        }

                        onTileFinished?.Invoke();
                        return (tileVolume, chf, tileX, tileZ);
                    }
                    finally
                    {
                        sem.Release();
                    }
                }));
            }
        }

        // Collect per-tile results. VoxelMap tiles are merged into the global volume;
        // CHFs are stitched together into a single merged CHF for ground extraction.
        var tileCHFs = new List<(CompactHeightfield chf, int tileX, int tileZ)>();
        foreach (var t in tasks)
        {
            t.Wait();
            var (tileVolume, tileCHF, tileX, tileZ) = t.Result;
            if (Navmesh.Volume != null && tileVolume != null)
                MergeTile(Navmesh.Volume, tileX, tileZ, tileVolume);
            if (tileCHF != null)
                tileCHFs.Add((tileCHF, tileX, tileZ));
        }

        if (Navmesh.Volume != null)
        {
            // Phase 3: build ground from fine CHF data.
            var mergedCHF = StitchTileCHFs(tileCHFs);
            var ground = QuadMesher.GreedyMesh(mergedCHF);
            Navmesh = Navmesh with { Ground = ground };
            var climbForAdjacency = Settings.AgentMaxClimb;
            Navmesh.Ground!.BuildAdjacency(climbForAdjacency, Settings.AgentRadius);
            Navmesh.Ground.InitFlags();
        }
    }

    // Stitch per-tile CHFs into a single merged CHF covering the full scene bounds.
    // Each tile CHF has a border strip of width BorderSize that overlaps with
    // neighbours; we strip the border before merging (task 3.7 seam healing).
    private CompactHeightfield StitchTileCHFs(List<(CompactHeightfield chf, int tileX, int tileZ)> tileCHFs)
    {
        // Compute the interior (non-border) cell dimensions per tile and the total
        // dimensions of the merged CHF.
        if (tileCHFs.Count == 0)
            throw new InvalidOperationException("No tile CHFs to stitch");

        var first = tileCHFs[0].chf;
        int border = first.BorderSize;
        int tileInnerW = first.Width - 2 * border;
        int tileInnerH = first.Height - 2 * border;

        int totalW = tileInnerW * NumTilesX;
        int totalH = tileInnerH * NumTilesZ;

        // The merged CHF origin is BoundsMin (no border; border cells were for
        // overlap with neighbours, stripped here).
        var mergedBoundsMin = new Vector3(BoundsMin.X, BoundsMin.Y, BoundsMin.Z);
        int climbVoxels = (int)MathF.Floor(Settings.AgentMaxClimb / Settings.CellHeight);
        var merged = new CompactHeightfield(
            mergedBoundsMin, first.CellSize, first.CellHeight,
            totalW, totalH, 0, climbVoxels, Settings.AgentMaxClimb);

        foreach (var (chf, tx, tz) in tileCHFs)
        {
            int baseX = tx * tileInnerW;
            int baseZ = tz * tileInnerH;

            for (int lz = 0; lz < tileInnerH; lz++)
            {
                for (int lx = 0; lx < tileInnerW; lx++)
                {
                    int srcX = lx + border;
                    int srcZ = lz + border;
                    int dstX = baseX + lx;
                    int dstZ = baseZ + lz;
                    if (dstX >= totalW || dstZ >= totalH)
                        continue;

                    var spans = chf.GetSpans(srcX, srcZ);
                    foreach (var span in spans)
                        merged.AddSpanSorted(dstX, dstZ, span.FloorY, span.Area);
                }
            }
        }

        // Task 3.7: tile-seam healing. At the seam between two adjacent tiles
        // (columns that meet at a tile boundary), ensure the floor-Y values from
        // both sides agree within CellHeight; if they differ by at most CellHeight,
        // average them. This compensates for the rasterizer's per-tile clipping.
        HealTileSeams(merged, tileInnerW, tileInnerH, first.CellHeight);

        merged.FinalizeAllClearances();
        return merged;
    }

    // Heal Y mismatches at tile seam columns. For each seam column pair (the last
    // column of tile n and the first column of tile n+1), if both have a floor span
    // whose Y values differ by <= CellHeight, average them.
    private static void HealTileSeams(CompactHeightfield chf, int tileInnerW, int tileInnerH, float cellHeight)
    {
        int totalW = chf.Width;
        int totalH = chf.Height;

        // Vertical seams (between tiles in X).
        for (int seamX = tileInnerW; seamX < totalW; seamX += tileInnerW)
        {
            int leftX = seamX - 1;
            int rightX = seamX;
            if (rightX >= totalW)
                break;
            for (int z = 0; z < totalH; z++)
                SnapSeamY(chf.GetSpansMutable(leftX, z), chf.GetSpansMutable(rightX, z), cellHeight);
        }

        // Horizontal seams (between tiles in Z).
        for (int seamZ = tileInnerH; seamZ < totalH; seamZ += tileInnerH)
        {
            int bottomZ = seamZ - 1;
            int topZ = seamZ;
            if (topZ >= totalH)
                break;
            for (int x = 0; x < totalW; x++)
                SnapSeamY(chf.GetSpansMutable(x, bottomZ), chf.GetSpansMutable(x, topZ), cellHeight);
        }
    }

    private static void SnapSeamY(List<FloorSpan> a, List<FloorSpan> b, float cellHeight)
    {
        // For each span in a, find the closest-Y span in b. If they differ by at
        // most cellHeight, set both to the average. Only snap walkable spans.
        for (int i = 0; i < a.Count; i++)
        {
            if (a[i].Area == 0) continue;
            for (int j = 0; j < b.Count; j++)
            {
                if (b[j].Area == 0) continue;
                float diff = MathF.Abs(a[i].FloorY - b[j].FloorY);
                if (diff <= cellHeight && diff > 0)
                {
                    float avg = (a[i].FloorY + b[j].FloorY) * 0.5f;
                    a[i] = a[i] with { FloorY = avg };
                    b[j] = b[j] with { FloorY = avg };
                    break;
                }
                else if (diff > cellHeight)
                {
                    // Seam Y gap exceeds one cell height; cannot average safely.
                    Service.Log.Warning($"Tile seam Y mismatch: span A floorY={a[i].FloorY:F4}, span B floorY={b[j].FloorY:F4}, diff={diff:F4} > cellHeight={cellHeight:F4}; seam left unsnapped.");
                }
            }
        }
    }

    private static void MergeTile(VoxelMap parent, int x, int z, VoxelMap child)
    {
        var shift = parent.RootTile.Subdivision.Count;

        for (ushort i = 0; i < child.RootTile.Contents.Length; i++)
        {
            var contents = child.RootTile.Contents[i];
            if ((contents & VoxelMap.VoxelOccupiedBit) == 0)
                continue; // empty

            if ((contents & VoxelMap.VoxelIdMask) != VoxelMap.VoxelIdMask)
                contents += (ushort)shift;

            parent.RootTile.Contents[i] = contents;
        }
        parent.RootTile.Subdivision.AddRange(child.RootTile.Subdivision);
    }

    // this can be called concurrently
    public (Voxelizer?, CompactHeightfield?, int tileX, int tileZ) BuildTile(int x, int z)
    {
        var timer = Timer.Create();

        // 0. calculate tile bounds
        // we expand the voxelization bounding box by border size to find the extents of geometry we need to build this tile
        float tileWidthWorld = (BoundsMax.X - BoundsMin.X) / NumTilesX;
        float tileHeightWorld = (BoundsMax.Z - BoundsMin.Z) / NumTilesZ;
        var tileBoundsMin = new Vector3(BoundsMin.X + x * tileWidthWorld, BoundsMin.Y, BoundsMin.Z + z * tileHeightWorld);
        var tileBoundsMax = new Vector3(tileBoundsMin.X + tileWidthWorld, BoundsMax.Y, tileBoundsMin.Z + tileHeightWorld);
        tileBoundsMin.X -= _borderSizeWorld;
        tileBoundsMin.Z -= _borderSizeWorld;
        tileBoundsMax.X += _borderSizeWorld;
        tileBoundsMax.Z += _borderSizeWorld;

        // 1. voxelize raw geometry into the voxelizer; simultaneously build the
        //    fine CompactHeightfield at 0.25y resolution for ground extraction.
        //    CHF is only built when Volume is present (flight pathfinding enabled).
        var vox = Navmesh.Volume != null ? new Voxelizer(_voxelizerNumX, _voxelizerNumY, _voxelizerNumZ) : null;
        int tileW = (int)MathF.Ceiling((tileBoundsMax.X - tileBoundsMin.X) / Settings.CellSize);
        int tileH = (int)MathF.Ceiling((tileBoundsMax.Z - tileBoundsMin.Z) / Settings.CellSize);
        CompactHeightfield? chf = null;
        if (Navmesh.Volume != null)
        {
            chf = new CompactHeightfield(
                tileBoundsMin, Settings.CellSize, Settings.CellHeight,
                tileW, tileH, _borderSizeVoxels, _walkableClimbVoxels, Settings.AgentMaxClimb);
        }

        var rasterizer = new NavmeshRasterizer(vox, tileBoundsMin, tileBoundsMax, Settings.CellSize, Settings.CellHeight, _borderSizeVoxels, _walkableNormalThreshold, _walkableClimbVoxels, _walkableHeightVoxels, Settings.Filtering.HasFlag(NavmeshSettings.Filter.Interiors), chf);
        rasterizer.Rasterize(Scene, SceneExtractor.MeshType.FileMesh | SceneExtractor.MeshType.CylinderMesh | SceneExtractor.MeshType.AnalyticShape, true, true); // rasterize normal geometry
        rasterizer.Rasterize(Scene, SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.AnalyticPlane, false, true); // rasterize terrain and bounding planes

        // Populate the CHF from accumulated raw spans (BEFORE iset is discarded).
        rasterizer.PopulateChf();

        Service.Log.Debug($"built navmesh tile {x}x{z} in {timer.Value().TotalMilliseconds}ms");
        return (vox, chf, x, z);
    }
}