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
            // Phase 3: build ground per-tile from fine CHF data into one shared
            // graph, then connect quads across tile borders via BuildAdjacency.
            // Meshing per tile (~512x512 cells each) avoids materializing a
            // full-zone fine grid (8192x8192 = 67M columns at 0.25y), which is a
            // multi-GB allocation. The border strip of each tile CHF is dropped
            // inside MeshInto so tiles tile cleanly.
            var ground = new QuadGraph(BoundsMin, BoundsMax);
            foreach (var (chf, _, _) in tileCHFs)
                QuadMesher.MeshInto(ground, chf);
            Navmesh = Navmesh with { Ground = ground };
            ground.BuildAdjacency(Settings.AgentMaxClimb, Settings.AgentRadius);
            ground.InitFlags();
            Service.Log.Debug($"[ground] quad graph: {ground.Count} quads, {ground.Portals.Count} portals (per-tile mesh, {tileCHFs.Count} tiles)");
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