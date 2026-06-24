using Dalamud.Plugin.Services;
using Navmesh.GroundGraph;
using Navmesh.NavVolume;
using Serilog;
using Serilog.Events;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Reflection;
using System.Threading;
using Xunit;

namespace Navmesh.Tests;

// Stub IPluginLog that discards all log messages.
file sealed class NullPluginLog : IPluginLog
{
    public ILogger Logger => Serilog.Core.Logger.None;
    public LogEventLevel MinimumLogLevel { get; set; } = LogEventLevel.Fatal;

    public void Fatal(string messageTemplate, params object[] values) { }
    public void Fatal(Exception? exception, string messageTemplate, params object[] values) { }
    public void Error(string messageTemplate, params object[] values) { }
    public void Error(Exception? exception, string messageTemplate, params object[] values) { }
    public void Warning(string messageTemplate, params object[] values) { }
    public void Warning(Exception? exception, string messageTemplate, params object[] values) { }
    public void Information(string messageTemplate, params object[] values) { }
    public void Information(Exception? exception, string messageTemplate, params object[] values) { }
    public void Info(string messageTemplate, params object[] values) { }
    public void Info(Exception? exception, string messageTemplate, params object[] values) { }
    public void Debug(string messageTemplate, params object[] values) { }
    public void Debug(Exception? exception, string messageTemplate, params object[] values) { }
    public void Verbose(string messageTemplate, params object[] values) { }
    public void Verbose(Exception? exception, string messageTemplate, params object[] values) { }
    public void Write(LogEventLevel level, Exception? exception, string messageTemplate, params object[] values) { }
}

// One-time setup: inject the stub logger and default Config so Service.Log works.
public sealed class ServiceFixture : IDisposable
{
    public ServiceFixture()
    {
        var logProp = typeof(Service).GetProperty(nameof(Service.Log),
            BindingFlags.Public | BindingFlags.Static)!;
        logProp.SetValue(null, new NullPluginLog());
        Service.Config = new Config();
    }

    public void Dispose() { }
}

// Builds a QuadGraph from a Scene using the same pipeline as NavmeshBuilder.
internal static class SceneHelper
{
    private const float MaxClimbBase = 0.5f;
    private const float AgentRadius = 0.5f;

    public static QuadGraph BuildGraph(Scene scene)
    {
        var vol = scene.Volume;
        var ground = QuadMesher.GreedyMesh(vol, scene.BoundsMin, scene.BoundsMax);
        var leafCellSize = vol.Levels[^1].CellSize;
        var climbForAdjacency = MathF.Max(MaxClimbBase, leafCellSize.Y * 1.5f);
        ground.BuildAdjacency(climbForAdjacency, AgentRadius);
        ground.InitFlags();
        return ground;
    }

    public static List<Vector3>? Pathfind(QuadGraph graph, Vector3 from, Vector3 to)
    {
        var path = graph.Pathfind(from, to, false, true, 0, CancellationToken.None);
        return path.Count == 0 ? null : path;
    }
}

// Deterministic pair generator for a scene's walkable points.
file static class PairGen
{
    public static IEnumerable<(Vector3 from, Vector3 to)> Generate(List<Vector3> points, int seed, int count)
    {
        var rng = new Random(seed);
        for (int i = 0; i < count; i++)
        {
            var from = points[rng.Next(points.Count)];
            var to = points[rng.Next(points.Count)];
            yield return (from, to);
        }
    }
}

[Collection("BackendTests")]
public class BackendTests : IClassFixture<ServiceFixture>
{
    private const int PairsPerScene = 200;
    private const int Seed = 42;

    [Fact]
    public void FillBox_FlatPlane_SolidLeafCount()
    {
        // Checkpoint test: FlatPlane floor should produce a specific solid leaf count.
        // FlatPlane fills [BMin.X, 0, BMin.Z] to [BMax.X, 1.0, BMax.Z].
        // Bounds: [-20,-10,-20] to [20,10,20], tiles [4,4,4].
        // Leaf cell size: X=0.625, Y=0.3125, Z=0.625.
        // Floor covers all XZ (40 wide / 0.625 = 64 leaf cells per axis).
        // Y span 0..1.0 = 1.0/0.3125 = 3.2 → 4 leaf cells in Y.
        // Solid leaves ≈ 64 * 4 * 64 = 16384, but exact count depends on center-in-box filter.
        var scene = SyntheticScenes.FlatPlane();
        int solidCount = CountSolidLeaves(scene.Volume);
        Assert.True(solidCount > 0, "FillBox produced no solid leaves");
        // The floor must have many solid leaves covering the XZ plane.
        // With 64x64 XZ cells and ~3-4 Y cells in [0,1.0], expect at least 10000 solid leaves.
        Assert.True(solidCount >= 10000,
            $"Expected at least 10000 solid leaves for FlatPlane floor, got {solidCount}");
    }

    [Fact]
    public void FlatPlane_PathfindOracle()
    {
        var scene = SyntheticScenes.FlatPlane();
        var graph = SceneHelper.BuildGraph(scene);
        float k = GeometryOracle.KForScene(scene.Name);

        foreach (var (from, to) in PairGen.Generate(scene.WalkablePoints, Seed, PairsPerScene))
        {
            if (from == to)
                continue;
            var path = SceneHelper.Pathfind(graph, from, to);
            if (path != null)
                GeometryOracle.AssertPathValid(scene, graph, path, from, to, k, assertStrictAnyAngle: false);
        }
    }

    [Fact]
    public void FlatPlane_StrictAnyAngle()
    {
        var scene = SyntheticScenes.FlatPlane();
        var graph = SceneHelper.BuildGraph(scene);
        float k = GeometryOracle.KForScene(scene.Name);

        foreach (var (from, to) in PairGen.Generate(scene.WalkablePoints, Seed, PairsPerScene))
        {
            if (from == to)
                continue;
            var path = SceneHelper.Pathfind(graph, from, to);
            if (path != null)
                GeometryOracle.AssertPathValid(scene, graph, path, from, to, k, assertStrictAnyAngle: true);
        }
    }

    [Fact]
    public void FlatPlane_GoldenSnapshot()
    {
        var scene = SyntheticScenes.FlatPlane();
        var graph = SceneHelper.BuildGraph(scene);
        GoldenSnapshot.AssertGround(graph, scene.Name);
    }

    [Fact]
    public void PlaneWithPillar_PathfindOracle()
    {
        var scene = SyntheticScenes.PlaneWithPillar();
        var graph = SceneHelper.BuildGraph(scene);
        float k = GeometryOracle.KForScene(scene.Name);

        foreach (var (from, to) in PairGen.Generate(scene.WalkablePoints, Seed, PairsPerScene))
        {
            if (from == to)
                continue;
            var path = SceneHelper.Pathfind(graph, from, to);
            if (path != null)
                GeometryOracle.AssertPathValid(scene, graph, path, from, to, k);
        }
    }

    [Fact]
    public void PlaneWithPillar_GoldenSnapshot()
    {
        var scene = SyntheticScenes.PlaneWithPillar();
        var graph = SceneHelper.BuildGraph(scene);
        GoldenSnapshot.AssertGround(graph, scene.Name);
    }

    [Fact]
    public void Overpass_PathfindOracle()
    {
        var scene = SyntheticScenes.Overpass();
        var graph = SceneHelper.BuildGraph(scene);
        float k = GeometryOracle.KForScene(scene.Name);

        foreach (var (from, to) in PairGen.Generate(scene.WalkablePoints, Seed, PairsPerScene))
        {
            if (from == to)
                continue;
            var path = SceneHelper.Pathfind(graph, from, to);
            if (path != null)
                GeometryOracle.AssertPathValid(scene, graph, path, from, to, k);
        }
    }

    [Fact]
    public void Overpass_GoldenSnapshot()
    {
        var scene = SyntheticScenes.Overpass();
        var graph = SceneHelper.BuildGraph(scene);
        GoldenSnapshot.AssertGround(graph, scene.Name);
    }

    [Fact]
    public void BridgeOnramp_PathfindOracle()
    {
        var scene = SyntheticScenes.BridgeOnramp();
        var graph = SceneHelper.BuildGraph(scene);
        float k = GeometryOracle.KForScene(scene.Name);

        foreach (var (from, to) in PairGen.Generate(scene.WalkablePoints, Seed, PairsPerScene))
        {
            if (from == to)
                continue;
            var path = SceneHelper.Pathfind(graph, from, to);
            if (path != null)
                GeometryOracle.AssertPathValid(scene, graph, path, from, to, k);
        }
    }

    [Fact]
    public void BridgeOnramp_GoldenSnapshot()
    {
        var scene = SyntheticScenes.BridgeOnramp();
        var graph = SceneHelper.BuildGraph(scene);
        GoldenSnapshot.AssertGround(graph, scene.Name);
    }

    [Fact]
    public void NarrowCorridor_PathfindOracle()
    {
        var scene = SyntheticScenes.NarrowCorridor(4.0f);
        var graph = SceneHelper.BuildGraph(scene);
        float k = GeometryOracle.KForScene(scene.Name);

        foreach (var (from, to) in PairGen.Generate(scene.WalkablePoints, Seed, PairsPerScene))
        {
            if (from == to)
                continue;
            var path = SceneHelper.Pathfind(graph, from, to);
            if (path != null)
                GeometryOracle.AssertPathValid(scene, graph, path, from, to, k);
        }
    }

    [Fact]
    public void NarrowCorridor_GoldenSnapshot()
    {
        var scene = SyntheticScenes.NarrowCorridor(4.0f);
        var graph = SceneHelper.BuildGraph(scene);
        GoldenSnapshot.AssertGround(graph, scene.Name);
    }

    [Fact]
    public void Staircase_PathfindOracle()
    {
        var scene = SyntheticScenes.Staircase(0.4f);
        var graph = SceneHelper.BuildGraph(scene);
        float k = GeometryOracle.KForScene(scene.Name);

        foreach (var (from, to) in PairGen.Generate(scene.WalkablePoints, Seed, PairsPerScene))
        {
            if (from == to)
                continue;
            var path = SceneHelper.Pathfind(graph, from, to);
            if (path != null)
                GeometryOracle.AssertPathValid(scene, graph, path, from, to, k);
        }
    }

    [Fact]
    public void Staircase_GoldenSnapshot()
    {
        var scene = SyntheticScenes.Staircase(0.4f);
        var graph = SceneHelper.BuildGraph(scene);
        GoldenSnapshot.AssertGround(graph, scene.Name);
    }

    private static int CountSolidLeaves(VoxelMap volume)
    {
        int count = 0;
        CountSolidLeavesInTile(volume.RootTile, volume, ref count);
        return count;
    }

    private static void CountSolidLeavesInTile(VoxelMap.Tile tile, VoxelMap volume, ref int count)
    {
        bool isLeaf = tile.Level == volume.Levels.Length - 1;
        foreach (var data in tile.Contents)
        {
            if ((data & VoxelMap.VoxelOccupiedBit) == 0)
                continue;
            var subIdx = data & VoxelMap.VoxelIdMask;
            if (subIdx == VoxelMap.VoxelIdMask)
            {
                count++;
            }
            else if (!isLeaf)
            {
                CountSolidLeavesInTile(tile.Subdivision[subIdx], volume, ref count);
            }
        }
    }
}

[CollectionDefinition("BackendTests")]
public class BackendTestsCollection { }
