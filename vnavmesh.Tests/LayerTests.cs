using Navmesh.GroundGraph;
using Navmesh.GroundGraph.Extraction;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Threading;
using Xunit;

namespace Navmesh.Tests;

// Layer topology tests (Phase 3). Each test builds a CHF from a synthetic VoxelMap
// scene and asserts the connectivity properties of the resulting QuadGraph.
[Collection("BackendTests")]
public class LayerTests : IClassFixture<ServiceFixture>
{
    private const float AgentMaxClimb = 0.5f;
    private const float AgentRadius = 0.5f;

    // Helper: count connected components of a QuadGraph (adjacency + off-mesh portals).
    private static int CountComponents(QuadGraph g, out int largestSize, out int singletonCount)
    {
        var comp = new int[g.Quads.Count];
        for (int i = 0; i < comp.Length; i++) comp[i] = -1;
        int n = 0; largestSize = 0; singletonCount = 0;
        for (int s = 0; s < g.Quads.Count; s++)
        {
            if (comp[s] >= 0) continue;
            int sz = 0;
            var queue = new Queue<int>();
            comp[s] = n; queue.Enqueue(s);
            while (queue.Count > 0)
            {
                var cur = queue.Dequeue(); sz++;
                foreach (var nb in g.Adjacency[cur])
                    if (comp[nb] < 0) { comp[nb] = n; queue.Enqueue(nb); }
                foreach (var p in g.Portals)
                    if (p.IsOffMesh && p.FromQuad == cur && comp[p.ToQuad] < 0)
                    { comp[p.ToQuad] = n; queue.Enqueue(p.ToQuad); }
            }
            if (sz > largestSize) largestSize = sz;
            if (sz == 1) singletonCount++;
            n++;
        }
        return n;
    }

    private static bool AreInSameComponent(QuadGraph g, int qa, int qb)
    {
        if (qa == qb) return true;
        var seen = new HashSet<int> { qa };
        var queue = new Queue<int>(); queue.Enqueue(qa);
        while (queue.Count > 0)
        {
            var cur = queue.Dequeue();
            if (cur == qb) return true;
            foreach (var nb in g.Adjacency[cur])
                if (seen.Add(nb)) queue.Enqueue(nb);
            foreach (var p in g.Portals)
                if (p.IsOffMesh && p.FromQuad == cur && seen.Add(p.ToQuad))
                    queue.Enqueue(p.ToQuad);
        }
        return false;
    }

    private static (QuadGraph graph, CompactHeightfield chf) BuildGraphAndChf(Scene scene)
    {
        var chf = CompactHeightfield.FromVoxelMap(scene.Volume, scene.BoundsMin, scene.BoundsMax, AgentMaxClimb);
        var ground = QuadMesher.GreedyMesh(chf);
        ground.BuildAdjacency(AgentMaxClimb, AgentRadius);
        ground.InitFlags();
        return (ground, chf);
    }

    private static QuadGraph BuildGraph(Scene scene) => BuildGraphAndChf(scene).graph;

    // Overpass: lower road and upper road are > climb apart in Y at the crossing.
    // They should end up as TWO disjoint layers with no link connecting them.
    [Fact]
    public void Overpass_IsDisjoint()
    {
        var scene = SyntheticScenes.Overpass();
        var (_, chf) = BuildGraphAndChf(scene);
        var partition = LayerPartition.Partition(chf);

        // The Overpass scene has a lower road (Y≈1.5) and upper road (Y≈7.5).
        // With AgentMaxClimb=0.5, they must be in different layers.
        // Find layers at the crossing XZ area.
        // Lower road: (0, lowerSurf, -10) — on the lower road.
        // Upper road: (−10, upperSurf, 0) — on the upper road.
        float lowerSurf = 1.5f; // FloorThickness + WalkableYOffset = 1.5
        float upperSurf = 7.5f; // 6 + 1.5

        // Find which CHF cell contains lower/upper road points.
        int lowerX = chf.WorldToCell_X(0);
        int lowerZ = chf.WorldToCell_Z(-10);
        int upperX = chf.WorldToCell_X(-10);
        int upperZ = chf.WorldToCell_Z(0);

        // Get layers at these cells.
        int lowerLayer = -1, upperLayer = -1;
        var lowerSpans = chf.GetSpans(lowerX, lowerZ);
        for (int si = 0; si < lowerSpans.Count; si++)
        {
            int lay = partition.GetLayer(lowerX, lowerZ, si);
            if (lay >= 0) { lowerLayer = lay; break; }
        }
        var upperSpans = chf.GetSpans(upperX, upperZ);
        for (int si = 0; si < upperSpans.Count; si++)
        {
            int lay = partition.GetLayer(upperX, upperZ, si);
            if (lay >= 0) { upperLayer = lay; break; }
        }

        Assert.True(lowerLayer >= 0, "Lower road cell has no walkable layer");
        Assert.True(upperLayer >= 0, "Upper road cell has no walkable layer");
        Assert.NotEqual(lowerLayer, upperLayer);

        // Graph connectivity: the two roads must be in separate components.
        var graph = BuildGraph(scene);
        var lowerPoint = new Vector3(0, lowerSurf, -10);
        var upperPoint = new Vector3(-10, upperSurf, 0);
        int lowerQuad = graph.NearestQuad(lowerPoint);
        int upperQuad = graph.NearestQuad(upperPoint);
        Assert.True(lowerQuad >= 0, "No quad found near lower road");
        Assert.True(upperQuad >= 0, "No quad found near upper road");
        Assert.False(AreInSameComponent(graph, lowerQuad, upperQuad),
            "Overpass lower and upper roads must be in SEPARATE connected components");
    }

    // BridgeOnramp: a ramp with two steps (each within climb threshold) connects
    // the main deck (Y=1) to the bridge deck (Y=1.625). The whole thing must be ONE
    // connected component.
    [Fact]
    public void BridgeOnramp_IsLinkConnected()
    {
        var scene = SyntheticScenes.BridgeOnramp();
        var graph = BuildGraph(scene);

        // Main deck point and bridge deck point.
        // FloorThickness=1.0, bridge step2 top = FloorThickness+0.625=1.625, WalkableYOffset=0.5
        float mainSurf = 1.5f;   // FloorThickness(1) + WalkableYOffset(0.5)
        float bridgeSurf = 2.125f; // FloorThickness+0.625 + WalkableYOffset

        var mainPoint = new Vector3(-15, mainSurf, 0);
        var bridgePoint = new Vector3(16, bridgeSurf, 0);

        int mainQuad = graph.NearestQuad(mainPoint);
        int bridgeQuad = graph.NearestQuad(bridgePoint);
        Assert.True(mainQuad >= 0, "No quad found near main deck");
        Assert.True(bridgeQuad >= 0, "No quad found near bridge deck");
        Assert.True(AreInSameComponent(graph, mainQuad, bridgeQuad),
            "BridgeOnramp main deck and bridge deck must be in the SAME connected component");
    }

    // Staircase with steps < climb threshold: all steps must be in ONE component.
    [Fact]
    public void Staircase_WithinClimb_IsOneComponent()
    {
        // Step height 0.3125 (= one leaf voxel height) quantizes to diff 0.3125 < AgentMaxClimb 0.5
        // → all steps within climb → one component.
        var scene = SyntheticScenes.Staircase(0.3125f);
        var graph = BuildGraph(scene);

        int components = CountComponents(graph, out int largest, out int singletons);
        // All walkable cells in a single staircase should form one connected component.
        // (Singletons from non-staircase area cells are OK to ignore; the walkable
        // staircase cells should connect.)
        // Assert: the walkable staircase points are all in the same component.
        var points = scene.WalkablePoints;
        if (points.Count < 2) return;

        int firstQuad = graph.NearestQuad(points[0]);
        Assert.True(firstQuad >= 0);
        for (int i = 1; i < points.Count; i++)
        {
            int q = graph.NearestQuad(points[i]);
            if (q < 0) continue;
            Assert.True(AreInSameComponent(graph, firstQuad, q),
                $"Staircase step {i} (Y≈{points[i].Y:F1}) is disconnected from step 0");
        }
    }

    // Staircase with step height > climb threshold: steps must be DISCONNECTED
    // (each is its own layer) — over-climb = wall.
    [Fact]
    public void Staircase_OverClimb_IsDisconnected()
    {
        // Step height 1.0 >> AgentMaxClimb 0.5 → over-climb → each step is a separate layer.
        var scene = SyntheticScenes.Staircase(1.0f);
        var graph = BuildGraph(scene);

        var points = scene.WalkablePoints;
        if (points.Count < 2) return;

        int firstQuad = graph.NearestQuad(points[0]);
        int lastQuad = graph.NearestQuad(points[^1]);
        if (firstQuad < 0 || lastQuad < 0) return;

        Assert.False(AreInSameComponent(graph, firstQuad, lastQuad),
            "Staircase with over-climb steps: first and last steps must be DISCONNECTED");
    }

    // Task 3.8 round-trip: serialize a QuadGraph with off-mesh links and verify it
    // deserializes to the same data. Also asserts Navmesh.Version == 29 (Phase 4).
    [Fact]
    public void NavmeshVersion_Is29()
    {
        Assert.Equal(29u, Navmesh.Version);
    }

    [Fact]
    public void RoundTrip_OffMeshLinks()
    {
        // Build a BridgeOnramp graph (has inter-layer links as off-mesh portals).
        var scene = SyntheticScenes.BridgeOnramp();
        var graph = BuildGraph(scene);

        int offMeshCount = 0;
        foreach (var p in graph.Portals)
            if (p.IsOffMesh) offMeshCount++;

        // Serialize then deserialize.
        var navmesh = new Navmesh(0, graph, null);
        byte[] bytes;
        using (var ms = new MemoryStream())
        using (var writer = new BinaryWriter(ms))
        {
            navmesh.Serialize(writer);
            writer.Flush();
            bytes = ms.ToArray();
        }

        using (var ms = new MemoryStream(bytes))
        using (var reader = new BinaryReader(ms))
        {
            var loaded = Navmesh.Deserialize(reader, 0);
            Assert.NotNull(loaded.Ground);
            Assert.Equal(graph.Quads.Count, loaded.Ground!.Quads.Count);
            Assert.Equal(graph.Portals.Count, loaded.Ground.Portals.Count);
            Assert.Equal(graph.Flags.Length, loaded.Ground.Flags.Length);

            int loadedOffMesh = 0;
            foreach (var p in loaded.Ground.Portals)
                if (p.IsOffMesh) loadedOffMesh++;
            Assert.Equal(offMeshCount, loadedOffMesh);
        }
    }
}
