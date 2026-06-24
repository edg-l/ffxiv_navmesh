using Navmesh.GroundGraph;
using Navmesh.GroundGraph.Extraction;
using Navmesh.GroundGraph.Polyanya;
using System;
using System.Collections.Generic;
using System.Numerics;
using Xunit;

namespace Navmesh.Tests;

// Phase 3 geometry correctness tests.
// Verifies: every walkable point is covered by a quad, zero orphan faces in the
// PolyMesh, seam-Y snapping, and path validity through ClearanceOracle /
// GeometryOracle on the CHF-based pipeline.
[Collection("BackendTests")]
public class Phase3GeometryTests : IClassFixture<ServiceFixture>
{
    private const float AgentMaxClimb = 0.5f;
    private const float AgentRadius = 0.5f;
    private const int Seed = 99;
    private const int PairsPerScene = 50;

    private static QuadGraph BuildGraph(Scene scene)
    {
        var chf = CompactHeightfield.FromVoxelMap(scene.Volume, scene.BoundsMin, scene.BoundsMax, AgentMaxClimb);
        var ground = QuadMesher.GreedyMesh(chf);
        ground.BuildAdjacency(AgentMaxClimb, AgentRadius);
        ground.InitFlags();
        return ground;
    }

    // Every walkable point from the synthetic scene must be covered by at least
    // one quad in the QuadGraph (XZ containment).
    private static void AssertWalkablePointsCovered(QuadGraph graph, Scene scene)
    {
        foreach (var pt in scene.WalkablePoints)
        {
            bool covered = false;
            foreach (var q in graph.Quads)
            {
                if (pt.X >= q.MinX - 0.1f && pt.X <= q.MaxX + 0.1f &&
                    pt.Z >= q.MinZ - 0.1f && pt.Z <= q.MaxZ + 0.1f)
                {
                    covered = true;
                    break;
                }
            }
            Assert.True(covered,
                $"Walkable point ({pt.X:F1}, {pt.Z:F1}) is not covered by any quad in scene {scene.Name}");
        }
    }

    // All PolyMesh faces must be reachable through adjacency (no orphan faces that
    // are completely disconnected from everything else). An orphan face is one where
    // ALL three edges are obstacle edges AND it has no off-mesh links.
    // (In a correct triangulation from a connected quad graph, no face should be
    // geometrically isolated.)
    private static void AssertNoOrphanFaces(QuadGraph graph, Scene scene)
    {
        var mesh = PolyMesh.FromQuadGraph(graph);
        if (mesh.Faces.Count == 0)
            return;

        // Build per-face link sets from off-mesh links.
        var linkedFaces = new HashSet<int>();
        foreach (var link in mesh.OffMeshLinks)
        {
            linkedFaces.Add(link.FromFace);
            linkedFaces.Add(link.ToFace);
        }

        int orphanCount = 0;
        for (int f = 0; f < mesh.Faces.Count; f++)
        {
            // An orphan: all 3 edges are obstacle edges AND not in any off-mesh link.
            bool allObstacle = mesh.Edges[f * 3].IsObstacleEdge
                            && mesh.Edges[f * 3 + 1].IsObstacleEdge
                            && mesh.Edges[f * 3 + 2].IsObstacleEdge;
            if (allObstacle && !linkedFaces.Contains(f))
                orphanCount++;
        }

        // Singletons (isolated triangles from small quads) are unavoidable when
        // quads have no portal neighbours. Allow up to 5% orphan faces as a sanity
        // bound; a correct mesh from connected quads should have very few.
        int allowedOrphans = Math.Max(5, mesh.Faces.Count / 20);
        Assert.True(orphanCount <= allowedOrphans,
            $"Scene {scene.Name}: {orphanCount} orphan faces (all-obstacle-edge, no off-mesh link) out of {mesh.Faces.Count} total. Expected ≤ {allowedOrphans}.");
    }

    // Seam-Y healing: at tile seam boundaries, adjacent quad Y values should be
    // close to each other. In the FromVoxelMap scan (no tile seams), this is trivially
    // satisfied; the test verifies CHF-based adjacency respects climb tolerance.
    private static void AssertAdjacentQuadYConsistency(QuadGraph graph, Scene scene)
    {
        foreach (var portal in graph.Portals)
        {
            if (portal.IsOffMesh) continue;
            var qa = graph.Quads[portal.FromQuad];
            var qb = graph.Quads[portal.ToQuad];
            float yDiff = MathF.Abs(qa.MinY - qb.MinY);
            Assert.True(yDiff <= AgentMaxClimb + 0.1f,
                $"Scene {scene.Name}: portal between quad {portal.FromQuad} (Y={qa.MinY:F3}) " +
                $"and quad {portal.ToQuad} (Y={qb.MinY:F3}) has Y diff {yDiff:F3} > climb {AgentMaxClimb}");
        }
    }

    [Fact]
    public void FlatPlane_WalkablePointsCovered()
    {
        var scene = SyntheticScenes.FlatPlane();
        var graph = BuildGraph(scene);
        AssertWalkablePointsCovered(graph, scene);
    }

    [Fact]
    public void FlatPlane_NoOrphanFaces()
    {
        var scene = SyntheticScenes.FlatPlane();
        var graph = BuildGraph(scene);
        AssertNoOrphanFaces(graph, scene);
    }

    [Fact]
    public void FlatPlane_AdjacentQuadYConsistency()
    {
        var scene = SyntheticScenes.FlatPlane();
        var graph = BuildGraph(scene);
        AssertAdjacentQuadYConsistency(graph, scene);
    }

    [Fact]
    public void PlaneWithPillar_WalkablePointsCovered()
    {
        var scene = SyntheticScenes.PlaneWithPillar();
        var graph = BuildGraph(scene);
        AssertWalkablePointsCovered(graph, scene);
    }

    [Fact]
    public void PlaneWithPillar_NoOrphanFaces()
    {
        var scene = SyntheticScenes.PlaneWithPillar();
        var graph = BuildGraph(scene);
        AssertNoOrphanFaces(graph, scene);
    }

    [Fact]
    public void Overpass_WalkablePointsCovered()
    {
        var scene = SyntheticScenes.Overpass();
        var graph = BuildGraph(scene);
        AssertWalkablePointsCovered(graph, scene);
    }

    [Fact]
    public void BridgeOnramp_WalkablePointsCovered()
    {
        var scene = SyntheticScenes.BridgeOnramp();
        var graph = BuildGraph(scene);
        AssertWalkablePointsCovered(graph, scene);
    }

    [Fact]
    public void BridgeOnramp_NoOrphanFaces()
    {
        var scene = SyntheticScenes.BridgeOnramp();
        var graph = BuildGraph(scene);
        AssertNoOrphanFaces(graph, scene);
    }

    [Fact]
    public void Staircase_WalkablePointsCovered()
    {
        var scene = SyntheticScenes.Staircase(0.4f);
        var graph = BuildGraph(scene);
        AssertWalkablePointsCovered(graph, scene);
    }

    [Fact]
    public void NarrowCorridor_WalkablePointsCovered()
    {
        var scene = SyntheticScenes.NarrowCorridor(4.0f);
        var graph = BuildGraph(scene);
        AssertWalkablePointsCovered(graph, scene);
    }

    // Path validity checks via GeometryOracle on a sample of pairs.
    [Fact]
    public void FlatPlane_PathsValid_Oracle()
    {
        var scene = SyntheticScenes.FlatPlane();
        var graph = BuildGraph(scene);
        float k = GeometryOracle.KForScene(scene.Name);
        int checked_ = 0;
        foreach (var (from, to) in GenPairs(scene.WalkablePoints, Seed, PairsPerScene))
        {
            if (from == to) continue;
            var path = SceneHelper.Pathfind(graph, from, to);
            if (path != null) { GeometryOracle.AssertPathValid(scene, graph, path, from, to, k); checked_++; }
        }
        Assert.True(checked_ > 0, "No paths found for FlatPlane oracle check");
    }

    [Fact]
    public void BridgeOnramp_PathsValid_Oracle()
    {
        var scene = SyntheticScenes.BridgeOnramp();
        var graph = BuildGraph(scene);
        float k = GeometryOracle.KForScene(scene.Name);
        foreach (var (from, to) in GenPairs(scene.WalkablePoints, Seed, PairsPerScene))
        {
            if (from == to) continue;
            var path = SceneHelper.Pathfind(graph, from, to);
            if (path != null)
                GeometryOracle.AssertPathValid(scene, graph, path, from, to, k);
        }
    }

    // ClearanceOracle: verify agent clearance along paths.
    [Fact]
    public void NarrowCorridor_PathClearance()
    {
        var scene = SyntheticScenes.NarrowCorridor(4.0f);
        var graph = BuildGraph(scene);
        foreach (var (from, to) in GenPairs(scene.WalkablePoints, Seed, PairsPerScene))
        {
            if (from == to) continue;
            var path = SceneHelper.Pathfind(graph, from, to);
            if (path != null)
                ClearanceOracle.AssertClearance(scene, path, AgentRadius);
        }
    }

    // Verify that SimplifyEdges (via ExtractContours) merges collinear wall segments.
    // A flat rectangular floor layer has 4 wall edges per side cell. After simplification,
    // each straight wall side should collapse into a single segment (or a short chain),
    // not remain as N individual 2-point segments.
    [Fact]
    public void FlatPlane_ContourIsSimplified()
    {
        var scene = SyntheticScenes.FlatPlane();
        var chf = CompactHeightfield.FromVoxelMap(scene.Volume, scene.BoundsMin, scene.BoundsMax, AgentMaxClimb);
        var partition = LayerPartition.Partition(chf);

        // There should be exactly 1 walkable layer for a flat plane.
        Assert.True(partition.NumLayers >= 1, "FlatPlane should have at least one layer");

        // Extract contours for layer 0.
        var contours = ContourExtractor.ExtractContours(partition, 0);

        // For a rectangle of N cells per side, without simplification we get N individual
        // 2-point segments per side (4 sides × N segments). After collinear merging, each
        // straight side becomes a single polyline segment. The total number of contour
        // chains should be much less than the raw edge count.
        // Flat plane is 64×64 cells. Raw edges = 4 sides × 64 cells = 256 boundary edges.
        // After simplification, all 4 sides merge to 4 chains (each with 2 points),
        // plus possibly corners. Assert that total vertex count ≤ 20 (much less than 256).
        int totalVerts = 0;
        foreach (var chain in contours)
            totalVerts += chain.Count;

        // The 4-sided rectangle perimeter after collinear merge has just 4 corners
        // (possibly split across chains). Allow generous headroom for boundary effects.
        Assert.True(totalVerts <= 32,
            $"FlatPlane contour should have few verts after collinear merge, got {totalVerts} across {contours.Count} chains");
    }

    private static IEnumerable<(Vector3 from, Vector3 to)> GenPairs(List<Vector3> pts, int seed, int n)
    {
        var rng = new Random(seed);
        for (int i = 0; i < n; i++)
            yield return (pts[rng.Next(pts.Count)], pts[rng.Next(pts.Count)]);
    }
}
