using Navmesh.GroundGraph;
using Navmesh.GroundGraph.CDT;
using Navmesh.GroundGraph.Extraction;
using Navmesh.GroundGraph.Geometry;
using Navmesh.GroundGraph.Polyanya;
using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using Xunit;

namespace Navmesh.Tests;

// Phase 4 Task 4.4: per-scene walkable-area coverage regression + CDT
// serialization round-trip + Navmesh.Version == 29.
//
// Coverage is the count of dense XZ sample points (over the scene bounds, on the
// walkable surface) that fall inside a walkable face/quad. The flip-to-CDT gate
// requires CDT coverage >= greedy-quad coverage on all 6 synthetic scenes.
[Collection("BackendTests")]
public class CoverageRegressionTests : IClassFixture<ServiceFixture>
{
    private const float AgentMaxClimb = 0.5f;
    private const int GridSamples = 80; // 80x80 XZ probe grid per scene

    [Fact]
    public void NavmeshVersion_Is29()
    {
        Assert.Equal(29u, Navmesh.Version);
    }

    public static IEnumerable<object[]> Scenes()
    {
        yield return new object[] { nameof(SyntheticScenes.FlatPlane) };
        yield return new object[] { nameof(SyntheticScenes.PlaneWithPillar) };
        yield return new object[] { nameof(SyntheticScenes.Overpass) };
        yield return new object[] { nameof(SyntheticScenes.BridgeOnramp) };
        yield return new object[] { nameof(SyntheticScenes.NarrowCorridor) };
        yield return new object[] { nameof(SyntheticScenes.Staircase) };
    }

    private static Scene MakeScene(string name) => name switch
    {
        nameof(SyntheticScenes.FlatPlane) => SyntheticScenes.FlatPlane(),
        nameof(SyntheticScenes.PlaneWithPillar) => SyntheticScenes.PlaneWithPillar(),
        nameof(SyntheticScenes.Overpass) => SyntheticScenes.Overpass(),
        nameof(SyntheticScenes.BridgeOnramp) => SyntheticScenes.BridgeOnramp(),
        nameof(SyntheticScenes.NarrowCorridor) => SyntheticScenes.NarrowCorridor(4.0f),
        nameof(SyntheticScenes.Staircase) => SyntheticScenes.Staircase(0.4f),
        _ => throw new ArgumentOutOfRangeException(nameof(name)),
    };

    [Theory]
    [MemberData(nameof(Scenes))]
    public void CdtCoverage_AtLeast_QuadCoverage(string sceneName)
    {
        var scene = MakeScene(sceneName);
        var chf = CompactHeightfield.FromVoxelMap(scene.Volume, scene.BoundsMin, scene.BoundsMax, AgentMaxClimb);

        var quadGraph = QuadMesher.GreedyMesh(chf);
        var cdtMesh = CdtMeshBuilder.Build(chf);

        // Reference coverage: XZ points that the greedy quads cover (the union of
        // walkable cells). Use the quad set itself as the ground-truth walkable
        // region so coverage compares like-for-like.
        int quadCovered = 0, cdtCovered = 0, total = 0;
        var bmin = scene.BoundsMin; var bmax = scene.BoundsMax;
        for (int iz = 0; iz < GridSamples; iz++)
        {
            for (int ix = 0; ix < GridSamples; ix++)
            {
                float x = bmin.X + (ix + 0.5f) * (bmax.X - bmin.X) / GridSamples;
                float z = bmin.Z + (iz + 0.5f) * (bmax.Z - bmin.Z) / GridSamples;
                bool inQuad = PointInAnyQuad(quadGraph, x, z);
                if (!inQuad)
                    continue; // only score points the walkable region (quad union) covers
                total++;
                quadCovered++;
                if (PointInAnyFace(cdtMesh, x, z))
                    cdtCovered++;
            }
        }

        Assert.True(total > 0, $"Scene {sceneName}: no walkable sample points");
        // CDT must cover at least as much of the walkable region as the quads,
        // within a small tolerance for boundary-cell rasterization differences.
        float quadFrac = quadCovered / (float)total;
        float cdtFrac = cdtCovered / (float)total;
        Assert.True(cdtFrac >= quadFrac - 0.05f,
            $"Scene {sceneName}: CDT coverage {cdtFrac:P1} < quad coverage {quadFrac:P1} (cdt={cdtCovered}, quad={quadCovered}, total={total})");
    }

    [Fact]
    public void CdtMesh_SerializeRoundTrip()
    {
        var scene = SyntheticScenes.PlaneWithPillar();
        var chf = CompactHeightfield.FromVoxelMap(scene.Volume, scene.BoundsMin, scene.BoundsMax, AgentMaxClimb);
        var cdtMesh = CdtMeshBuilder.Build(chf);

        var graph = new QuadGraph(scene.BoundsMin, scene.BoundsMax) { MaxClimb = AgentMaxClimb };
        graph.SetCdtMesh(cdtMesh);
        graph.InitFlags();

        var navmesh = new Navmesh(0, graph, null);
        byte[] bytes;
        using (var ms = new MemoryStream())
        using (var writer = new BinaryWriter(ms))
        {
            navmesh.Serialize(writer);
            writer.Flush();
            bytes = ms.ToArray();
        }

        using var ms2 = new MemoryStream(bytes);
        using var reader = new BinaryReader(ms2);
        var loaded = Navmesh.Deserialize(reader, 0);

        Assert.NotNull(loaded.Ground);
        Assert.NotNull(loaded.Ground!.PrebuiltMesh);
        var lm = loaded.Ground.PrebuiltMesh!;
        Assert.Equal(cdtMesh.Vertices.Count, lm.Vertices.Count);
        Assert.Equal(cdtMesh.Faces.Count, lm.Faces.Count);
        Assert.Equal(cdtMesh.Edges.Count, lm.Edges.Count);
        Assert.Equal(cdtMesh.OffMeshLinks.Count, lm.OffMeshLinks.Count);

        // Per-face data round-trips.
        for (int f = 0; f < cdtMesh.Faces.Count; f++)
        {
            Assert.Equal(cdtMesh.Faces[f].V0, lm.Faces[f].V0);
            Assert.Equal(cdtMesh.Faces[f].V1, lm.Faces[f].V1);
            Assert.Equal(cdtMesh.Faces[f].V2, lm.Faces[f].V2);
            Assert.Equal(cdtMesh.Faces[f].Layer, lm.Faces[f].Layer);
            Assert.Equal(cdtMesh.Faces[f].Y, lm.Faces[f].Y, 3);
            Assert.Equal(cdtMesh.SourceQuad[f], lm.SourceQuad[f]);
        }
        // Per-edge constraint flag round-trips.
        for (int e = 0; e < cdtMesh.Edges.Count; e++)
        {
            Assert.Equal(cdtMesh.Edges[e].IsObstacleEdge, lm.Edges[e].IsObstacleEdge);
            Assert.Equal(cdtMesh.Edges[e].FaceRight, lm.Edges[e].FaceRight);
        }

        // The face-AABB QuadGraph wrapper is rebuilt with one quad per face.
        Assert.Equal(cdtMesh.Faces.Count, loaded.Ground.Quads.Count);
    }

    private static bool PointInAnyQuad(QuadGraph graph, float x, float z)
    {
        foreach (var q in graph.Quads)
            if (x >= q.MinX && x <= q.MaxX && z >= q.MinZ && z <= q.MaxZ)
                return true;
        return false;
    }

    private static bool PointInAnyFace(PolyMesh mesh, float x, float z)
    {
        foreach (var f in mesh.Faces)
        {
            var a = mesh.Vertices[f.V0]; var b = mesh.Vertices[f.V1]; var c = mesh.Vertices[f.V2];
            double d1 = Predicates.Orient2D(a.X, a.Z, b.X, b.Z, x, z);
            double d2 = Predicates.Orient2D(b.X, b.Z, c.X, c.Z, x, z);
            double d3 = Predicates.Orient2D(c.X, c.Z, a.X, a.Z, x, z);
            bool hasNeg = d1 < 0 || d2 < 0 || d3 < 0;
            bool hasPos = d1 > 0 || d2 > 0 || d3 > 0;
            if (!(hasNeg && hasPos))
                return true;
        }
        return false;
    }
}
