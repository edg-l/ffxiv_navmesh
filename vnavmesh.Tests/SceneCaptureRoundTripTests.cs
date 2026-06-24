using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using Xunit;

namespace Navmesh.Tests;

// Round-trip the SceneExtractor capture format on a synthetic hand-built scene
// and assert deep equality across every serialized field.
public class SceneCaptureRoundTripTests
{
    private static SceneExtractor BuildSynthetic()
    {
        var scene = SceneExtractor.CreateEmpty();

        // Mesh A: terrain with one part (two triangles) + two instances.
        var meshA = new SceneExtractor.Mesh { MeshType = SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.FileMesh };
        var partA = new SceneExtractor.MeshPart
        {
            Vertices = new List<Vector3>
            {
                new(0, 0, 0), new(1, 0, 0), new(1, 0, 1), new(0, 0, 1),
            },
            Primitives = new List<SceneExtractor.Primitive>
            {
                new(0, 1, 2, SceneExtractor.PrimitiveFlags.None, 0x1234),
                new(0, 2, 3, SceneExtractor.PrimitiveFlags.Unlandable | SceneExtractor.PrimitiveFlags.FlyThrough, 0xABCDEF),
            },
        };
        meshA.Parts.Add(partA);
        meshA.Instances.Add(new SceneExtractor.MeshInstance(
            0xDEAD,
            new Matrix4x3
            {
                Row0 = new(1, 2, 3),
                Row1 = new(4, 5, 6),
                Row2 = new(7, 8, 9),
                Row3 = new(10, 11, 12),
            },
            new AABB { Min = new(-1, -2, -3), Max = new(4, 5, 6) },
            0x55,
            SceneExtractor.PrimitiveFlags.ForceWalkable,
            SceneExtractor.PrimitiveFlags.Fishable));
        meshA.Instances.Add(new SceneExtractor.MeshInstance(
            0xBEEF,
            new Matrix4x3 { Row0 = new(0.5f, 0, 0), Row1 = new(0, 0.5f, 0), Row2 = new(0, 0, 0.5f), Row3 = new(100, 200, 300) },
            new AABB { Min = new(99, 199, 299), Max = new(101, 201, 301) },
            0,
            SceneExtractor.PrimitiveFlags.None,
            SceneExtractor.PrimitiveFlags.None));
        scene.Meshes["terrain/a.pcb"] = meshA;

        // Mesh B: analytic shape, two parts, no instances.
        var meshB = new SceneExtractor.Mesh { MeshType = SceneExtractor.MeshType.AnalyticShape };
        meshB.Parts.Add(new SceneExtractor.MeshPart
        {
            Vertices = new List<Vector3> { new(-1, -1, -1), new(1, 1, 1) },
            Primitives = new List<SceneExtractor.Primitive> { new(0, 1, 0, SceneExtractor.PrimitiveFlags.ForceUnwalkable) },
        });
        meshB.Parts.Add(new SceneExtractor.MeshPart()); // empty part
        scene.Meshes["<box>"] = meshB;

        // Mesh C: empty mesh, no parts/instances.
        scene.Meshes["empty"] = new SceneExtractor.Mesh { MeshType = SceneExtractor.MeshType.None };

        return scene;
    }

    [Fact]
    public void RoundTrip_PreservesAllFields()
    {
        var original = BuildSynthetic();

        using var ms = new MemoryStream();
        SceneExtractorSerialization.Serialize(original, ms);
        Assert.True(ms.Length > 8, "serialized stream should carry header + payload");

        ms.Position = 0;
        var restored = SceneExtractorSerialization.Deserialize(ms);

        AssertScenesEqual(original, restored);
    }

    [Fact]
    public void Deserialize_RejectsBadMagic()
    {
        using var ms = new MemoryStream();
        using (var w = new BinaryWriter(ms, System.Text.Encoding.UTF8, leaveOpen: true))
        {
            w.Write(0xDEADBEEFu);
            w.Write(SceneExtractorSerialization.Version);
        }
        ms.Position = 0;
        Assert.Throws<InvalidDataException>(() => SceneExtractorSerialization.Deserialize(ms));
    }

    private static void AssertScenesEqual(SceneExtractor a, SceneExtractor b)
    {
        Assert.Equal(a.Meshes.Count, b.Meshes.Count);
        foreach (var (name, meshA) in a.Meshes)
        {
            Assert.True(b.Meshes.TryGetValue(name, out var meshB), $"missing mesh '{name}'");
            Assert.Equal(meshA.MeshType, meshB!.MeshType);

            Assert.Equal(meshA.Parts.Count, meshB.Parts.Count);
            for (int p = 0; p < meshA.Parts.Count; ++p)
            {
                var pa = meshA.Parts[p];
                var pb = meshB.Parts[p];
                Assert.Equal(pa.Vertices, pb.Vertices);
                Assert.Equal(pa.Primitives, pb.Primitives);
            }

            Assert.Equal(meshA.Instances.Count, meshB.Instances.Count);
            for (int i = 0; i < meshA.Instances.Count; ++i)
            {
                var ia = meshA.Instances[i];
                var ib = meshB.Instances[i];
                Assert.Equal(ia.Id, ib.Id);
                Assert.Equal(ia.Material, ib.Material);
                Assert.Equal(ia.WorldTransform.Row0, ib.WorldTransform.Row0);
                Assert.Equal(ia.WorldTransform.Row1, ib.WorldTransform.Row1);
                Assert.Equal(ia.WorldTransform.Row2, ib.WorldTransform.Row2);
                Assert.Equal(ia.WorldTransform.Row3, ib.WorldTransform.Row3);
                Assert.Equal(ia.WorldBounds.Min, ib.WorldBounds.Min);
                Assert.Equal(ia.WorldBounds.Max, ib.WorldBounds.Max);
                Assert.Equal(ia.ForceSetPrimFlags, ib.ForceSetPrimFlags);
                Assert.Equal(ia.ForceClearPrimFlags, ib.ForceClearPrimFlags);
            }
        }
    }
}
