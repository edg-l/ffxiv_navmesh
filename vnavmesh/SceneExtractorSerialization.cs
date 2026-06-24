using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using System;
using System.IO;
using System.IO.Compression;
using System.Numerics;

namespace Navmesh;

// Binary capture/replay format for an extracted SceneExtractor.
//
// Layout: [Magic u32][Version u32] then a Brotli-compressed payload that
// round-trips EVERY field of the in-memory scene: each mesh's name + MeshType,
// each MeshPart's vertices + primitives (with per-primitive flags + material),
// and each MeshInstance's id, material, world transform (Matrix4x3 Row0..Row3),
// world bounds (AABB Min/Max) and force set/clear primitive flag masks.
//
// This lets the full navmesh-creation pipeline (NavmeshBuilder) run offline from
// a file captured in-game, without re-extracting geometry from game memory.
public static class SceneExtractorSerialization
{
    public const uint Magic = 0x43534E56; // 'VNSC'
    public const uint Version = 1;

    public static void Serialize(SceneExtractor scene, Stream stream)
    {
        using var writer = new BinaryWriter(stream, System.Text.Encoding.UTF8, leaveOpen: true);
        writer.Write(Magic);
        writer.Write(Version);

        using var compressed = new BinaryWriter(new BrotliStream(stream, CompressionLevel.Optimal, true));
        compressed.Write(scene.Meshes.Count);
        foreach (var (name, mesh) in scene.Meshes)
        {
            compressed.Write(name);
            compressed.Write((int)mesh.MeshType);

            compressed.Write(mesh.Parts.Count);
            foreach (var part in mesh.Parts)
            {
                compressed.Write(part.Vertices.Count);
                foreach (var v in part.Vertices)
                    WriteVector3(compressed, v);

                compressed.Write(part.Primitives.Count);
                foreach (var p in part.Primitives)
                {
                    compressed.Write(p.V1);
                    compressed.Write(p.V2);
                    compressed.Write(p.V3);
                    compressed.Write((int)p.Flags);
                    compressed.Write(p.Material);
                }
            }

            compressed.Write(mesh.Instances.Count);
            foreach (var inst in mesh.Instances)
            {
                compressed.Write(inst.Id);
                compressed.Write(inst.Material);
                WriteMatrix4x3(compressed, inst.WorldTransform);
                WriteVector3(compressed, inst.WorldBounds.Min);
                WriteVector3(compressed, inst.WorldBounds.Max);
                compressed.Write((int)inst.ForceSetPrimFlags);
                compressed.Write((int)inst.ForceClearPrimFlags);
            }
        }
    }

    // throws on a bad header or truncated payload
    public static SceneExtractor Deserialize(Stream stream)
    {
        using var reader = new BinaryReader(stream, System.Text.Encoding.UTF8, leaveOpen: true);
        var magic = reader.ReadUInt32();
        var version = reader.ReadUInt32();
        if (magic != Magic)
            throw new InvalidDataException($"Not a scene capture file (magic {magic:X8} != {Magic:X8})");
        if (version != Version)
            throw new InvalidDataException($"Unsupported scene capture version {version} (expected {Version})");

        var scene = SceneExtractor.CreateEmpty();
        using var compressed = new BinaryReader(new BrotliStream(stream, CompressionMode.Decompress, true));
        var meshCount = compressed.ReadInt32();
        for (int m = 0; m < meshCount; ++m)
        {
            var name = compressed.ReadString();
            var mesh = new SceneExtractor.Mesh { MeshType = (SceneExtractor.MeshType)compressed.ReadInt32() };

            var partCount = compressed.ReadInt32();
            for (int pi = 0; pi < partCount; ++pi)
            {
                var part = new SceneExtractor.MeshPart();
                var vertCount = compressed.ReadInt32();
                for (int v = 0; v < vertCount; ++v)
                    part.Vertices.Add(ReadVector3(compressed));

                var primCount = compressed.ReadInt32();
                for (int p = 0; p < primCount; ++p)
                {
                    int v1 = compressed.ReadInt32(), v2 = compressed.ReadInt32(), v3 = compressed.ReadInt32();
                    var flags = (SceneExtractor.PrimitiveFlags)compressed.ReadInt32();
                    var material = compressed.ReadUInt64();
                    part.Primitives.Add(new(v1, v2, v3, flags, material));
                }
                mesh.Parts.Add(part);
            }

            var instCount = compressed.ReadInt32();
            for (int ii = 0; ii < instCount; ++ii)
            {
                var id = compressed.ReadUInt64();
                var material = compressed.ReadUInt64();
                var transform = ReadMatrix4x3(compressed);
                var bounds = new AABB { Min = ReadVector3(compressed), Max = ReadVector3(compressed) };
                var forceSet = (SceneExtractor.PrimitiveFlags)compressed.ReadInt32();
                var forceClear = (SceneExtractor.PrimitiveFlags)compressed.ReadInt32();
                mesh.Instances.Add(new SceneExtractor.MeshInstance(id, transform, bounds, material, forceSet, forceClear));
            }

            scene.Meshes[name] = mesh;
        }
        return scene;
    }

    public static void SerializeToFile(SceneExtractor scene, string path)
    {
        using var stream = File.Open(path, FileMode.Create, FileAccess.Write, FileShare.None);
        Serialize(scene, stream);
    }

    public static SceneExtractor DeserializeFromFile(string path)
    {
        using var stream = File.OpenRead(path);
        return Deserialize(stream);
    }

    private static void WriteVector3(BinaryWriter w, Vector3 v)
    {
        w.Write(v.X);
        w.Write(v.Y);
        w.Write(v.Z);
    }

    private static Vector3 ReadVector3(BinaryReader r) => new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());

    private static void WriteMatrix4x3(BinaryWriter w, Matrix4x3 m)
    {
        WriteVector3(w, m.Row0);
        WriteVector3(w, m.Row1);
        WriteVector3(w, m.Row2);
        WriteVector3(w, m.Row3);
    }

    private static Matrix4x3 ReadMatrix4x3(BinaryReader r) => new()
    {
        Row0 = ReadVector3(r),
        Row1 = ReadVector3(r),
        Row2 = ReadVector3(r),
        Row3 = ReadVector3(r),
    };
}
