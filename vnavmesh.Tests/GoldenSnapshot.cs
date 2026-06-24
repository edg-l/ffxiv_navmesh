using Navmesh.GroundGraph;
using System;
using System.IO;
using Xunit;

namespace Navmesh.Tests;

public static class GoldenSnapshot
{
    private static readonly string GoldensDir = Path.Combine(
        Path.GetDirectoryName(typeof(GoldenSnapshot).Assembly.Location)!,
        "Goldens");

    public static void AssertGround(QuadGraph? graph, string sceneName)
    {
        var bytes = SerializeGround(graph);
        var goldenPath = Path.Combine(GoldensDir, $"{sceneName}.golden");

        if (Environment.GetEnvironmentVariable("UPDATE_GOLDENS") == "1")
        {
            Directory.CreateDirectory(GoldensDir);
            File.WriteAllBytes(goldenPath, bytes);
            return;
        }

        Assert.True(File.Exists(goldenPath),
            $"Golden file not found: {goldenPath}. Run with UPDATE_GOLDENS=1 to generate.");

        var expected = File.ReadAllBytes(goldenPath);
        Assert.Equal(expected.Length, bytes.Length);
        Assert.True(expected.AsSpan().SequenceEqual(bytes.AsSpan()),
            $"Golden snapshot mismatch for {sceneName}. Run with UPDATE_GOLDENS=1 to regenerate.");
    }

    private static byte[] SerializeGround(QuadGraph? graph)
    {
        using var ms = new MemoryStream();
        using var writer = new BinaryWriter(ms);
        Navmesh.SerializeGround(writer, graph);
        writer.Flush();
        return ms.ToArray();
    }
}
