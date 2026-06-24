using Navmesh.GroundGraph;
using Newtonsoft.Json;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Reflection;
using System.Threading;
using Xunit;

namespace Navmesh.Tests;

// Phase 2 task 2.4: capture funnel reference paths BEFORE task 2.3 deletes
// FunnelStringPull.cs. The fixture is a committed JSON file that
// PolyanyaVsFunnelTests loads to compare Polyanya path lengths against the
// old funnel backend's path lengths.
//
// This test class drives the CURRENT (pre-deletion) QuadGraph.Pathfind with
// useStringPulling=true on a FIXED seeded set of pairs per scene and writes
// the resulting waypoint lists to vnavmesh.Tests/Fixtures/funnel_reference.json.
//
// Run with CAPTURE_FUNNEL=1 to (re)generate the fixture; otherwise it just
// asserts the fixture file is present and well-formed.
public class FunnelFixtureCapture : IClassFixture<ServiceFixture>
{
    private const int PairsPerScene = 200;
    private const int Seed = 42;

    private static readonly string FixturesDir = Path.Combine(
        Path.GetDirectoryName(typeof(FunnelFixtureCapture).Assembly.Location)!,
        "Fixtures");

    private static readonly string FixturePath = Path.Combine(FixturesDir, "funnel_reference.json");

    // Source-tree Fixtures directory (for committing the captured fixture).
    // Resolved relative to the test project directory: walk up from the bin
    // output dir to find vnavmesh.Tests/Fixtures.
    private static readonly string SourceFixturesDir = ResolveSourceFixturesDir();

    private static string ResolveSourceFixturesDir()
    {
        // Assembly lives in vnavmesh.Tests/bin/<config>/net10.0-windows/.
        // Walk up four directories to reach the vnavmesh.Tests project root.
        var dir = new DirectoryInfo(Path.GetDirectoryName(typeof(FunnelFixtureCapture).Assembly.Location)!);
        for (int i = 0; i < 6 && dir != null; i++)
        {
            var candidate = Path.Combine(dir.FullName, "vnavmesh.Tests", "Fixtures");
            if (Directory.Exists(candidate) ||
                (dir.GetDirectories("vnavmesh.Tests").Length > 0))
            {
                Directory.CreateDirectory(candidate);
                return candidate;
            }
            dir = dir.Parent;
        }
        return FixturesDir;
    }

    private static readonly string SourceFixturePath = Path.Combine(SourceFixturesDir, "funnel_reference.json");

    private static IEnumerable<(string name, Scene scene)> Scenes()
    {
        yield return (nameof(SyntheticScenes.FlatPlane), SyntheticScenes.FlatPlane());
        yield return (nameof(SyntheticScenes.PlaneWithPillar), SyntheticScenes.PlaneWithPillar());
        yield return (nameof(SyntheticScenes.Overpass), SyntheticScenes.Overpass());
        yield return (nameof(SyntheticScenes.BridgeOnramp), SyntheticScenes.BridgeOnramp());
        yield return (nameof(SyntheticScenes.NarrowCorridor), SyntheticScenes.NarrowCorridor(4.0f));
        yield return (nameof(SyntheticScenes.Staircase), SyntheticScenes.Staircase(0.4f));
    }

    private static List<(Vector3 from, Vector3 to)> Pairs(Scene scene)
    {
        var rng = new System.Random(Seed);
        var pts = scene.WalkablePoints;
        var pairs = new List<(Vector3, Vector3)>(PairsPerScene);
        for (int i = 0; i < PairsPerScene; i++)
            pairs.Add((pts[rng.Next(pts.Count)], pts[rng.Next(pts.Count)]));
        return pairs;
    }

    [Fact]
    public void CaptureOrVerifyFunnelFixture()
    {
        if (System.Environment.GetEnvironmentVariable("CAPTURE_FUNNEL") == "1")
        {
            var root = new Dictionary<string, List<List<List<float>>>>();
            foreach (var (name, scene) in Scenes())
            {
                var graph = SceneHelper.BuildGraph(scene);
                var scenePaths = new List<List<List<float>>>();
                foreach (var (from, to) in Pairs(scene))
                {
                    var path = graph.Pathfind(from, to, false, true, 0, CancellationToken.None);
                    var serial = new List<List<float>>(path.Count);
                    foreach (var wp in path)
                        serial.Add(new List<float> { wp.X, wp.Y, wp.Z });
                    scenePaths.Add(serial);
                }
                root[name] = scenePaths;
            }
            Directory.CreateDirectory(FixturesDir);
            File.WriteAllText(FixturePath, JsonConvert.SerializeObject(root, Formatting.Indented));
            // Also write to the source tree so the fixture is committed.
            Directory.CreateDirectory(SourceFixturesDir);
            File.WriteAllText(SourceFixturePath, JsonConvert.SerializeObject(root, Formatting.Indented));
            return;
        }

        // Without CAPTURE_FUNNEL: assert the committed fixture is present and parses.
        Assert.True(File.Exists(FixturePath),
            $"Funnel reference fixture missing: {FixturePath}. Run with CAPTURE_FUNNEL=1 to generate.");
        var json = File.ReadAllText(FixturePath);
        var parsed = JsonConvert.DeserializeObject<Dictionary<string, List<List<List<float>>>>>(json);
        Assert.NotNull(parsed);
        Assert.Equal(6, parsed!.Count);
        foreach (var (name, _) in Scenes())
        {
            Assert.True(parsed.ContainsKey(name), $"fixture missing scene {name}");
            Assert.Equal(PairsPerScene, parsed[name].Count);
        }
    }
}