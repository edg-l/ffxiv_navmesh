using Navmesh.GroundGraph;
using System.IO;
using Xunit;
using Xunit.Abstractions;

namespace Navmesh.Tests;

// Offline build benchmark + creation regression: deserialize a captured .scene
// fixture and run the ENTIRE navmesh-creation pipeline offline (no game memory
// access), asserting it produces a usable Ground mesh, and printing per-stage
// timings. No-op when the fixture is absent so other devs / CI are unaffected.
public class SceneBuildHarnessTests : IClassFixture<ServiceFixture>
{
    private readonly ITestOutputHelper _out;
    public SceneBuildHarnessTests(ITestOutputHelper o) => _out = o;

    // Drop a captured scene here (see DebugNavmeshManager "[dev] Export scene":
    // it writes <configDir>/scenecapture/<cacheKey>.scene; copy one to this path).
    private const string FixturePath = "/home/edgar/limsa.scene";

    [Fact]
    public void OfflineBuild_FromCapturedScene_ProducesGround()
    {
        if (!File.Exists(FixturePath))
        {
            _out.WriteLine($"fixture absent ({FixturePath}); skipping offline build harness");
            return;
        }

        var scene = SceneExtractorSerialization.DeserializeFromFile(FixturePath);
        _out.WriteLine($"loaded captured scene: {scene.Meshes.Count} meshes");

        // The captured scene was already customized at capture time; the offline
        // ctor does NOT re-extract or re-customize, only reads Settings/NumTiles.
        var customization = NavmeshCustomizationRegistry.Default;
        var builder = new NavmeshBuilder(scene, customization);
        builder.BuildTiles();

        var timings = builder.LastBuildTimings;
        _out.WriteLine($"[benchmark] {timings}");

        Assert.NotNull(builder.Navmesh);
        var ground = builder.Navmesh.Ground;
        Assert.NotNull(ground);

        if (ground!.PrebuiltMesh != null)
        {
            int faces = ground.PrebuiltMesh.Faces.Count;
            _out.WriteLine($"CDT mesh: {faces} faces, {ground.PrebuiltMesh.Vertices.Count} verts");
            Assert.True(faces > 0, "expected > 0 CDT faces");
        }
        else
        {
            _out.WriteLine($"quad graph: {ground.Count} quads, {ground.Portals.Count} portals");
            Assert.True(ground.Count > 0, "expected > 0 quads");
        }
    }
}
