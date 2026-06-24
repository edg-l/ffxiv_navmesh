using Navmesh;
using Navmesh.GroundGraph;
using Navmesh.GroundGraph.Polyanya;
using System;
using System.Diagnostics;
using System.IO;
using System.Numerics;
using System.Threading;
using Xunit;
using Xunit.Abstractions;

namespace Navmesh.Tests;

// TEMP diagnostic: load the real uploaded Limsa navmesh and time PolyMesh build
// + a real pathfind. Remove with the TEMP-DEVLOG cleanup. No-op if file absent.
public class RealNavmeshDiag : IClassFixture<ServiceFixture>
{
    private readonly ITestOutputHelper _out;
    public RealNavmeshDiag(ITestOutputHelper o) => _out = o;

    private const string Path = "/home/edgar/ffxiv_sea_s1_twn_s1t2_level_s1t2__11F3A____0.navmesh";

    [Fact]
    public void ReproFailingQuery()
    {
        if (!File.Exists(Path)) return;
        using var stream = File.OpenRead(Path);
        using var reader = new BinaryReader(stream);
        Navmesh mesh;
        try { mesh = Navmesh.Deserialize(reader, 0); }
        catch (Exception ex) when (ex.Message.Contains("Incorrect header")) { _out.WriteLine("stale cache version; skip"); return; }
        var g = mesh.Ground!;

        int reach = 0;
        for (int i = 0; i < g.Flags.Length; ++i) if ((g.Flags[i] & QuadGraph.FLAG_UNREACHABLE) == 0) reach++;
        _out.WriteLine($"quads={g.Quads.Count} portals={g.Portals.Count} reachable={reach}");

        // Time the PolyMesh build (the O(quads*portals) hang was here).
        var sw = Stopwatch.StartNew();
        var pm = PolyMesh.FromQuadGraph(g);
        sw.Stop();
        _out.WriteLine($"FromQuadGraph: {sw.Elapsed.TotalMilliseconds:F0}ms -> faces={pm.Faces.Count} edges={pm.Edges.Count} offmesh={pm.OffMeshLinks.Count}");

        // The exact in-game failing query.
        var from = new Vector3(-175.01009f, 18f, 31.081926f);
        var to = new Vector3(-184.56314f, 17f, 34.816566f);
        int fq = g.NearestQuad(from, float.MaxValue, false);
        int tq = g.NearestQuad(to, float.MaxValue, false);
        _out.WriteLine($"fromQuad={fq} toQuad={tq}");

        sw.Restart();
        using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(30));
        var search = new PolyanyaSearch(pm);
        if (g.Flags.Length > 0) search.SetQuadFlags(g.Flags, QuadGraph.FLAG_UNREACHABLE);
        var path = search.FindPath(from, to, fq, tq, 0f, cts.Token);
        sw.Stop();
        _out.WriteLine($"Polyanya: {sw.Elapsed.TotalMilliseconds:F0}ms expanded={search._expandedCount} goalPush={search._goalPushCount} -> wps={path.Count}");
        if (path.Count > 0)
        {
            float len = 0; for (int i = 1; i < path.Count; ++i) len += Vector3.Distance(path[i - 1], path[i]);
            _out.WriteLine($"  path len={len:F1} straight={Vector3.Distance(from, to):F1} first={path[0]} last={path[^1]}");
        }
    }
}
