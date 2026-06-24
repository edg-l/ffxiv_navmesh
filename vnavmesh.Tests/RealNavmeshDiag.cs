using Navmesh;
using Navmesh.GroundGraph;
using Navmesh.GroundGraph.Polyanya;
using System;
using System.IO;
using System.Numerics;
using System.Threading;
using Xunit;
using Xunit.Abstractions;

namespace Navmesh.Tests;

// TEMP diagnostic: load the real uploaded Limsa navmesh and reproduce the
// failing query from the in-game devlog. Remove with the TEMP-DEVLOG cleanup.
public class RealNavmeshDiag : IClassFixture<ServiceFixture>
{
    private readonly ITestOutputHelper _out;
    public RealNavmeshDiag(ITestOutputHelper o) => _out = o;

    private const string Path = "/home/edgar/ffxiv_sea_s1_twn_s1t2_level_s1t2__11F3A____0.navmesh";

    [Fact]
    public void ReproFailingQuery()
    {
        if (!File.Exists(Path)) return; // manual diagnostic: no-op when the sample file is absent
        using var stream = File.OpenRead(Path);
        using var reader = new BinaryReader(stream);
        var mesh = Navmesh.Deserialize(reader, 0);
        var g = mesh.Ground!;
        _out.WriteLine($"quads={g.Quads.Count} portals={g.Portals.Count} flags={g.Flags.Length}");

        var from = new Vector3(-175.7f, 18f, 31.4f);
        var to = new Vector3(-184.8f, 17f, 35.3f);

        int fq = g.NearestQuad(from, float.MaxValue, false);
        int tq = g.NearestQuad(to, float.MaxValue, false);
        _out.WriteLine($"fromQuad={fq} toQuad={tq}");
        if (fq >= 0) { var q = g.Quads[fq]; _out.WriteLine($"  fromQuad bounds X[{q.MinX:F1},{q.MaxX:F1}] Y[{q.MinY:F1},{q.MaxY:F1}] Z[{q.MinZ:F1},{q.MaxZ:F1}]"); }
        if (tq >= 0) { var q = g.Quads[tq]; _out.WriteLine($"  toQuad   bounds X[{q.MinX:F1},{q.MaxX:F1}] Y[{q.MinY:F1},{q.MaxY:F1}] Z[{q.MinZ:F1},{q.MaxZ:F1}]"); }

        const int UNREACH = QuadGraph.FLAG_UNREACHABLE;
        bool fReach = fq >= 0 && (g.Flags[fq] & UNREACH) == 0;
        bool tReach = tq >= 0 && (g.Flags[tq] & UNREACH) == 0;
        _out.WriteLine($"fromReachable={fReach} toReachable={tReach}");
        int reachCount = 0;
        for (int i = 0; i < g.Flags.Length; ++i) if ((g.Flags[i] & UNREACH) == 0) reachCount++;
        _out.WriteLine($"total reachable quads={reachCount}/{g.Quads.Count}");

        // Graph connectivity (ignoring FLAG_UNREACHABLE): BFS from fromQuad over
        // adjacency + portals. Is toQuad reachable in the raw graph at all?
        var seen = new System.Collections.Generic.HashSet<int>();
        var queue = new System.Collections.Generic.Queue<int>();
        if (fq >= 0) { seen.Add(fq); queue.Enqueue(fq); }
        while (queue.Count > 0)
        {
            var cur = queue.Dequeue();
            foreach (var n in g.Adjacency[cur]) if (seen.Add(n)) queue.Enqueue(n);
            foreach (var p in g.Portals) if (p.FromQuad == cur && seen.Add(p.ToQuad)) queue.Enqueue(p.ToQuad);
        }
        _out.WriteLine($"graph BFS from {fq}: reached {seen.Count} quads; toQuad {tq} reached={seen.Contains(tq)}");

        _out.WriteLine($"build MaxClimb={g.MaxClimb}");
        _out.WriteLine($"[as-loaded] portals={g.Portals.Count} adj[fq]={g.Adjacency[fq].Count}");

        // CRITICAL EXPERIMENT: rebuild adjacency with CURRENT code on the loaded
        // quads. If the graph reconnects, the cached file was built by old buggy
        // code and the user just needs /vnav rebuild.
        foreach (var climb in new[] { 3f, 4.5f, 6f, 8f, 12f })
        {
            g.BuildAdjacency(climb, 0f);
            int comps = CountComponents(g, out int largest, out int single);
            _out.WriteLine($"[climb={climb}] portals={g.Portals.Count} components={comps} largest={largest} singletons={single} fq~tq? {SameComp(g, fq, tq)}");
        }
        g.BuildAdjacency(3f, 0f);
        g.InitFlags();
        // Sample isolated singletons: why no neighbor? Look for any quad sharing
        // an exact edge in XZ (ignoring climb) and report the Y gap.
        {
            int shown = 0;
            for (int s = 0; s < g.Quads.Count && shown < 6; ++s)
            {
                if (g.Adjacency[s].Count != 0) continue;
                var A = g.Quads[s];
                int touchAnyY = 0; float minYGap = float.MaxValue;
                for (int b = 0; b < g.Quads.Count; ++b)
                {
                    if (b == s) continue;
                    var B = g.Quads[b];
                    bool zOv = MathF.Max(A.MinZ, B.MinZ) < MathF.Min(A.MaxZ, B.MaxZ);
                    bool xOv = MathF.Max(A.MinX, B.MinX) < MathF.Min(A.MaxX, B.MaxX);
                    bool exX = (MathF.Abs(A.MaxX - B.MinX) < 0.001f || MathF.Abs(A.MinX - B.MaxX) < 0.001f) && zOv;
                    bool exZ = (MathF.Abs(A.MaxZ - B.MinZ) < 0.001f || MathF.Abs(A.MinZ - B.MaxZ) < 0.001f) && xOv;
                    if (exX || exZ) { touchAnyY++; minYGap = MathF.Min(minYGap, MathF.Abs(A.MinY - B.MinY)); }
                }
                _out.WriteLine($"singleton {s}: X[{A.MinX:F1},{A.MaxX:F1}] Z[{A.MinZ:F1},{A.MaxZ:F1}] Y={A.MinY:F1} | exact-edge-neighbors(anyY)={touchAnyY} minYgap={(minYGap == float.MaxValue ? -1 : minYGap):F2}");
                shown++;
            }
        }
        // How many quads are geometrically edge-adjacent (touching border, XZ
        // overlap>0) to fromQuad within climb=3, regardless of whether a portal
        // exists? If many "should" connect but didn't, TryFindSharedEdge is buggy.
        {
            var A = g.Quads[fq];
            int shouldTouch = 0, exactTouch = 0;
            for (int b = 0; b < g.Quads.Count; ++b)
            {
                if (b == fq) continue;
                var B = g.Quads[b];
                if (MathF.Abs(A.MinY - B.MinY) > 3f) continue;
                bool zOv = MathF.Max(A.MinZ, B.MinZ) < MathF.Min(A.MaxZ, B.MaxZ);
                bool xOv = MathF.Max(A.MinX, B.MinX) < MathF.Min(A.MaxX, B.MaxX);
                // touching on a vertical edge (X) with Z overlap, or horizontal edge (Z) with X overlap
                bool edgeX = (MathF.Abs(A.MaxX - B.MinX) < 0.6f || MathF.Abs(A.MinX - B.MaxX) < 0.6f) && zOv;
                bool edgeZ = (MathF.Abs(A.MaxZ - B.MinZ) < 0.6f || MathF.Abs(A.MinZ - B.MaxZ) < 0.6f) && xOv;
                if (edgeX || edgeZ)
                {
                    shouldTouch++;
                    bool exX = (MathF.Abs(A.MaxX - B.MinX) < 0.001f || MathF.Abs(A.MinX - B.MaxX) < 0.001f) && zOv;
                    bool exZ = (MathF.Abs(A.MaxZ - B.MinZ) < 0.001f || MathF.Abs(A.MinZ - B.MaxZ) < 0.001f) && xOv;
                    if (exX || exZ) exactTouch++;
                    if (shouldTouch <= 8)
                        _out.WriteLine($"  near {b}: Y={B.MinY:F1} X[{B.MinX:F1},{B.MaxX:F1}] Z[{B.MinZ:F1},{B.MaxZ:F1}] exactEdge={exX || exZ}");
                }
            }
            _out.WriteLine($"fromQuad geom-adjacent(<0.6)={shouldTouch}, exact(<0.001)={exactTouch}, actual portals={g.Adjacency[fq].Count}");
        }
        // fromQuad's direct neighbors and their Y.
        _out.WriteLine($"quad {fq} neighbors:");
        foreach (var n in g.Adjacency[fq])
            _out.WriteLine($"  -> {n} Y={g.Quads[n].MinY:F1} X[{g.Quads[n].MinX:F0},{g.Quads[n].MaxX:F0}] Z[{g.Quads[n].MinZ:F0},{g.Quads[n].MaxZ:F0}]");

        // Connected-component size distribution over adjacency+portals.
        var comp = new int[g.Quads.Count];
        for (int i = 0; i < comp.Length; ++i) comp[i] = -1;
        int nComp = 0;
        var sizes = new System.Collections.Generic.List<int>();
        for (int s = 0; s < g.Quads.Count; ++s)
        {
            if (comp[s] >= 0) continue;
            int sz = 0; var q2 = new System.Collections.Generic.Queue<int>();
            comp[s] = nComp; q2.Enqueue(s);
            while (q2.Count > 0) { var c = q2.Dequeue(); sz++; foreach (var n in g.Adjacency[c]) if (comp[n] < 0) { comp[n] = nComp; q2.Enqueue(n); } foreach (var p in g.Portals) if (p.FromQuad == c && comp[p.ToQuad] < 0) { comp[p.ToQuad] = nComp; q2.Enqueue(p.ToQuad); } }
            sizes.Add(sz); nComp++;
        }
        sizes.Sort(); sizes.Reverse();
        _out.WriteLine($"components={nComp}; largest 10 sizes=[{string.Join(",", sizes.GetRange(0, Math.Min(10, sizes.Count)))}]");
        _out.WriteLine($"fromQuad comp size={sizes.Count}, comp[fq]={comp[fq]} comp[tq]={comp[tq]}");
        int singletons = 0; foreach (var z in sizes) if (z == 1) singletons++;
        _out.WriteLine($"singleton components={singletons}");

        var pm = PolyMesh.FromQuadGraph(g);
        _out.WriteLine($"polymesh faces={pm.Faces.Count} edges={pm.Edges.Count}");

        // Direct PolyanyaSearch with flags + quad hints, capture counters.
        var search = new PolyanyaSearch(pm);
        search.SetQuadFlags(g.Flags, UNREACH);
        var path = search.FindPath(from, to, fq, tq, 0f, CancellationToken.None);
        _out.WriteLine($"polyanya path={path.Count} expanded={search._expandedCount} goalPush={search._goalPushCount}");
        foreach (var w in path) _out.WriteLine($"  {w}");
    }

    private static int CountComponents(QuadGraph g, out int largest, out int singletons)
    {
        var comp = new int[g.Quads.Count];
        for (int i = 0; i < comp.Length; ++i) comp[i] = -1;
        int n = 0; largest = 0; singletons = 0;
        for (int s = 0; s < g.Quads.Count; ++s)
        {
            if (comp[s] >= 0) continue;
            int sz = 0; var q = new System.Collections.Generic.Queue<int>();
            comp[s] = n; q.Enqueue(s);
            while (q.Count > 0) { var c = q.Dequeue(); sz++; foreach (var nb in g.Adjacency[c]) if (comp[nb] < 0) { comp[nb] = n; q.Enqueue(nb); } }
            if (sz > largest) largest = sz;
            if (sz == 1) singletons++;
            n++;
        }
        return n;
    }

    private static bool SameComp(QuadGraph g, int a, int b)
    {
        var seen = new System.Collections.Generic.HashSet<int> { a };
        var q = new System.Collections.Generic.Queue<int>(); q.Enqueue(a);
        while (q.Count > 0) { var c = q.Dequeue(); if (c == b) return true; foreach (var nb in g.Adjacency[c]) if (seen.Add(nb)) q.Enqueue(nb); }
        return false;
    }
}
