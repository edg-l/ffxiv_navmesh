using Navmesh.GroundGraph.CDT;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using Xunit;
using Xunit.Abstractions;

namespace Navmesh.Tests;

// TEMP probe: does CDT scale to real-zone layer sizes (thousands of grid-aligned
// verts)? Reproduces the in-game "build stuck at 99%" hang offline.
public class CdtPerfProbe : IClassFixture<ServiceFixture>
{
    private readonly ITestOutputHelper _out;
    public CdtPerfProbe(ITestOutputHelper o) => _out = o;

    [Theory]
    [InlineData(500)]
    [InlineData(2000)]
    [InlineData(8000)]
    public void LargeGridAlignedLoop(int n)
    {
        // A big rectilinear "comb" loop: 2*n vertices on a 0.25 grid, the kind of
        // jagged grid-aligned contour a real zone layer produces.
        var verts = new List<Vector2>();
        for (int i = 0; i < n; i++)
        {
            float x = i * 0.25f;
            verts.Add(new(x, (i % 2) * 0.25f));      // bottom jagged edge
        }
        for (int i = n - 1; i >= 0; i--)
        {
            float x = i * 0.25f;
            verts.Add(new(x, 10f + (i % 2) * 0.25f)); // top jagged edge
        }
        int count = verts.Count;
        var constraints = new List<Cdt.Constraint>();
        var loop = new List<int>();
        for (int i = 0; i < count; i++)
        {
            loop.Add(i);
            constraints.Add(new Cdt.Constraint(i, (i + 1) % count));
        }
        var loops = new List<List<int>> { loop };

        var sw = Stopwatch.StartNew();
        var result = Cdt.Triangulate(verts, constraints, loops);
        sw.Stop();
        _out.WriteLine($"n={n} verts={count} -> {sw.Elapsed.TotalMilliseconds:F0}ms, tris={result?.Triangles.Count ?? -1}");
        Assert.NotNull(result);
    }
}
