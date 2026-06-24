using Navmesh.GroundGraph;
using System;
using System.Numerics;
using System.Threading;
using Xunit;

namespace Navmesh.Tests;

// Phase 2 task 2.4: verify range semantics are preserved through the Polyanya
// backend. range>0 terminates the search early when a node's closest position
// is within `range` of the goal; range==0 = exact goal. On FlatPlane, a
// range=3.0 path must (a) end within 3.0 of `to` and (b) be shorter than the
// range=0 (exact) path, because it stops early.
public class RangeSemanticsTests : IClassFixture<ServiceFixture>
{
    private const float Eps = 0.05f;

    [Fact]
    public void FlatPlane_Range3_LastWaypointWithinRangeAndShorterThanExact()
    {
        var scene = SyntheticScenes.FlatPlane();
        var graph = SceneHelper.BuildGraph(scene);
        var from = new Vector3(-15f, scene.WalkablePoints[0].Y, -15f);
        var to = new Vector3(15f, scene.WalkablePoints[^1].Y, 15f);

        var exactPath = graph.Pathfind(from, to, false, true, 0f, CancellationToken.None);
        var rangePath = graph.Pathfind(from, to, false, true, 3.0f, CancellationToken.None);

        Assert.NotEmpty(exactPath);
        Assert.NotEmpty(rangePath);

        // range=0 exact path: last waypoint must equal `to`.
        Assert.True(Vector3.Distance(exactPath[^1], to) <= Eps,
            $"exact path last waypoint {exactPath[^1]} not at to={to}");

        // range=3 path: last waypoint must be within 3.0 of `to`.
        var last = rangePath[^1];
        Assert.True(Vector3.Distance(last, to) <= 3.0f + Eps,
            $"range=3 path last waypoint {last} is {Vector3.Distance(last, to)} from to={to}, expected <= 3.0");

        // range=3 path must be shorter than the exact path (it stops early).
        float exactLen = PathLength(exactPath);
        float rangeLen = PathLength(rangePath);
        Assert.True(rangeLen < exactLen,
            $"range=3 path length {rangeLen} should be < exact path length {exactLen}");
    }

    [Fact]
    public void FlatPlane_Range0_ExactGoal()
    {
        var scene = SyntheticScenes.FlatPlane();
        var graph = SceneHelper.BuildGraph(scene);
        var from = new Vector3(-15f, scene.WalkablePoints[0].Y, -15f);
        var to = new Vector3(15f, scene.WalkablePoints[^1].Y, 15f);

        var path = graph.Pathfind(from, to, false, true, 0f, CancellationToken.None);
        Assert.NotEmpty(path);
        Assert.True(Vector3.Distance(path[^1], to) <= Eps,
            $"range=0 path last waypoint {path[^1]} not at to={to}");
    }

    private static float PathLength(System.Collections.Generic.List<Vector3> path)
    {
        float len = 0;
        for (int i = 0; i < path.Count - 1; i++)
            len += Vector3.Distance(path[i], path[i + 1]);
        return len;
    }
}