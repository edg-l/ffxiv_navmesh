using Navmesh.GroundGraph;
using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.Numerics;
using Xunit;

namespace Navmesh.Tests;

public static class GeometryOracle
{
    private const float EndpointEps = 0.05f;
    private const float AgentHeight = 2.0f;
    private const int SegmentSamples = 10;

    // Per-scene k values from PLAN3.md
    public static float KForScene(string sceneName) => sceneName switch
    {
        nameof(SyntheticScenes.FlatPlane) => 1.01f,
        nameof(SyntheticScenes.PlaneWithPillar) => 1.20f,
        nameof(SyntheticScenes.Staircase) => 1.30f,
        nameof(SyntheticScenes.BridgeOnramp) => 1.30f,
        nameof(SyntheticScenes.Overpass) => 1.40f,
        nameof(SyntheticScenes.NarrowCorridor) => 1.50f,
        _ => 2.0f,
    };

    public static void AssertPathValid(Scene scene, QuadGraph graph, List<Vector3> path, Vector3 from, Vector3 to, float k, bool assertStrictAnyAngle = false)
    {
        Assert.NotEmpty(path);

        // If from/to are on disconnected graph components, the pathfinder returns a
        // best-effort partial path (to nearest reachable node) + appends 'to'.
        // Detect this by checking graph reachability rather than asserting garbage.
        var fromQuad = graph.NearestQuad(from);
        var toQuad = graph.NearestQuad(to);
        if (fromQuad < 0 || toQuad < 0)
            return; // can't even locate quads; nothing to assert

        // BFS/flood from fromQuad; if toQuad not reachable, skip (unreachable pair).
        if (!IsReachable(graph, fromQuad, toQuad))
            return;

        // Final waypoint must equal 'to' within eps
        var last = path[^1];
        Assert.True(
            Vector3.Distance(last, to) <= EndpointEps,
            $"Last waypoint {last} not within {EndpointEps} of destination {to}");

        // Path length must not exceed k * straight-line distance
        float straightLine = Vector3.Distance(from, to);
        float pathLength = PathLength(path);
        if (straightLine > 0.001f)
        {
            Assert.True(
                pathLength <= k * straightLine + 0.1f,
                $"Path length {pathLength:f3} exceeds {k}x straight-line {straightLine:f3} (bound {k * straightLine:f3})");
        }

        // Each segment must pass through walkable quads in XZ projection
        for (int i = 0; i < path.Count - 1; i++)
        {
            var segA = path[i];
            var segB = path[i + 1];
            AssertSegmentThroughWalkableXZ(graph, segA, segB, i);
        }

        // Coarse not-underground sanity: sample points along path must not be underground
        // using VoxelSearch.LineOfSight to check that the point is in empty voxel space
        // (checked at surfaceY + AgentHeight*0.5 implicitly via finding empty voxel above)
        foreach (var wp in path)
        {
            var probePos = wp + new Vector3(0, AgentHeight * 0.5f, 0);
            var (voxel, empty) = scene.Volume.FindLeafVoxel(probePos);
            // A point well above the surface should be in empty space; if it is solid, the path is underground.
            // We accept if the voxel is invalid (out of bounds) since waypoints near scene edge may be clipped.
            if (voxel != VoxelMap.InvalidVoxel)
            {
                Assert.True(empty,
                    $"Waypoint {wp} appears underground: probe at {probePos} is in solid voxel {voxel:X}");
            }
        }

        if (assertStrictAnyAngle)
        {
            Assert.True(path.Count <= 2,
                $"Expected single-segment any-angle path (2 waypoints), got {path.Count}");
        }
    }

    private static float PathLength(List<Vector3> path)
    {
        float len = 0;
        for (int i = 0; i < path.Count - 1; i++)
            len += Vector3.Distance(path[i], path[i + 1]);
        return len;
    }

    private static void AssertSegmentThroughWalkableXZ(QuadGraph graph, Vector3 a, Vector3 b, int segIndex)
    {
        for (int s = 0; s <= SegmentSamples; s++)
        {
            float t = s / (float)SegmentSamples;
            var p = Vector3.Lerp(a, b, t);
            bool covered = false;
            foreach (var q in graph.Quads)
            {
                if (p.X >= q.MinX - 0.01f && p.X <= q.MaxX + 0.01f &&
                    p.Z >= q.MinZ - 0.01f && p.Z <= q.MaxZ + 0.01f)
                {
                    covered = true;
                    break;
                }
            }
            Assert.True(covered,
                $"Segment {segIndex}: point ({p.X:f2}, {p.Z:f2}) at t={t:f2} is not covered by any walkable quad");
        }
    }

    private static bool IsReachable(QuadGraph graph, int fromQuad, int toQuad)
    {
        if (fromQuad == toQuad)
            return true;
        var visited = new HashSet<int> { fromQuad };
        var queue = new Queue<int>();
        queue.Enqueue(fromQuad);
        while (queue.Count > 0)
        {
            var cur = queue.Dequeue();
            foreach (var neighbor in graph.Adjacency[cur])
            {
                if (neighbor == toQuad)
                    return true;
                if (visited.Add(neighbor))
                    queue.Enqueue(neighbor);
            }
            foreach (var portal in graph.Portals)
            {
                if (portal.IsOffMesh && portal.FromQuad == cur)
                {
                    if (portal.ToQuad == toQuad)
                        return true;
                    if (visited.Add(portal.ToQuad))
                        queue.Enqueue(portal.ToQuad);
                }
            }
        }
        return false;
    }
}
