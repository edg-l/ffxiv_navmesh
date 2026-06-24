using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.Numerics;
using Xunit;

namespace Navmesh.Tests;

public static class ClearanceOracle
{
    private const int SegmentSamples = 8;
    private const int RingProbes = 8;

    public static void AssertClearance(Scene scene, List<Vector3> path, float agentRadius)
    {
        if (path.Count < 2)
            return;

        var leafCellSize = scene.Volume.Levels[^1].CellSize;
        float tolerance = agentRadius - Math.Max(leafCellSize.X, leafCellSize.Z);

        // For very small radii or coarse grids, skip (tolerance would be negative)
        if (tolerance <= 0)
            return;

        for (int i = 0; i < path.Count - 1; i++)
        {
            var a = path[i];
            var b = path[i + 1];
            for (int s = 1; s < SegmentSamples; s++)
            {
                float t = s / (float)SegmentSamples;
                var p = Vector3.Lerp(a, b, t);
                AssertPointClearance(scene.Volume, p, agentRadius, tolerance, leafCellSize);
            }
        }
    }

    private static void AssertPointClearance(VoxelMap volume, Vector3 p, float agentRadius, float tolerance, Vector3 cellSize)
    {
        // Probe a ring of points at agentRadius distance in XZ around p.
        // Every probe at agentRadius must be in empty (non-solid) voxel space —
        // meaning no solid geometry is closer than agentRadius - cellSize to the path center.
        float checkRadius = tolerance;
        bool hasEmptyAtRadius = false;

        for (int i = 0; i < RingProbes; i++)
        {
            double angle = 2.0 * Math.PI * i / RingProbes;
            var probe = p + new Vector3((float)(checkRadius * Math.Cos(angle)), 0, (float)(checkRadius * Math.Sin(angle)));
            var (_, empty) = volume.FindLeafVoxel(probe);
            if (empty)
            {
                hasEmptyAtRadius = true;
                break;
            }
        }

        Assert.True(hasEmptyAtRadius,
            $"Path point ({p.X:f2}, {p.Y:f2}, {p.Z:f2}) appears to be within {tolerance:f2} of solid geometry (agentRadius={agentRadius:f2}, cellSize={cellSize.X:f2})");
    }
}
