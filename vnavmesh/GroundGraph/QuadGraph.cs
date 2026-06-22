using System;
using System.Collections.Generic;
using System.Numerics;

namespace Navmesh.GroundGraph;

public class QuadGraph
{
    public List<Quad> Quads = [];
    public List<List<int>> Adjacency = [];
    public List<Portal> Portals = [];
    public Vector3 BoundsMin;
    public Vector3 BoundsMax;
    public float MaxClimb;

    public int Count => Quads.Count;

    public QuadGraph(Vector3 boundsMin, Vector3 boundsMax)
    {
        BoundsMin = boundsMin;
        BoundsMax = boundsMax;
    }

    public int AddQuad(Quad q)
    {
        var index = Quads.Count;
        Quads.Add(q);
        Adjacency.Add([]);
        return index;
    }

    public void BuildAdjacency(float maxClimb)
    {
        MaxClimb = maxClimb;
        for (int i = 0; i < Quads.Count; ++i)
            Adjacency[i].Clear();
        Portals.Clear();

        for (int a = 0; a < Quads.Count; ++a)
        {
            var qa = Quads[a];
            for (int b = a + 1; b < Quads.Count; ++b)
            {
                var qb = Quads[b];
                if (TryFindSharedEdge(qa, qb, maxClimb, out var spanMin, out var spanMax, out var yFrom, out var yTo))
                {
                    Adjacency[a].Add(b);
                    Adjacency[b].Add(a);
                    Portals.Add(new Portal(a, b, spanMin, spanMax, yFrom, yTo, false, Navmesh.AreaId.Default));
                }
            }
        }
    }

    private static bool TryFindSharedEdge(Quad a, Quad b, float maxClimb, out Vector2 spanMin, out Vector2 spanMax, out float yFrom, out float yTo)
    {
        spanMin = default;
        spanMax = default;
        yFrom = a.MinY;
        yTo = b.MinY;

        if (MathF.Abs(a.MinY - b.MinY) > maxClimb)
            return false;

        if (MathF.Abs(a.MaxX - b.MinX) < 0.001f)
        {
            var zMin = MathF.Max(a.MinZ, b.MinZ);
            var zMax = MathF.Min(a.MaxZ, b.MaxZ);
            if (zMin < zMax)
            {
                spanMin = new(a.MaxX, zMin);
                spanMax = new(a.MaxX, zMax);
                return true;
            }
        }
        else if (MathF.Abs(a.MinX - b.MaxX) < 0.001f)
        {
            var zMin = MathF.Max(a.MinZ, b.MinZ);
            var zMax = MathF.Min(a.MaxZ, b.MaxZ);
            if (zMin < zMax)
            {
                spanMin = new(a.MinX, zMin);
                spanMax = new(a.MinX, zMax);
                return true;
            }
        }
        else if (MathF.Abs(a.MaxZ - b.MinZ) < 0.001f)
        {
            var xMin = MathF.Max(a.MinX, b.MinX);
            var xMax = MathF.Min(a.MaxX, b.MaxX);
            if (xMin < xMax)
            {
                spanMin = new(xMin, a.MaxZ);
                spanMax = new(xMax, a.MaxZ);
                return true;
            }
        }
        else if (MathF.Abs(a.MinZ - b.MaxZ) < 0.001f)
        {
            var xMin = MathF.Max(a.MinX, b.MinX);
            var xMax = MathF.Min(a.MaxX, b.MaxX);
            if (xMin < xMax)
            {
                spanMin = new(xMin, a.MinZ);
                spanMax = new(xMax, a.MinZ);
                return true;
            }
        }

        return false;
    }
}

public readonly record struct Portal(int FromQuad, int ToQuad, Vector2 SpanMin, Vector2 SpanMax, float YFrom, float YTo, bool IsOffMesh, Navmesh.AreaId Area);