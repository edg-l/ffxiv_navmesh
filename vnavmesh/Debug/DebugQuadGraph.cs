using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using Navmesh.GroundGraph;
using Navmesh.Render;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace Navmesh.Debug;

public class DebugQuadGraph : IDisposable
{
    private static readonly Vector4 _colAreaNull = new(0, 0, 0, 0.25f);
    private static readonly Vector4 _colAreaWalkable = new(0, 0.75f, 1.0f, 0.5f);
    private static readonly Vector4 _colUnreachable = new(0.6f, 0.2f, 0.2f, 0.5f);
    private static readonly Vector4 _colOffMesh = new(1.0f, 0.5f, 0.0f, 0.5f);

    private enum InstanceID { AreaNull, AreaWalkable, Unreachable, OffMesh, Count }

    private QuadGraph _graph;
    private UITree _tree;
    private DebugDrawer _dd;
    private EffectMesh.Data? _visu;
    private int[] _quadMeshIndex = [];

    public DebugQuadGraph(QuadGraph graph, UITree tree, DebugDrawer dd)
    {
        _graph = graph;
        _tree = tree;
        _dd = dd;
    }

    public void Dispose()
    {
        _visu?.Dispose();
    }

    public void Draw()
    {
        using var nr = _tree.Node($"Quad graph ({_graph.Count} quads, {_graph.Portals.Count} portals)");
        if (!nr.Opened)
            return;

        _tree.LeafNode($"Bounds: [{_graph.BoundsMin}] - [{_graph.BoundsMax}], max climb: {_graph.MaxClimb:f3}");

        if (nr.SelectedOrHovered)
            VisualizeAllQuads();
        if (nr.Opened)
        {
            for (int i = 0; i < _graph.Quads.Count; ++i)
            {
                var q = _graph.Quads[i];
                var unreachable = i < _graph.Flags.Length && (_graph.Flags[i] & QuadGraph.FLAG_UNREACHABLE) != 0;
                var offMesh = q.Area.HasFlag(Navmesh.AreaId.Endpoint);
                using var nq = _tree.Node($"Quad {i}: [{q.MinX:f2},{q.MinY:f2},{q.MinZ:f2}]-[{q.MaxX:f2},{q.MaxY:f2},{q.MaxZ:f2}], area={(int)q.Area:X}{(unreachable ? " [unreachable]" : "")}{(offMesh ? " [offmesh]" : "")}###{i}");
                if (nq.SelectedOrHovered)
                    VisualizeQuad(i);
                if (nq.Opened)
                {
                    _tree.LeafNode($"Center: {q.Center:f3}");
                    if (i < _graph.Adjacency.Count)
                        _tree.LeafNode($"Neighbours: {string.Join(", ", _graph.Adjacency[i])}");
                }
            }

            using var np = _tree.Node($"Portals ({_graph.Portals.Count})###portals");
            if (np.SelectedOrHovered)
                VisualizeAllPortals();
            if (np.Opened)
            {
                for (int i = 0; i < _graph.Portals.Count; ++i)
                {
                    var p = _graph.Portals[i];
                    using var npn = _tree.Node($"Portal {i}: {p.FromQuad} -> {p.ToQuad}, y={p.YFrom:f2}->{p.YTo:f2}, offmesh={p.IsOffMesh}, area={(int)p.Area:X}###{i}", true);
                    if (npn.SelectedOrHovered)
                        VisualizePortal(i);
                }
            }
        }
    }

    private static Vector4 IntColor(int v, float a)
    {
        var mask = new BitMask((ulong)v);
        float r = (mask[1] ? 0.25f : 0) + (mask[3] ? 0.5f : 0) + 0.25f;
        float g = (mask[2] ? 0.25f : 0) + (mask[4] ? 0.5f : 0) + 0.25f;
        float b = (mask[0] ? 0.25f : 0) + (mask[5] ? 0.5f : 0) + 0.25f;
        return new(r, g, b, a);
    }

    private EffectMesh.Data GetOrInitVisualizer()
    {
        if (_visu != null)
            return _visu;
        _visu = new(_dd.RenderContext, _graph.Quads.Count * 4, _graph.Quads.Count * 2, (int)InstanceID.Count, false);
        using var builder = _visu.Map(_dd.RenderContext);

        builder.AddInstance(new(Matrix4x3.Identity, _colAreaNull));
        builder.AddInstance(new(Matrix4x3.Identity, _colAreaWalkable));
        builder.AddInstance(new(Matrix4x3.Identity, _colUnreachable));
        builder.AddInstance(new(Matrix4x3.Identity, _colOffMesh));

        _quadMeshIndex = new int[_graph.Quads.Count];
        int startPrim = 0;
        for (int i = 0; i < _graph.Quads.Count; ++i)
        {
            var q = _graph.Quads[i];
            float y = q.MinY;
            // 4 corner vertices in XZ plane at surface Y
            int v0 = builder.NumVertices;
            builder.AddVertex(new(q.MinX, y, q.MinZ));
            builder.AddVertex(new(q.MaxX, y, q.MinZ));
            builder.AddVertex(new(q.MaxX, y, q.MaxZ));
            builder.AddVertex(new(q.MinX, y, q.MaxZ));
            // 2 triangles (flipped for dx order)
            builder.AddTriangle(v0, v0 + 2, v0 + 1);
            builder.AddTriangle(v0, v0 + 3, v0 + 2);
            builder.AddMesh(0, startPrim, 2, 0, 0);
            _quadMeshIndex[i] = i;
            startPrim += 2;
        }
        return _visu;
    }

    private int QuadInstance(int i)
    {
        if (i < _graph.Flags.Length && (_graph.Flags[i] & QuadGraph.FLAG_UNREACHABLE) != 0)
            return (int)InstanceID.Unreachable;
        var q = _graph.Quads[i];
        return q.Area.HasFlag(Navmesh.AreaId.Endpoint) ? (int)InstanceID.OffMesh : q.Area == Navmesh.AreaId.None ? (int)InstanceID.AreaNull : (int)InstanceID.AreaWalkable;
    }

    private void VisualizeAllQuads()
    {
        if (_dd.EffectMesh == null)
            return;
        var visu = GetOrInitVisualizer();
        _dd.EffectMesh.Bind(_dd.RenderContext, false, false);
        visu.Bind(_dd.RenderContext);
        for (int i = 0; i < _graph.Quads.Count; ++i)
        {
            var mesh = visu.Meshes[i] with { FirstInstance = QuadInstance(i) };
            visu.DrawManual(_dd.RenderContext, mesh);
        }
    }

    private void VisualizeQuad(int i)
    {
        if (_dd.EffectMesh == null)
            return;
        var visu = GetOrInitVisualizer();
        _dd.EffectMesh.Bind(_dd.RenderContext, false, false);
        visu.Bind(_dd.RenderContext);
        var mesh = visu.Meshes[i] with { FirstInstance = QuadInstance(i) };
        visu.DrawManual(_dd.RenderContext, mesh);

        // draw edges + neighbors
        var q = _graph.Quads[i];
        var y = q.MinY;
        var c0 = new Vector3(q.MinX, y, q.MinZ);
        var c1 = new Vector3(q.MaxX, y, q.MinZ);
        var c2 = new Vector3(q.MaxX, y, q.MaxZ);
        var c3 = new Vector3(q.MinX, y, q.MaxZ);
        _dd.DrawWorldLine(c0, c1, 0xff000000, 3);
        _dd.DrawWorldLine(c1, c2, 0xff000000, 3);
        _dd.DrawWorldLine(c2, c3, 0xff000000, 3);
        _dd.DrawWorldLine(c3, c0, 0xff000000, 3);
    }

    private void VisualizeAllPortals()
    {
        for (int i = 0; i < _graph.Portals.Count; ++i)
            VisualizePortal(i);
    }

    private void VisualizePortal(int i)
    {
        var p = _graph.Portals[i];
        var from = p.FromQuad < _graph.Quads.Count ? _graph.Quads[p.FromQuad].Center : default;
        var to = p.ToQuad < _graph.Quads.Count ? _graph.Quads[p.ToQuad].Center : default;
        uint color = p.IsOffMesh ? 0xFFFF8000 : 0xFF00FF00;
        _dd.DrawWorldPointFilled(from, 4, color);
        _dd.DrawWorldLine(from, to, color, p.IsOffMesh ? 3 : 1);
        _dd.DrawWorldArrowPoint(from, to, 40, color, 2);
    }
}