using System;
using System.Collections.Generic;
using System.Numerics;
using Navmesh.GroundGraph.Extraction;
using Navmesh.GroundGraph.Geometry;
using Navmesh.GroundGraph.Polyanya;

namespace Navmesh.GroundGraph.CDT;

// Builds a triangle PolyMesh (the type PolyanyaSearch consumes) from Phase-3
// layer contours via per-layer Constrained Delaunay Triangulation. This is the
// Config.UseCdtMesh==true ground backend, replacing the greedy-quad path.
//
// Per layer:
//   1. ExtractContours yields collinear-merged WALL boundary segments as
//      individual polylines. Assemble those into closed polygon loops via an
//      endpoint adjacency map, classify the outer boundary vs holes by signed
//      area / containment.
//   2. Run Cdt.Triangulate over the loop vertices + the loop edges as
//      constraints.
//   3. Append the resulting CCW triangles as faces (carrying the layer's floor-Y
//      onto TriFace.Y, layer id onto SourceQuad/Layer), with bilateral TriEdge
//      adjacency and faceRight=-1 on constrained/boundary edges.
//
// Inter-layer links (ExtractInterLayerLinks) become OffMeshLinks; their endpoint
// faces are resolved against the triangulated layers.
public static class CdtMeshBuilder
{
    // Snap tolerance for welding contour vertices that are meant to coincide.
    private const float WeldEps = 1e-3f;

    // Build the combined PolyMesh from a merged CompactHeightfield. Mirrors the
    // greedy path's MeshInto: partition into layers, mesh each, wire inter-layer
    // links.
    public static PolyMesh Build(CompactHeightfield chf, System.Threading.CancellationToken cancel = default)
    {
        var partition = LayerPartition.Partition(chf);
        var mesh = new PolyMesh();
        // Per layer: the list of face indices it produced (for link endpoint
        // resolution + the FacesByQuad index, which we key by layer id).
        var facesByLayer = new List<int>[partition.NumLayers];
        for (int i = 0; i < facesByLayer.Length; i++)
            facesByLayer[i] = new List<int>();
        // Per layer: representative floor-Y (median of the layer's span floors).
        var layerY = new float[partition.NumLayers];

        for (int layerId = 0; layerId < partition.NumLayers; layerId++)
        {
            cancel.ThrowIfCancellationRequested();
            float floorY = LayerFloorY(partition, layerId);
            layerY[layerId] = floorY;
            BuildLayer(chf, partition, layerId, floorY, mesh, facesByLayer[layerId]);
        }

        // Wire triangle adjacency across the whole mesh (shared non-constrained
        // edges become bilateral interior edges).
        WireAdjacency(mesh);

        // Off-mesh links from inter-layer seams.
        var links = LinkExtractor.ExtractInterLayerLinks(partition);
        foreach (var link in links)
        {
            int fromFace = FindFaceForLayerPoint(mesh, facesByLayer, link.LayerA, link.PosA);
            int toFace = FindFaceForLayerPoint(mesh, facesByLayer, link.LayerB, link.PosB);
            if (fromFace < 0 || toFace < 0)
                continue;
            mesh.OffMeshLinks.Add(new OffMeshLink(fromFace, toFace, link.PosA, link.PosB, Navmesh.AreaId.Shortcut | Navmesh.AreaId.Endpoint));
            mesh.OffMeshLinks.Add(new OffMeshLink(toFace, fromFace, link.PosB, link.PosA, Navmesh.AreaId.Shortcut | Navmesh.AreaId.Endpoint));
        }

        // FacesByQuad keyed by face id (identity): FacesByQuad[face] == [face].
        // PolyanyaSearch resolves the start/goal face from the quad id returned by
        // QuadGraph.NearestQuad, which for the CDT wrapper is the face id.
        var facesByQuad = new List<int>[mesh.Faces.Count];
        for (int f = 0; f < mesh.Faces.Count; f++)
            facesByQuad[f] = new List<int> { f };
        mesh.FacesByQuad = facesByQuad;

        return mesh;
    }

    // Build a combined PolyMesh from multiple per-tile CompactHeightfields. Each
    // tile is triangulated independently (Build), then merged into one mesh with
    // globally-welded border vertices so WireAdjacency reconnects faces across
    // tile seams. Mirrors the greedy path's per-tile MeshInto + cross-tile
    // BuildAdjacency, without materializing a full-zone fine grid.
    public static PolyMesh BuildMerged(IReadOnlyList<CompactHeightfield> tiles, System.Threading.CancellationToken cancel = default)
    {
        var combined = new PolyMesh();
        // Global vertex weld map so coincident border vertices from adjacent tiles
        // collapse to one index (enables cross-tile adjacency in WireAdjacency).
        var weld = new Dictionary<(long, long, long), int>();
        var pendingLinks = new List<(int fromFace, int toFace, Vector3 fromPos, Vector3 toPos, Navmesh.AreaId area)>();
        // Per combined-face tile id, so the seam de-walling pass can tell a tile
        // BORDER edge (duplicated by an adjacent tile) from a genuine intra-tile
        // wall (never duplicated).
        var faceTile = new List<int>();

        int tileId = 0;
        foreach (var chf in tiles)
        {
            cancel.ThrowIfCancellationRequested();
            var tileMesh = Build(chf, cancel);
            int faceOffset = combined.Faces.Count;
            // Remap this tile's vertices into the combined mesh (welded).
            var vmap = new int[tileMesh.Vertices.Count];
            for (int i = 0; i < tileMesh.Vertices.Count; i++)
                vmap[i] = WeldVertex3(tileMesh.Vertices[i], combined, weld);
            // Append faces (edges seeded as obstacle, constraint flag preserved).
            for (int f = 0; f < tileMesh.Faces.Count; f++)
            {
                var face = tileMesh.Faces[f];
                int nf = combined.AddFace(vmap[face.V0], vmap[face.V1], vmap[face.V2], face.Y, face.Layer);
                combined.SourceQuad[nf] = nf;
                faceTile.Add(tileId);
                for (int e = 0; e < 3; e++)
                    combined.Edges[nf * 3 + e] = new TriEdge(nf, -1, tileMesh.Edges[f * 3 + e].IsObstacleEdge);
            }
            foreach (var link in tileMesh.OffMeshLinks)
                pendingLinks.Add((link.FromFace + faceOffset, link.ToFace + faceOffset, link.FromPos, link.ToPos, link.Area));
            tileId++;
        }

        // Tile-seam healing: a tile's outer-border edge is emitted as an obstacle
        // (the tile was triangulated in isolation), so a flat surface split across
        // two tiles would stay disconnected. When the SAME welded edge is shared by
        // exactly two faces from DIFFERENT tiles whose floors are within climb, it
        // is a tile seam, not a real wall (real walls live inside a single tile and
        // are never duplicated by a neighbour). Clear the obstacle flag on both
        // sides so WireAdjacency pairs them across the seam.
        HealTileSeams(combined, faceTile);

        WireAdjacency(combined);

        foreach (var l in pendingLinks)
            combined.OffMeshLinks.Add(new OffMeshLink(l.fromFace, l.toFace, l.fromPos, l.toPos, l.area));

        var facesByQuad = new List<int>[combined.Faces.Count];
        for (int f = 0; f < combined.Faces.Count; f++)
            facesByQuad[f] = new List<int> { f };
        combined.FacesByQuad = facesByQuad;
        return combined;
    }

    // Clear the obstacle flag on tile-seam edges so WireAdjacency can connect
    // faces across the seam. A seam edge is a welded vertex pair shared by exactly
    // two obstacle edges whose two faces belong to DIFFERENT tiles and whose floor
    // Ys are close (continuous surface). Genuine intra-tile walls are shared by
    // faces of the SAME tile (or are single-sided) and are left untouched.
    //
    // Tiles are triangulated independently, so a seam can carry a T-junction: one
    // tile spans the seam with a single edge while the neighbour splits it at an
    // extra contour vertex. Such mismatched edges cannot weld 1:1, so the pass
    // first resolves T-junctions by splitting the spanning edge's face at the
    // neighbour's interior seam vertex, then clears obstacle flags on the now-
    // matching shared edges.
    private static void HealTileSeams(PolyMesh mesh, List<int> faceTile)
    {
        // Vertical tolerance for "continuous surface across the seam": small, since
        // a flat seam shares the exact rasterized Y. A generous half-cell-ish bound
        // avoids floating-point seam mismatch without bridging real height walls.
        const float seamYEps = 0.25f;

        // Resolve T-junctions until none remain (a split can expose a further one).
        // Bounded by the vertex count so a degenerate case fails loudly.
        int guard = mesh.Vertices.Count + 16;
        while (guard-- > 0 && SplitOneSeamTJunction(mesh, faceTile, seamYEps))
        {
        }

        var edgeMap = new Dictionary<(int, int), List<(int face, int edge)>>();
        for (int f = 0; f < mesh.Faces.Count; f++)
        {
            var face = mesh.Faces[f];
            int[] vs = { face.V0, face.V1, face.V2 };
            for (int e = 0; e < 3; e++)
            {
                int va = vs[e], vb = vs[(e + 1) % 3];
                var key = (Math.Min(va, vb), Math.Max(va, vb));
                if (!edgeMap.TryGetValue(key, out var list))
                    edgeMap[key] = list = new List<(int, int)>();
                list.Add((f, e));
            }
        }

        foreach (var (_, occ) in edgeMap)
        {
            if (occ.Count != 2)
                continue; // boundary or non-manifold; not a clean seam
            var (fa, ea) = occ[0];
            var (fb, eb) = occ[1];
            if (faceTile[fa] == faceTile[fb])
                continue; // same tile: a real intra-tile wall or interior edge
            // A tile-isolated border: at least one side is currently an obstacle
            // edge. (After a T-junction split the two halves may carry the obstacle
            // flag asymmetrically, so requiring it on both sides would miss seams.)
            if (!mesh.Edges[fa * 3 + ea].IsObstacleEdge && !mesh.Edges[fb * 3 + eb].IsObstacleEdge)
                continue;
            // Continuous surface: floor Ys close across the seam.
            if (MathF.Abs(mesh.Faces[fa].Y - mesh.Faces[fb].Y) > seamYEps)
                continue;
            // Clear obstacle flags; WireAdjacency will then pair these as interior.
            mesh.Edges[fa * 3 + ea] = mesh.Edges[fa * 3 + ea] with { IsObstacleEdge = false };
            mesh.Edges[fb * 3 + eb] = mesh.Edges[fb * 3 + eb] with { IsObstacleEdge = false };
        }
    }

    // Find ONE tile-seam T-junction and resolve it by splitting the spanning face.
    // A T-junction: an obstacle edge (a,b) of face f and a DISTINCT welded vertex s
    // (belonging to some other face) that lies strictly on segment a->b in XZ at a
    // matching height. Splits f's edge (a,b) at s into two faces. Returns true if a
    // split was made (caller re-scans), false when the seam is clean.
    private static bool SplitOneSeamTJunction(PolyMesh mesh, List<int> faceTile, float seamYEps)
    {
        for (int f = 0; f < mesh.Faces.Count; f++)
        {
            var face = mesh.Faces[f];
            int[] vs = { face.V0, face.V1, face.V2 };
            for (int e = 0; e < 3; e++)
            {
                if (!mesh.Edges[f * 3 + e].IsObstacleEdge)
                    continue; // only seam/boundary obstacle edges can carry a T-junction
                int a = vs[e], b = vs[(e + 1) % 3];
                var pa = mesh.Vertices[a];
                var pb = mesh.Vertices[b];
                // Look for a vertex s strictly between a and b in XZ at a close Y.
                for (int s = 0; s < mesh.Vertices.Count; s++)
                {
                    if (s == a || s == b) continue;
                    var ps = mesh.Vertices[s];
                    if (MathF.Abs(ps.Y - pa.Y) > seamYEps || MathF.Abs(ps.Y - pb.Y) > seamYEps)
                        continue;
                    // Collinear in XZ?
                    if (Predicates.Orient2D(pa.X, pa.Z, pb.X, pb.Z, ps.X, ps.Z) != 0)
                        continue;
                    // Strictly between (projection parameter in (0,1) in XZ).
                    float dx = pb.X - pa.X, dz = pb.Z - pa.Z;
                    float denom = dx * dx + dz * dz;
                    if (denom <= 0f) continue;
                    float tpar = ((ps.X - pa.X) * dx + (ps.Z - pa.Z) * dz) / denom;
                    if (tpar <= 1e-4f || tpar >= 1f - 1e-4f) continue;
                    // s belongs to a face other than f (a neighbour-tile seam vertex)?
                    if (!VertexUsedByOtherTileFace(mesh, faceTile, s, faceTile[f])) continue;

                    SplitFaceEdge(mesh, faceTile, f, e, s);
                    return true;
                }
            }
        }
        return false;
    }

    // Is welded vertex s referenced by any face whose tile differs from `tile`?
    private static bool VertexUsedByOtherTileFace(PolyMesh mesh, List<int> faceTile, int s, int tile)
    {
        for (int f = 0; f < mesh.Faces.Count; f++)
        {
            if (faceTile[f] == tile) continue;
            var face = mesh.Faces[f];
            if (face.V0 == s || face.V1 == s || face.V2 == s)
                return true;
        }
        return false;
    }

    // Split face f's edge e (from local vertex e to (e+1)%3) at interior vertex s.
    // Replaces face f with the triangle (a,s,c) and appends a new face (s,b,c),
    // where (a,b) is edge e and c is the opposite vertex. Both inherit the original
    // edge's obstacle flag on the split halves; the interior edges to c are
    // non-obstacle (they sit inside the original triangle). All other faces' edge
    // data is unaffected; adjacency is rebuilt by the caller's WireAdjacency.
    private static void SplitFaceEdge(PolyMesh mesh, List<int> faceTile, int f, int e, int s)
    {
        var face = mesh.Faces[f];
        int[] vs = { face.V0, face.V1, face.V2 };
        int a = vs[e], b = vs[(e + 1) % 3], c = vs[(e + 2) % 3];
        bool abObstacle = mesh.Edges[f * 3 + e].IsObstacleEdge;
        // The two non-split edges of the original triangle: (b->c) and (c->a).
        bool bcObstacle = mesh.Edges[f * 3 + (e + 1) % 3].IsObstacleEdge;
        bool caObstacle = mesh.Edges[f * 3 + (e + 2) % 3].IsObstacleEdge;
        float y = face.Y;
        int layer = face.Layer;
        int tile = faceTile[f];

        // Reuse face f as (a, s, c): edge0 a->s (half of a->b, obstacle), edge1 s->c
        // (interior, non-obstacle), edge2 c->a (original c->a flag).
        mesh.Faces[f] = new TriFace(a, s, c, y, layer);
        mesh.Edges[f * 3 + 0] = new TriEdge(f, -1, abObstacle);
        mesh.Edges[f * 3 + 1] = new TriEdge(f, -1, false);
        mesh.Edges[f * 3 + 2] = new TriEdge(f, -1, caObstacle);

        // New face (s, b, c): edge0 s->b (other half of a->b, obstacle), edge1 b->c
        // (original b->c flag), edge2 c->s (interior, non-obstacle).
        int nf = mesh.AddFace(s, b, c, y, layer);
        mesh.SourceQuad[nf] = nf;
        faceTile.Add(tile);
        mesh.Edges[nf * 3 + 0] = new TriEdge(nf, -1, abObstacle);
        mesh.Edges[nf * 3 + 1] = new TriEdge(nf, -1, bcObstacle);
        mesh.Edges[nf * 3 + 2] = new TriEdge(nf, -1, false);
    }

    private static int WeldVertex3(Vector3 v, PolyMesh mesh, Dictionary<(long, long, long), int> weld)
    {
        long kx = (long)MathF.Round(v.X / WeldEps);
        long ky = (long)MathF.Round(v.Y / WeldEps);
        long kz = (long)MathF.Round(v.Z / WeldEps);
        var key = (kx, ky, kz);
        if (weld.TryGetValue(key, out int idx))
            return idx;
        idx = mesh.AddVertex(v);
        weld[key] = idx;
        return idx;
    }

    // Median floor-Y across all spans assigned to this layer.
    private static float LayerFloorY(LayerPartition partition, int layerId)
    {
        var chf = partition.Chf;
        var ys = new List<float>();
        for (int z = 0; z < chf.Height; z++)
            for (int x = 0; x < chf.Width; x++)
            {
                var spans = chf.GetSpans(x, z);
                for (int si = 0; si < spans.Count; si++)
                    if (partition.GetLayer(x, z, si) == layerId)
                        ys.Add(spans[si].FloorY);
            }
        if (ys.Count == 0)
            return 0f;
        ys.Sort();
        return ys[ys.Count / 2];
    }

    private static void BuildLayer(CompactHeightfield chf, LayerPartition partition, int layerId, float floorY,
        PolyMesh mesh, List<int> faceList)
    {
        // Per-cell layer floor-Y, so faces sit on the local floor (staircases).
        var contours = ContourExtractor.ExtractContours(partition, layerId);
        if (contours.Count == 0)
            return;

        // Assemble the collinear-merged polylines into closed loops, then weld
        // and index vertices.
        var verts = new List<Vector2>();
        var vertIndex = new Dictionary<(long, long), int>();
        var loops = AssembleLoops(contours, verts, vertIndex);
        if (loops.Count == 0 || verts.Count < 3)
            return;

        // Constraints: every loop edge.
        var constraints = new List<Cdt.Constraint>();
        foreach (var loop in loops)
        {
            int n = loop.Count;
            for (int i = 0; i < n; i++)
                constraints.Add(new Cdt.Constraint(loop[i], loop[(i + 1) % n]));
        }

        var result = Cdt.Triangulate(verts, constraints, loops);
        if (result == null)
            return;

        // Append each triangle as a face. Vertex Y is the per-cell floor for that
        // XZ position when available, else the layer median; this keeps stair
        // layers (single layer, sloped) at the correct height.
        // Map result vertex index -> mesh global vertex index.
        var globalVert = new int[result.Vertices.Count];
        for (int i = 0; i < result.Vertices.Count; i++)
        {
            var v = result.Vertices[i];
            float y = SampleFloorY(chf, partition, layerId, v, floorY);
            globalVert[i] = mesh.AddVertex(new Vector3(v.X, y, v.Y));
        }

        for (int ti = 0; ti < result.Triangles.Count; ti++)
        {
            var (a, b, c) = result.Triangles[ti];
            int ga = globalVert[a], gb = globalVert[b], gc = globalVert[c];
            float fy = (mesh.Vertices[ga].Y + mesh.Vertices[gb].Y + mesh.Vertices[gc].Y) / 3f;
            int face = mesh.AddFace(ga, gb, gc, fy, layerId);
            // SourceQuad == the face's own index (identity) so the face-AABB
            // QuadGraph wrapper, NearestQuad, and PolyanyaSearch's per-quad face
            // lookup all agree (FacesByQuad[face] == [face]). Layer id is carried
            // on TriFace.Layer for debugging/serialization.
            mesh.SourceQuad[face] = face;
            faceList.Add(face);
            // Mark constrained edges as obstacle edges (already true by default in
            // AddFace; flip the non-constrained ones to non-obstacle so WireAdjacency
            // can later pair them, but keep faceRight=-1 until wired).
            for (int e = 0; e < 3; e++)
            {
                bool constrained = result.ConstraintEdges[ti * 3 + e];
                mesh.Edges[face * 3 + e] = new TriEdge(face, -1, constrained);
            }
        }
    }

    // Sample the layer's floor-Y nearest to XZ position v; falls back to the
    // layer median. Used so stair layers carry per-cell height.
    private static float SampleFloorY(CompactHeightfield chf, LayerPartition partition, int layerId, Vector2 v, float fallback)
    {
        int cx = chf.WorldToCell_X(v.X);
        int cz = chf.WorldToCell_Z(v.Y);
        float best = fallback;
        float bestD = float.MaxValue;
        for (int dz = -1; dz <= 0; dz++)
            for (int dx = -1; dx <= 0; dx++)
            {
                int nx = Math.Clamp(cx + dx, 0, chf.Width - 1);
                int nz = Math.Clamp(cz + dz, 0, chf.Height - 1);
                var spans = chf.GetSpans(nx, nz);
                for (int si = 0; si < spans.Count; si++)
                {
                    if (partition.GetLayer(nx, nz, si) != layerId) continue;
                    float cxw = chf.CellMinX(nx), czw = chf.CellMinZ(nz);
                    float d = (cxw - v.X) * (cxw - v.X) + (czw - v.Y) * (czw - v.Y);
                    if (d < bestD) { bestD = d; best = spans[si].FloorY; }
                }
            }
        return best;
    }

    // Assemble the collinear-merged WALL polylines into closed polygon loops.
    // ExtractContours already emits DIRECTED polylines that trace the
    // walkable/non-walkable boundary consistently (each wall edge oriented around
    // the walkable cell). We weld their endpoints to shared vertex indices, then
    // chain the directed polylines end-to-start: a polyline ending at vertex v is
    // followed by the next polyline starting at v, until the loop closes. This
    // preserves the boundary winding (no double-traversal of undirected edges).
    private static List<List<int>> AssembleLoops(List<List<Vector2>> contours, List<Vector2> verts,
        Dictionary<(long, long), int> vertIndex)
    {
        var loops = new List<List<int>>();

        // Convert each polyline into a welded directed index chain, dropping
        // consecutive duplicates. A self-closing polyline (first == last) is a
        // complete loop on its own; emit it directly. Remaining open chains are
        // stitched together below.
        var chains = new List<List<int>>();
        foreach (var poly in contours)
        {
            var chain = new List<int>();
            foreach (var p in poly)
            {
                int vi = WeldVertex(p, verts, vertIndex);
                if (chain.Count == 0 || chain[^1] != vi)
                    chain.Add(vi);
            }
            if (chain.Count >= 4 && chain[0] == chain[^1])
            {
                // Closed ring: drop the duplicate closing vertex, emit as a loop.
                chain.RemoveAt(chain.Count - 1);
                if (chain.Count >= 3)
                    loops.Add(chain);
                continue;
            }
            if (chain.Count >= 2 && chain[0] == chain[^1])
                chain.RemoveAt(chain.Count - 1);
            if (chain.Count >= 2)
                chains.Add(chain);
        }

        // Index chains by their START vertex so we can follow tail -> head.
        var byStart = new Dictionary<int, List<int>>();
        for (int ci = 0; ci < chains.Count; ci++)
        {
            int s = chains[ci][0];
            if (!byStart.TryGetValue(s, out var list)) byStart[s] = list = new List<int>();
            list.Add(ci);
        }

        var used = new bool[chains.Count];

        for (int ci = 0; ci < chains.Count; ci++)
        {
            if (used[ci]) continue;
            // A directed chain that already closes on itself is a complete loop.
            var loop = new List<int>();
            int cur = ci;
            int guard = chains.Count + 4;
            bool closed = false;
            while (guard-- > 0)
            {
                used[cur] = true;
                var chain = chains[cur];
                // Append all but the last vertex (the last is the next chain's start).
                for (int i = 0; i < chain.Count - 1; i++)
                    loop.Add(chain[i]);
                int tail = chain[^1];
                if (tail == loop[0])
                {
                    closed = true;
                    break; // loop closes back to its first vertex
                }
                // Find an unused chain starting at `tail`.
                int next = -1;
                if (byStart.TryGetValue(tail, out var candidates))
                    foreach (int cand in candidates)
                        if (!used[cand]) { next = cand; break; }
                if (next < 0)
                {
                    // Open chain; append the tail and stop (not a closed loop).
                    loop.Add(tail);
                    break;
                }
                cur = next;
            }
            if (closed && loop.Count >= 3)
                loops.Add(loop);
        }

        // Remove collinear vertices from each loop (the contour simplifier leaves
        // a collinear vertex at each chain's start/end seam; a collinear corner on
        // a hole boundary produces a sliver triangle whose centroid can fall just
        // inside the hole and survive the point-in-region cull).
        for (int li = 0; li < loops.Count; li++)
            loops[li] = RemoveCollinear(loops[li], verts);

        // Drop degenerate loops (zero signed area).
        loops.RemoveAll(l => l.Count < 3 || MathF.Abs(SignedArea(l, verts)) < 1e-6f);
        return loops;
    }

    // Drop vertices that are collinear with their neighbours (Orient2D == 0).
    private static List<int> RemoveCollinear(List<int> loop, List<Vector2> verts)
    {
        int n = loop.Count;
        if (n < 3) return loop;
        var result = new List<int>(n);
        for (int i = 0; i < n; i++)
        {
            var prev = verts[loop[(i - 1 + n) % n]];
            var cur = verts[loop[i]];
            var next = verts[loop[(i + 1) % n]];
            if (Predicates.Orient2D(prev, cur, next) != 0)
                result.Add(loop[i]);
        }
        return result.Count >= 3 ? result : loop;
    }

    private static float SignedArea(List<int> loop, List<Vector2> verts)
    {
        float area = 0;
        int n = loop.Count;
        for (int i = 0; i < n; i++)
        {
            var a = verts[loop[i]];
            var b = verts[loop[(i + 1) % n]];
            area += a.X * b.Y - b.X * a.Y;
        }
        return area * 0.5f;
    }

    private static int WeldVertex(Vector2 v, List<Vector2> verts, Dictionary<(long, long), int> vertIndex)
    {
        // Quantize to a grid finer than any real edge spacing for welding. Key on
        // the (kx,kz) tuple directly; a hashed-long key collides on sign-flipped
        // coordinate pairs (XOR symmetry) and merges distinct corners.
        long kx = (long)MathF.Round(v.X / WeldEps);
        long kz = (long)MathF.Round(v.Y / WeldEps);
        var key = (kx, kz);
        if (vertIndex.TryGetValue(key, out int idx))
            return idx;
        idx = verts.Count;
        verts.Add(v);
        vertIndex[key] = idx;
        return idx;
    }

    // Wire bilateral adjacency for shared, non-constrained edges across all faces.
    // An edge (a,b) shared by exactly two faces and constrained on neither becomes
    // an interior edge with mutual faceRight links.
    private static void WireAdjacency(PolyMesh mesh)
    {
        // Map undirected vertex-pair -> (face, edge) occurrences.
        var edgeMap = new Dictionary<(int, int), List<(int face, int edge)>>();
        for (int f = 0; f < mesh.Faces.Count; f++)
        {
            var face = mesh.Faces[f];
            int[] vs = { face.V0, face.V1, face.V2 };
            for (int e = 0; e < 3; e++)
            {
                int va = vs[e], vb = vs[(e + 1) % 3];
                var key = (Math.Min(va, vb), Math.Max(va, vb));
                if (!edgeMap.TryGetValue(key, out var list))
                    edgeMap[key] = list = new List<(int, int)>();
                list.Add((f, e));
            }
        }

        foreach (var (_, occ) in edgeMap)
        {
            if (occ.Count != 2)
                continue; // boundary edge or non-manifold; leave as obstacle
            var (fa, ea) = occ[0];
            var (fb, eb) = occ[1];
            // If either side is a constraint (boundary/obstacle), keep both as
            // obstacle edges: the search must not cross a walkable boundary.
            if (IsConstrained(mesh, fa, ea) || IsConstrained(mesh, fb, eb))
                continue;
            mesh.Edges[fa * 3 + ea] = new TriEdge(fa, fb, false);
            mesh.Edges[fb * 3 + eb] = new TriEdge(fb, fa, false);
        }
    }

    // An edge is constrained iff it was emitted as an obstacle edge AND has no
    // neighbour yet. CdtMeshBuilder seeds every edge as TriEdge(face, -1,
    // constrained); IsObstacleEdge stores the constraint flag.
    private static bool IsConstrained(PolyMesh mesh, int face, int edge)
        => mesh.Edges[face * 3 + edge].IsObstacleEdge;

    private static int FindFaceForLayerPoint(PolyMesh mesh, List<int>[] facesByLayer, int layer, Vector3 pos)
    {
        if (layer < 0 || layer >= facesByLayer.Length)
            return -1;
        var faces = facesByLayer[layer];
        int best = -1;
        float bestD = float.MaxValue;
        foreach (int f in faces)
        {
            var face = mesh.Faces[f];
            var a = mesh.Vertices[face.V0];
            var b = mesh.Vertices[face.V1];
            var c = mesh.Vertices[face.V2];
            if (PointInTriangleXZ(pos.X, pos.Z, a, b, c))
                return f;
            float cxw = (a.X + b.X + c.X) / 3f, czw = (a.Z + b.Z + c.Z) / 3f;
            float d = (cxw - pos.X) * (cxw - pos.X) + (czw - pos.Z) * (czw - pos.Z);
            if (d < bestD) { bestD = d; best = f; }
        }
        return best;
    }

    private static bool PointInTriangleXZ(float x, float z, Vector3 a, Vector3 b, Vector3 c)
    {
        double d1 = Predicates.Orient2D(a.X, a.Z, b.X, b.Z, x, z);
        double d2 = Predicates.Orient2D(b.X, b.Z, c.X, c.Z, x, z);
        double d3 = Predicates.Orient2D(c.X, c.Z, a.X, a.Z, x, z);
        bool hasNeg = d1 < 0 || d2 < 0 || d3 < 0;
        bool hasPos = d1 > 0 || d2 > 0 || d3 > 0;
        return !(hasNeg && hasPos);
    }
}
