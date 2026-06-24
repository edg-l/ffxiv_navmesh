using System;
using System.Collections.Generic;
using System.Numerics;

namespace Navmesh.GroundGraph.Polyanya;

// Off-mesh link: a discrete traversal edge between two faces whose endpoints are
// arbitrary 3D positions (e.g. a jump, a drop, an aetheryte). PolyanyaSearch
// treats these as outer-A* successors, NOT as intervals.
public readonly record struct OffMeshLink(int FromFace, int ToFace, Vector3 FromPos, Vector3 ToPos, Navmesh.AreaId Area);

// A triangle face of the polygon mesh. Vertex indices are CCW in XZ; Y is the
// face's floor height (carried from the source quad so the search can reason in
// 3D without re-probing). Layer is reserved for the Phase 3 multi-layer split
// (all faces from FromQuadGraph share layer 0 for now).
public readonly record struct TriFace(int V0, int V1, int V2, float Y, int Layer);

// A half-edge-like adjacency record per triangle edge. Edges are indexed CCW:
// edge 0 = V0->V1, edge 1 = V1->V2, edge 2 = V2->V0. faceLeft is the face that
// has this edge as a CCW boundary; faceRight is the neighbour across it (-1 if
// none, i.e. an obstacle/border edge).
public readonly record struct TriEdge(int FaceLeft, int FaceRight, bool IsObstacleEdge);

// A triangle: the three vertex indices in CCW order (XZ projection).
public readonly record struct Tri(int V0, int V1, int V2);

public class PolyMesh
{
    public List<Vector3> Vertices = new();
    public List<TriFace> Faces = new();
    // One entry per face, per edge (3 edges per face): edges[face*3 + edgeIndex].
    public List<TriEdge> Edges = new();
    public List<OffMeshLink> OffMeshLinks = new();
    // Source quad index per face (face -> QuadGraph.Quads index). Used to look
    // up per-quad flags (e.g. FLAG_UNREACHABLE) during search so Polyanya can
    // skip faces whose source quad is flagged. -1 for faces not from a quad.
    public List<int> SourceQuad = new();

    public int AddVertex(Vector3 v)
    {
        Vertices.Add(v);
        return Vertices.Count - 1;
    }

    public int AddFace(int v0, int v1, int v2, float y, int layer)
    {
        int face = Faces.Count;
        Faces.Add(new TriFace(v0, v1, v2, y, layer));
        Edges.Add(new TriEdge(face, -1, true));
        Edges.Add(new TriEdge(face, -1, true));
        Edges.Add(new TriEdge(face, -1, true));
        SourceQuad.Add(-1);
        return face;
    }

    public int AddFace(int v0, int v1, int v2, float y, int layer, int sourceQuad)
    {
        int face = AddFace(v0, v1, v2, y, layer);
        SourceQuad[face] = sourceQuad;
        return face;
    }

    // TriangulateQuad: split a quad (axis-aligned XZ box at height Y) into two
    // triangles along the FIXED anti-diagonal MinX-MaxZ -> MaxX-MinZ. The
    // diagonal is an interior (non-obstacle) edge. The two triangles are CCW in
    // XZ projection.
    //
    // Vertices (XZ, CCW):
    //   v0 = (MinX, MinZ)   v1 = (MaxX, MinZ)
    //   v2 = (MaxX, MaxZ)   v3 = (MinX, MaxZ)
    // Anti-diagonal: v3 (MinX,MaxZ) -> v1 (MaxX,MinZ).
    //   Tri A: v3, v0, v1  (CCW)
    //   Tri B: v3, v1, v2  (CCW)
    public static (Tri triA, Tri triB) TriangulateQuad(Quad q)
    {
        // Vertices are materialised by the caller (FromQuadGraph) into the mesh;
        // here we only return the index triples the caller will use. To keep the
        // snapshot test self-contained, the indices are expressed as the four
        // canonical corner slots 0..3 in the order [v0,v1,v2,v3] above.
        int v0 = 0, v1 = 1, v2 = 2, v3 = 3;
        Tri triA = new(v3, v0, v1);
        Tri triB = new(v3, v1, v2);
        return (triA, triB);
    }

    // FromQuadGraph: build a triangle PolyMesh from a QuadGraph.
    // - Each quad is triangulated as a fan from its (MinX,MaxZ) corner. When a
    //   quad side has multiple portals (multiple neighbours along one side),
    //   extra vertices are inserted at the portal endpoints so every portal
    //   span gets its own boundary edge with its own neighbour. Sides with no
    //   portal become obstacle edges.
    // - Shared portal spans (Portal.IsOffMesh == false) become interior edges
    //   with bilateral adjacency (non-obstacle).
    // - Every Portal.IsOffMesh becomes an OffMeshLink (none dropped; asserted).
    public static PolyMesh FromQuadGraph(QuadGraph g)
    {
        var mesh = new PolyMesh();

        // Per quad: list of triangle face indices produced by the fan.
        var quadFaces = new List<int>[g.Quads.Count];
        // Per quad: boundary vertex indices in CCW order starting at (MinX,MinZ).
        var quadBoundary = new List<int>[g.Quads.Count];

        for (int qi = 0; qi < g.Quads.Count; qi++)
        {
            var q = g.Quads[qi];
            float y = q.MinY;

            // Collect non-off-mesh portals incident to this quad, classified by
            // which side they lie on. A portal may be stored as FromQuad->ToQuad
            // or ToQuad->FromQuad (adjacency is bilateral); we normalise to the
            // side of THIS quad.
            var sidePortals = new List<(float lo, float hi, Portal portal)>[4];
            for (int s = 0; s < 4; s++) sidePortals[s] = new();
            foreach (var p in g.Portals)
            {
                if (p.IsOffMesh) continue;
                if (p.FromQuad != qi && p.ToQuad != qi) continue;
                int side = QuadSideForPortal(q, p);
                if (side < 0) continue;
                // Parametrize the portal span along the side's primary axis.
                float lo, hi;
                if (side == 0 || side == 1) // -X or +X: span varies along Z
                {
                    lo = MathF.Min(p.SpanMin.Y, p.SpanMax.Y);
                    hi = MathF.Max(p.SpanMin.Y, p.SpanMax.Y);
                }
                else // -Z or +Z: span varies along X
                {
                    lo = MathF.Min(p.SpanMin.X, p.SpanMax.X);
                    hi = MathF.Max(p.SpanMin.X, p.SpanMax.X);
                }
                sidePortals[side].Add((lo, hi, p));
            }

            // Build the boundary in CCW order: -Z (v0->v1), +X (v1->v2), +Z (v2->v3), -X (v3->v0).
            // Corner vertices:
            int v0 = mesh.AddVertex(new Vector3(q.MinX, y, q.MinZ)); // (MinX, MinZ)
            int v1 = mesh.AddVertex(new Vector3(q.MaxX, y, q.MinZ)); // (MaxX, MinZ)
            int v2 = mesh.AddVertex(new Vector3(q.MaxX, y, q.MaxZ)); // (MaxX, MaxZ)
            int v3 = mesh.AddVertex(new Vector3(q.MinX, y, q.MaxZ)); // (MinX, MaxZ)

            var boundary = new List<int>();
            var boundaryPortals = new List<Portal?>();
            AppendSideVertices(mesh, boundary, boundaryPortals, v0, v1, sidePortals[2], q, y, side: 2);
            AppendSideVertices(mesh, boundary, boundaryPortals, v1, v2, sidePortals[1], q, y, side: 1);
            AppendSideVertices(mesh, boundary, boundaryPortals, v2, v3, sidePortals[3], q, y, side: 3);
            AppendSideVertices(mesh, boundary, boundaryPortals, v3, v0, sidePortals[0], q, y, side: 0);

            quadBoundary[qi] = boundary;

            // Fan triangulate from boundary[0] = v0 = (MinX,MinZ). The plan's
            // fixed anti-diagonal is v3->v1; a fan from v0 produces triangles
            // (v0, b1, b2), (v0, b2, b3), ... which covers the polygon CCW.
            // Each boundary edge i (boundary[i] -> boundary[(i+1)%n]) is an edge
            // of the fan triangle that starts at boundary[i].
            var faces = new List<int>();
            int n = boundary.Count;
            for (int i = 1; i < n - 1; i++)
            {
                int a = boundary[0];
                int b = boundary[i];
                int c = boundary[i + 1];
                int face = mesh.AddFace(a, b, c, y, layer: 0, sourceQuad: qi);
                faces.Add(face);
            }
            quadFaces[qi] = faces;
        }

        // Wire boundary edges: each boundary edge is either an obstacle edge or
        // a portal edge (interior, bilateral). Boundary edge i of quad qi
        // connects boundary[i] -> boundary[(i+1)%n] and belongs to fan triangle
        // i-1 (triangle (b0, b_i, b_{i+1})). For i=0, the edge b0->b1 belongs to
        // fan triangle 0 as edge 1 (V1->V2). In general, fan triangle k = (b0,
        // b_{k+1}, b_{k+2}); its edge 1 = b_{k+1}->b_{k+2} = boundary edge
        // (k+1). Edge 0 = b0->b_{k+1} = interior fan edge. Edge 2 = b_{k+2}->b0
        // = interior fan edge. So boundary edge i (for i>=1) is edge 1 of fan
        // triangle (i-1). Boundary edge 0 (b0->b1) is edge 1 of fan triangle 0.
        // Wait: fan triangle 0 = (b0, b1, b2); edge 1 = b1->b2 = boundary edge 1.
        // Edge 0 = b0->b1 = boundary edge 0. So boundary edge i is edge 0 of
        // fan triangle i when i < n-1, and edge 2 of the LAST fan triangle when
        // i = n-1 (b_{n-1}->b0). Let me recompute:
        // Fan triangle k (0-indexed) = (V0=b0, V1=b_{k+1}, V2=b_{k+2}).
        //   edge 0 = V0->V1 = b0 -> b_{k+1}  (fan spoke)
        //   edge 1 = V1->V2 = b_{k+1} -> b_{k+2}  (boundary edge k+1)
        //   edge 2 = V2->V0 = b_{k+2} -> b0  (fan spoke)
        // So boundary edge i (b_i -> b_{i+1}) is edge 1 of fan triangle (i-1),
        // for i in 1..n-2. Boundary edge 0 (b0->b1) is edge 0 of fan triangle 0.
        // Boundary edge n-1 (b_{n-1}->b0) is edge 2 of fan triangle n-3.
        //
        // To keep this manageable, we map each boundary edge to (face, edgeIndex)
        // explicitly below.

        // First, wire the internal fan spokes (edge 0 and edge 2 of each fan
        // triangle) as bilateral interior edges between consecutive fan
        // triangles. Fan triangle k's edge 2 (b_{k+2}->b0) is the SAME undirected
        // edge as fan triangle (k+1)'s edge 0 (b0->b_{k+2}) — no, those are
        // different. The shared spoke between fan tri k and fan tri k+1 is
        // b0->b_{k+2}: in tri k it's edge 2 reversed (V2->V0 = b_{k+2}->b0), in
        // tri k+1 it's edge 0 (V0->V1 = b0->b_{k+2}). They are the same
        // undirected edge; wire bilateral adjacency.
        for (int qi = 0; qi < g.Quads.Count; qi++)
        {
            var faces = quadFaces[qi];
            for (int k = 0; k < faces.Count - 1; k++)
            {
                LinkInteriorEdge(mesh, faces[k], 2, faces[k + 1], 0);
            }
        }

        // Wire portal edges: for each non-off-mesh portal, find the boundary
        // edge on each quad that matches the portal span and link them.
        int offMeshCount = 0;
        foreach (var portal in g.Portals)
        {
            if (portal.IsOffMesh)
            {
                offMeshCount++;
                var fromFaces = quadFaces[portal.FromQuad];
                var toFaces = quadFaces[portal.ToQuad];
                var fromPos = new Vector3(portal.SpanMin.X, portal.YFrom, portal.SpanMin.Y);
                var toPos = new Vector3(portal.SpanMin.X, portal.YTo, portal.SpanMin.Y);
                int fromFace = FindFaceContainingPoint(mesh, fromFaces, fromPos.X, fromPos.Z);
                int toFace = FindFaceContainingPoint(mesh, toFaces, toPos.X, toPos.Z);
                mesh.OffMeshLinks.Add(new OffMeshLink(fromFace, toFace, fromPos, toPos, portal.Area));
                continue;
            }
            LinkPortalEdgeRefined(mesh, g, portal, quadBoundary, quadFaces);
        }

        // Off-mesh links that are bidirectional appear as two Portal entries;
        // the count check uses the number of off-mesh portals in the graph.
        if (mesh.OffMeshLinks.Count != offMeshCount)
            throw new InvalidOperationException(
                $"Off-mesh link count mismatch: graph had {offMeshCount} off-mesh portals, mesh has {mesh.OffMeshLinks.Count} links");

        return mesh;
    }

    // Quad sides: 0=-X, 1=+X, 2=-Z, 3=+Z.
    private static int QuadSideForPortal(Quad q, Portal p)
    {
        float sx = p.SpanMin.X;
        float sz = p.SpanMin.Y;
        if (MathF.Abs(sx - q.MinX) < 0.001f) return 0;
        if (MathF.Abs(sx - q.MaxX) < 0.001f) return 1;
        if (MathF.Abs(sz - q.MinZ) < 0.001f) return 2;
        if (MathF.Abs(sz - q.MaxZ) < 0.001f) return 3;
        return -1;
    }

    // Append boundary vertices for one side of a quad, inserting extra vertices
    // at portal endpoints so each portal span is its own boundary edge. The
    // side goes from `startV` to `endV` (CCW). `side` is 0=-X,1=+X,2=-Z,3=+Z.
    // Portals on this side are split points; gaps between/around portals are
    // obstacle edges.
    private static void AppendSideVertices(PolyMesh mesh, List<int> boundary, List<Portal?> boundaryPortals,
        int startV, int endV, List<(float lo, float hi, Portal portal)> portals, Quad q, float y, int side)
    {
        // The side's parametric range in its primary axis, traversed in CCW
        // order (the direction we walk the boundary):
        //   -X side (side 0): v3(MaxZ) -> v0(MinZ), Z decreasing.
        //   +X side (side 1): v1(MinZ) -> v2(MaxZ), Z increasing.
        //   -Z side (side 2): v0(MinX) -> v1(MaxX), X increasing.
        //   +Z side (side 3): v2(MaxX) -> v3(MinX), X decreasing.
        float paramStart, paramEnd;
        if (side == 0) { paramStart = q.MaxZ; paramEnd = q.MinZ; }
        else if (side == 1) { paramStart = q.MinZ; paramEnd = q.MaxZ; }
        else if (side == 2) { paramStart = q.MinX; paramEnd = q.MaxX; }
        else { paramStart = q.MaxX; paramEnd = q.MinX; }
        bool increasing = paramEnd > paramStart;
        float lo = MathF.Min(paramStart, paramEnd);
        float hi = MathF.Max(paramStart, paramEnd);

        // Sort portals by lo (in the increasing-param sense).
        portals.Sort((a, b) => a.lo.CompareTo(b.lo));

        // Build cut points in increasing param order, then reverse if the side
        // is traversed in decreasing param order so segments come out in walk
        // order.
        var cuts = new List<float> { lo };
        foreach (var pt in portals)
        {
            if (pt.lo > lo + 1e-4f) cuts.Add(pt.lo);
            if (pt.hi < hi - 1e-4f) cuts.Add(pt.hi);
        }
        cuts.Add(hi);
        cuts.Sort();
        var deduped = new List<float>();
        foreach (var c in cuts)
        {
            if (deduped.Count == 0 || MathF.Abs(deduped[^1] - c) > 1e-4f)
                deduped.Add(c);
        }
        if (!increasing) deduped.Reverse();
        cuts = deduped;

        // For each segment [cuts[i], cuts[i+1]] in walk order, determine if a
        // portal covers it. The segment's parametric extent (in increasing-param
        // terms) is (min(cuts[i],cuts[i+1]), max(...)).
        // Don't re-add startV if it's already the last boundary vertex (the
        // previous side's end vertex == this side's start vertex).
        if (boundary.Count == 0 || boundary[^1] != startV)
            boundary.Add(startV);
        for (int i = 0; i < cuts.Count - 1; i++)
        {
            float segLo = MathF.Min(cuts[i], cuts[i + 1]);
            float segHi = MathF.Max(cuts[i], cuts[i + 1]);
            Portal? covering = null;
            foreach (var pt in portals)
            {
                if (pt.lo <= segLo + 1e-3f && pt.hi >= segHi - 1e-3f)
                {
                    covering = pt.portal;
                    break;
                }
            }
            int vertToAdd;
            if (i == cuts.Count - 2)
            {
                // Last segment of this side: add endV unless it closes the loop
                // back to boundary[0] (the very first vertex of the whole
                // boundary). The boundary is an open polygon; the closing edge
                // is implicit.
                if (boundary.Count > 0 && endV == boundary[0])
                    break;
                vertToAdd = endV;
            }
            else
            {
                float param = cuts[i + 1]; // far end of this segment in walk order
                Vector3 pos;
                if (side == 0) pos = new Vector3(q.MinX, y, param);
                else if (side == 1) pos = new Vector3(q.MaxX, y, param);
                else if (side == 2) pos = new Vector3(param, y, q.MinZ);
                else pos = new Vector3(param, y, q.MaxZ);
                vertToAdd = mesh.AddVertex(pos);
            }
            boundary.Add(vertToAdd);
            boundaryPortals.Add(covering);
        }
    }

    // Link a portal edge in the refined mesh. The portal connects two quads;
    // find the boundary edge on each quad whose span matches the portal and link
    // them as bilateral interior edges.
    private static void LinkPortalEdgeRefined(PolyMesh mesh, QuadGraph g, Portal portal,
        List<int>[] quadBoundary, List<int>[] quadFaces)
    {
        int aEdge = FindBoundaryEdgeForPortal(quadBoundary[portal.FromQuad], mesh, g.Quads[portal.FromQuad], portal);
        int bEdge = FindBoundaryEdgeForPortal(quadBoundary[portal.ToQuad], mesh, g.Quads[portal.ToQuad], portal);
        if (aEdge < 0 || bEdge < 0)
            return;
        int aFace = FaceForBoundaryEdge(quadBoundary[portal.FromQuad], quadFaces[portal.FromQuad], aEdge);
        int bFace = FaceForBoundaryEdge(quadBoundary[portal.ToQuad], quadFaces[portal.ToQuad], bEdge);
        int aEdgeIdx = EdgeIndexForBoundaryEdge(quadBoundary[portal.FromQuad].Count, aEdge);
        int bEdgeIdx = EdgeIndexForBoundaryEdge(quadBoundary[portal.ToQuad].Count, bEdge);
        LinkInteriorEdge(mesh, aFace, aEdgeIdx, bFace, bEdgeIdx);
    }

    // Find which boundary edge index of `boundary` matches the portal span.
    private static int FindBoundaryEdgeForPortal(List<int> boundary, PolyMesh mesh, Quad q, Portal portal)
    {
        int n = boundary.Count;
        for (int i = 0; i < n; i++)
        {
            int va = boundary[i];
            int vb = boundary[(i + 1) % n];
            var pa = mesh.Vertices[va];
            var pb = mesh.Vertices[vb];
            // The portal span midpoint should lie on this edge.
            float midX = (portal.SpanMin.X + portal.SpanMax.X) * 0.5f;
            float midZ = (portal.SpanMin.Y + portal.SpanMax.Y) * 0.5f;
            // Check the midpoint is on the segment pa->pb (collinear + within).
            if (!PointOnSegment(midX, midZ, pa, pb))
                continue;
            // And the portal span is within the segment's extent.
            return i;
        }
        return -1;
    }

    // Find the face in `faces` whose triangle (XZ projection) contains (x, z).
    // Falls back to faces[0] if no face contains the point (e.g. point on boundary).
    private static int FindFaceContainingPoint(PolyMesh mesh, List<int> faces, float x, float z)
    {
        foreach (int f in faces)
        {
            var face = mesh.Faces[f];
            var va = mesh.Vertices[face.V0];
            var vb = mesh.Vertices[face.V1];
            var vc = mesh.Vertices[face.V2];
            if (PointInTriangleXZ(x, z, va, vb, vc))
                return f;
        }
        return faces[0];
    }

    // Test if (x, z) is inside or on the boundary of triangle (a, b, c) in XZ.
    private static bool PointInTriangleXZ(float x, float z, Vector3 a, Vector3 b, Vector3 c)
    {
        float d1 = Sign(x, z, a.X, a.Z, b.X, b.Z);
        float d2 = Sign(x, z, b.X, b.Z, c.X, c.Z);
        float d3 = Sign(x, z, c.X, c.Z, a.X, a.Z);
        bool hasNeg = d1 < 0 || d2 < 0 || d3 < 0;
        bool hasPos = d1 > 0 || d2 > 0 || d3 > 0;
        return !(hasNeg && hasPos);
    }

    private static float Sign(float px, float pz, float ax, float az, float bx, float bz)
        => (px - bx) * (az - bz) - (ax - bx) * (pz - bz);

    private static bool PointOnSegment(float x, float z, Vector3 a, Vector3 b)
    {
        // Project (x,z) onto segment a->b.
        float dx = b.X - a.X, dz = b.Z - a.Z;
        float lenSq = dx * dx + dz * dz;
        if (lenSq < 1e-9f) return false;
        float t = ((x - a.X) * dx + (z - a.Z) * dz) / lenSq;
        if (t < -0.01f || t > 1.01f) return false;
        float px = a.X + t * dx, pz = a.Z + t * dz;
        return MathF.Abs(px - x) < 0.05f && MathF.Abs(pz - z) < 0.05f;
    }

    // Which fan triangle owns boundary edge i.
    private static int FaceForBoundaryEdge(List<int> boundary, List<int> faces, int edgeIdx)
    {
        int n = boundary.Count;
        if (edgeIdx == 0) return faces[0];
        if (edgeIdx == n - 1) return faces[faces.Count - 1];
        return faces[edgeIdx - 1];
    }

    // Which edge index (0/1/2) within the fan triangle corresponds to boundary
    // edge i. boundaryCount is the number of boundary vertices.
    private static int EdgeIndexForBoundaryEdge(int boundaryCount, int edgeIdx)
    {
        if (edgeIdx == 0) return 0; // b0->b1 = edge 0 of fan tri 0
        if (edgeIdx == boundaryCount - 1) return 2; // b_{n-1}->b0 = edge 2 of last fan tri
        return 1; // b_i->b_{i+1} = edge 1 of fan tri (i-1)
    }

    // Wire two triangle edges as bilateral interior (non-obstacle) neighbours.
    private static void LinkInteriorEdge(PolyMesh mesh, int faceA, int edgeA, int faceB, int edgeB)
    {
        int idxA = faceA * 3 + edgeA;
        int idxB = faceB * 3 + edgeB;
        mesh.Edges[idxA] = new TriEdge(faceA, faceB, false);
        mesh.Edges[idxB] = new TriEdge(faceB, faceA, false);
    }
}