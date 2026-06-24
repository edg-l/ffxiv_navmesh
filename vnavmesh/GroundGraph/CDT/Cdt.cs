using System;
using System.Collections.Generic;
using System.Numerics;
using Navmesh.GroundGraph.Geometry;
using Navmesh.GroundGraph.Polyanya;

namespace Navmesh.GroundGraph.CDT;

// Incremental Constrained Delaunay Triangulation.
//
// Pipeline:
//   1. Bowyer-Watson incremental insertion of every input vertex into a triangle
//      mesh seeded by a super-triangle large enough to contain all input. Each
//      insertion deletes every triangle whose circumcircle contains the new point
//      (the InCircle predicate, robust via Phase-1 Shewchuk predicates), then
//      re-triangulates the resulting star-shaped cavity. This yields the
//      unconstrained Delaunay triangulation.
//   2. Constraint enforcement by edge flipping: for every constrained boundary
//      segment that is not already an edge of the triangulation, repeatedly flip
//      the diagonals of the quadrilaterals straddling the segment until the
//      segment appears as a mesh edge (Sloan's diagonal-swap algorithm).
//   3. Remove triangles outside the walkable region: classify every triangle by
//      its centroid against the assembled boundary loops (outer minus holes) and
//      drop the exterior ones, along with the super-triangle skirt.
//
// All orientation / in-circle tests use the robust adaptive predicates in
// Navmesh.GroundGraph.Geometry.Predicates (Orient2D / InCircle).
public static class Cdt
{
    // A constrained boundary segment given as two vertex indices into the input
    // vertex list.
    public readonly record struct Constraint(int A, int B);

    // Result of a single-layer triangulation: the surviving CCW triangles (as
    // index triples into Vertices) and, per triangle, whether each of its three
    // edges (0:v0->v1, 1:v1->v2, 2:v2->v0) is a constrained (boundary/obstacle)
    // edge.
    public sealed class Result
    {
        public List<Vector2> Vertices = new();
        public List<(int v0, int v1, int v2)> Triangles = new();
        // Per triangle, 3 booleans (constrained edge?) flattened: tri*3 + edge.
        public List<bool> ConstraintEdges = new();
    }

    // Internal mutable triangle with neighbour links. Neighbour[e] is the triangle
    // across edge e (0:v0-v1, 1:v1-v2, 2:v2-v0), or -1 if none. Constrained[e]
    // marks an edge as a constraint segment. Alive=false marks a deleted triangle.
    private struct Tri
    {
        public int V0, V1, V2;
        public int N0, N1, N2;      // neighbour triangle across edge e
        public bool C0, C1, C2;     // constrained edge?
        public bool Alive;

        public int GetV(int i) => i == 0 ? V0 : i == 1 ? V1 : V2;
        public int GetN(int e) => e == 0 ? N0 : e == 1 ? N1 : N2;
        public bool GetC(int e) => e == 0 ? C0 : e == 1 ? C1 : C2;
        public void SetN(int e, int v) { if (e == 0) N0 = v; else if (e == 1) N1 = v; else N2 = v; }
        public void SetC(int e, bool v) { if (e == 0) C0 = v; else if (e == 1) C1 = v; else C2 = v; }
    }

    private sealed class Mesh
    {
        public List<Vector2> Verts = new();
        public List<Tri> Tris = new();

        // A recently-touched live triangle slot, used to seed the point-location
        // walk for the next Bowyer-Watson insertion (so insertion does not scan
        // every triangle). Updated to a freshly created fan triangle per insert.
        public int LastTri = 0;

        // Vertex -> incident triangle slots. A slot index appears in VertTris[v]
        // while triangle `v` is (or was) incident to vertex v. Entries are NOT
        // eagerly removed when a slot is killed or rewritten to drop a vertex;
        // readers must filter via TriHasVertex (which checks Alive + membership).
        // To keep lists short, IndexTri/ReindexTri append fresh memberships and
        // ReindexTri scrubs the stale vertex from a rewritten slot's old owners.
        public List<List<int>> VertTris = new();

        // Ensure VertTris has an entry for every current vertex.
        public void EnsureVertCapacity()
        {
            while (VertTris.Count < Verts.Count)
                VertTris.Add(new List<int>());
        }

        // Does live triangle slot t currently have vertex v as a corner?
        public bool TriHasVertex(int t, int v)
        {
            var tr = Tris[t];
            return tr.Alive && (tr.V0 == v || tr.V1 == v || tr.V2 == v);
        }

        // Record slot t as incident to each of its three current vertices.
        private void IndexTri(int t)
        {
            var tr = Tris[t];
            AddIncidence(tr.V0, t);
            AddIncidence(tr.V1, t);
            AddIncidence(tr.V2, t);
        }

        private void AddIncidence(int v, int t)
        {
            if (v < 0) return;
            EnsureVertCapacity();
            var list = VertTris[v];
            // Avoid duplicate live entries for the same slot (cheap: lists are short).
            for (int i = 0; i < list.Count; i++)
                if (list[i] == t) return;
            list.Add(t);
        }

        // Re-index a slot whose vertices changed in place (FlipEdge). Add the slot
        // to its new vertices' lists; stale memberships in vertices it no longer
        // touches are filtered out by readers via TriHasVertex. Periodically scrub
        // the old vertices' lists so they cannot grow without bound under heavy
        // flipping.
        public void ReindexTri(int t, int oldV0, int oldV1, int oldV2)
        {
            ScrubStale(oldV0);
            ScrubStale(oldV1);
            ScrubStale(oldV2);
            IndexTri(t);
        }

        // Drop dead/no-longer-incident slots from a single vertex's incident list.
        public void ScrubStale(int v)
        {
            if (v < 0 || v >= VertTris.Count) return;
            var list = VertTris[v];
            int w = 0;
            for (int i = 0; i < list.Count; i++)
            {
                int t = list[i];
                if (TriHasVertex(t, v))
                    list[w++] = t;
            }
            if (w < list.Count)
                list.RemoveRange(w, list.Count - w);
        }

        public int AddTri(int v0, int v1, int v2)
        {
            Tris.Add(new Tri { V0 = v0, V1 = v1, V2 = v2, N0 = -1, N1 = -1, N2 = -1, Alive = true });
            int t = Tris.Count - 1;
            IndexTri(t);
            return t;
        }

        // Edge e of triangle t goes from vertex `e` to vertex `(e+1)%3`.
        public (int a, int b) EdgeVerts(int t, int e)
        {
            var tr = Tris[t];
            int a = tr.GetV(e);
            int b = tr.GetV((e + 1) % 3);
            return (a, b);
        }

        // Find which edge index of triangle t is the directed/undirected edge (a,b).
        public int FindEdge(int t, int a, int b)
        {
            var tr = Tris[t];
            for (int e = 0; e < 3; e++)
            {
                int ea = tr.GetV(e), eb = tr.GetV((e + 1) % 3);
                if ((ea == a && eb == b) || (ea == b && eb == a))
                    return e;
            }
            return -1;
        }

        // Set the symmetric neighbour link: triangle t's edge e is adjacent to
        // triangle nbr. Also fixes nbr's back-link to point at t.
        public void SetNeighbour(int t, int e, int nbr)
        {
            var tr = Tris[t];
            tr.SetN(e, nbr);
            Tris[t] = tr;
            if (nbr >= 0)
            {
                var (a, b) = EdgeVerts(t, e);
                int ne = FindEdge(nbr, a, b);
                if (ne >= 0)
                {
                    var ntr = Tris[nbr];
                    ntr.SetN(ne, t);
                    Tris[nbr] = ntr;
                }
            }
        }
    }

    // Build a single-layer triangulation from input vertices and constrained
    // boundary segments. Returns null if the input is degenerate (fewer than 3
    // distinct points). The returned triangulation is the walkable interior only
    // (exterior + super-triangle skirt removed).
    public static Result? Triangulate(IReadOnlyList<Vector2> inputVerts, IReadOnlyList<Constraint> constraints,
        IReadOnlyList<List<int>> loops)
    {
        if (inputVerts.Count < 3)
            return null;

        var mesh = new Mesh();
        foreach (var v in inputVerts)
            mesh.Verts.Add(v);

        // 1. Super-triangle: encloses all input vertices with margin.
        float minX = float.MaxValue, minY = float.MaxValue, maxX = float.MinValue, maxY = float.MinValue;
        foreach (var v in inputVerts)
        {
            minX = MathF.Min(minX, v.X); minY = MathF.Min(minY, v.Y);
            maxX = MathF.Max(maxX, v.X); maxY = MathF.Max(maxY, v.Y);
        }
        float dx = maxX - minX, dy = maxY - minY;
        float dmax = MathF.Max(dx, dy);
        if (dmax < 1e-6f)
            return null; // all points coincide
        float midX = (minX + maxX) * 0.5f, midY = (minY + maxY) * 0.5f;
        // Generous margin so no real point lies on/outside the super-triangle.
        int s0 = mesh.Verts.Count; mesh.Verts.Add(new Vector2(midX - 20f * dmax, midY - dmax));
        int s1 = mesh.Verts.Count; mesh.Verts.Add(new Vector2(midX, midY + 20f * dmax));
        int s2 = mesh.Verts.Count; mesh.Verts.Add(new Vector2(midX + 20f * dmax, midY - dmax));
        // CCW super-triangle.
        EnsureCcw(mesh, ref s0, ref s1, ref s2);
        mesh.AddTri(s0, s1, s2);

        // 2. Incremental Bowyer-Watson insertion of each real input vertex, in
        //    spatially-coherent (Hilbert-curve) order. Inserting consecutive points
        //    that are close in the plane keeps each insertion's cavity small and the
        //    point-location walk short; inserting in raw index order (which, for a
        //    contour, marches monotonically across the extent) makes every new point
        //    fall inside the huge sliver circumcircles of its just-inserted
        //    predecessors, ballooning cavities to O(N) and the whole phase to O(N^2).
        //    The Delaunay triangulation is order-independent, so this only changes
        //    intermediate state, not the final mesh.
        var order = HilbertOrder(inputVerts, minX, minY, dmax);
        for (int i = 0; i < order.Length; i++)
            InsertPoint(mesh, order[i]);

        // 3. Enforce each constraint segment as a mesh edge by edge-flipping.
        foreach (var c in constraints)
        {
            if (c.A == c.B) continue;
            EnforceConstraint(mesh, c.A, c.B);
        }

        // 3b. Restore the Delaunay property on the unconstrained edges created by
        //     constraint insertion (Sloan's second pass): legalize every edge,
        //     exempting constrained ones, so triangle quality does not degrade.
        LegalizeAll(mesh);

        // 4. Classify and remove triangles: drop any triangle touching a
        //    super-triangle vertex, and any triangle whose centroid is outside
        //    the walkable region (outer loop minus holes).
        var result = new Result();
        foreach (var v in inputVerts)
            result.Vertices.Add(v);

        for (int t = 0; t < mesh.Tris.Count; t++)
        {
            var tr = mesh.Tris[t];
            if (!tr.Alive) continue;
            if (tr.V0 >= inputVerts.Count || tr.V1 >= inputVerts.Count || tr.V2 >= inputVerts.Count)
                continue; // touches super-triangle
            var pa = mesh.Verts[tr.V0];
            var pb = mesh.Verts[tr.V1];
            var pc = mesh.Verts[tr.V2];
            var centroid = (pa + pb + pc) / 3f;
            if (!PointInRegion(centroid, inputVerts, loops))
                continue;

            // Emit CCW (the mesh maintains CCW orientation; guard anyway).
            int a = tr.V0, b = tr.V1, c = tr.V2;
            if (Predicates.Orient2D(pa, pb, pc) < 0)
                (b, c) = (c, b);
            result.Triangles.Add((a, b, c));
        }

        // Per emitted triangle, mark which edges are constraints. We re-test
        // against the constraint set (a segment may be split by an intervening
        // vertex into collinear pieces, but boundary input has no interior points
        // on segments, so an exact endpoint match suffices).
        var constraintSet = new HashSet<(int, int)>();
        foreach (var c in constraints)
        {
            constraintSet.Add((Math.Min(c.A, c.B), Math.Max(c.A, c.B)));
        }
        foreach (var (v0, v1, v2) in result.Triangles)
        {
            result.ConstraintEdges.Add(constraintSet.Contains((Math.Min(v0, v1), Math.Max(v0, v1))));
            result.ConstraintEdges.Add(constraintSet.Contains((Math.Min(v1, v2), Math.Max(v1, v2))));
            result.ConstraintEdges.Add(constraintSet.Contains((Math.Min(v2, v0), Math.Max(v2, v0))));
        }

        return result;
    }

    // Order input-vertex indices along a Hilbert space-filling curve so that
    // consecutive insertions are spatially close (small cavities, short locate
    // walks). Coordinates are quantised to a 2^bits grid over the bounding box.
    private static int[] HilbertOrder(IReadOnlyList<Vector2> verts, float minX, float minY, float extent)
    {
        int n = verts.Count;
        var order = new int[n];
        for (int i = 0; i < n; i++)
            order[i] = i;
        if (extent <= 0f)
            return order;

        const int bits = 16;
        const int side = 1 << bits;
        float scale = (side - 1) / extent;
        var keys = new ulong[n];
        for (int i = 0; i < n; i++)
        {
            uint hx = (uint)Math.Clamp((int)((verts[i].X - minX) * scale + 0.5f), 0, side - 1);
            uint hy = (uint)Math.Clamp((int)((verts[i].Y - minY) * scale + 0.5f), 0, side - 1);
            keys[i] = HilbertD2XY(bits, hx, hy);
        }
        Array.Sort(keys, order);
        return order;
    }

    // Map (x,y) grid coordinates to their 1-D distance along a Hilbert curve of
    // the given order (canonical xy2d from Wikipedia's Hilbert-curve article).
    private static ulong HilbertD2XY(int bits, uint x, uint y)
    {
        uint nside = 1u << bits;
        ulong d = 0;
        for (uint s = nside / 2; s > 0; s /= 2)
        {
            uint rx = (uint)((x & s) > 0 ? 1 : 0);
            uint ry = (uint)((y & s) > 0 ? 1 : 0);
            d += (ulong)s * (ulong)s * ((3 * rx) ^ ry);
            // Rotate/reflect the quadrant.
            if (ry == 0)
            {
                if (rx == 1)
                {
                    x = nside - 1 - x;
                    y = nside - 1 - y;
                }
                (x, y) = (y, x);
            }
        }
        return d;
    }

    private static void EnsureCcw(Mesh mesh, ref int a, ref int b, ref int c)
    {
        if (Predicates.Orient2D(mesh.Verts[a], mesh.Verts[b], mesh.Verts[c]) < 0)
            (b, c) = (c, b);
    }

    // Bowyer-Watson point insertion. Find every triangle whose circumcircle
    // contains vertex `vi` (the "bad" triangles forming the cavity), collect the
    // cavity boundary edges, delete the bad triangles, and re-triangulate the
    // cavity by fanning new triangles from `vi` to each boundary edge.
    private static void InsertPoint(Mesh mesh, int vi)
    {
        var p = mesh.Verts[vi];

        // Locate the triangle containing p by a straight (Lawson) walk from the
        // last-touched triangle, then flood the Bowyer-Watson cavity outward from
        // it across neighbour links. The bad-triangle set (every triangle whose
        // circumcircle contains p) is connected and contains the locating
        // triangle, so the flood finds exactly the same set the old full-mesh scan
        // did — without an O(triangles) sweep per insertion.
        int seed = LocateTriangle(mesh, p);
        if (seed < 0)
            return; // p numerically outside the mesh (shouldn't happen inside super-triangle)

        var bad = new List<int>();
        var badSet = new HashSet<int>();
        var stack = new Stack<int>();
        stack.Push(seed);
        badSet.Add(seed);
        var boundary = new List<(int a, int b, int nbr, bool constrained)>();
        while (stack.Count > 0)
        {
            int t = stack.Pop();
            bad.Add(t);
            var tr = mesh.Tris[t];
            for (int e = 0; e < 3; e++)
            {
                int nbr = tr.GetN(e);
                bool nbrBad = nbr >= 0 && mesh.Tris[nbr].Alive && InCircumcircle(mesh, mesh.Tris[nbr], p);
                if (nbrBad)
                {
                    if (badSet.Add(nbr))
                        stack.Push(nbr);
                }
                else
                {
                    // Neighbour is good (or absent): this edge is on the cavity
                    // boundary.
                    var (a, b) = mesh.EdgeVerts(t, e);
                    boundary.Add((a, b, nbr, tr.GetC(e)));
                }
            }
        }
        if (bad.Count == 0)
            return; // numerically outside everything (shouldn't happen inside super-triangle)

        // Delete bad triangles.
        foreach (int t in bad)
        {
            var tr = mesh.Tris[t];
            tr.Alive = false;
            mesh.Tris[t] = tr;
        }

        // Re-triangulate the cavity: one new triangle per boundary edge, fanned
        // from p. Keep edge order (a -> b) so the new triangle (a, b, p) is CCW
        // when the boundary edge was CCW around the cavity. Each fan triangle
        // (a,b,vi) owns two spoke edges incident to vi: edge 1 = (b->vi) and
        // edge 2 = (vi->a). The fan whose first boundary vertex equals b shares
        // that b-spoke, so index fans by their `a` vertex to link spokes in O(1)
        // instead of an O(cavity^2) pairwise scan (cavities on regular grids are
        // large, so the quadratic scan dominated insertion).
        var newTris = new List<int>(boundary.Count);
        var fanByA = new Dictionary<int, int>(boundary.Count);
        foreach (var (a, b, nbr, constrained) in boundary)
        {
            int nt = mesh.AddTri(a, b, vi);
            // Edge 0 (a->b) faces the old neighbour outside the cavity.
            var tr = mesh.Tris[nt];
            tr.C0 = constrained;
            mesh.Tris[nt] = tr;
            // Link edge 0 to the outside neighbour.
            mesh.SetNeighbour(nt, 0, nbr);
            newTris.Add(nt);
            fanByA[a] = nt;
        }

        // Link each fan's b-spoke (edge 1: b->vi) to the fan that starts at b
        // (the one with a == b), which owns the matching vi->b spoke. This wires
        // every shared spoke exactly once.
        foreach (int nt in newTris)
        {
            var tr = mesh.Tris[nt];
            int b = tr.V1; // fan triangle is (a, b, vi)
            if (fanByA.TryGetValue(b, out int other) && other != nt)
                mesh.SetNeighbour(nt, 1, other);
        }

        // The cavity boundary vertices accumulated dead-slot entries when the bad
        // triangles were killed; scrub them so incident lists stay proportional to
        // true vertex degree rather than to the total number of insertions.
        var touched = new HashSet<int> { vi };
        foreach (var (a, b, _, _) in boundary)
        {
            touched.Add(a);
            touched.Add(b);
        }
        foreach (int v in touched)
            mesh.ScrubStale(v);

        // Seed the next insertion's point-location walk from a fan triangle (it
        // is live and near the just-inserted point).
        if (newTris.Count > 0)
            mesh.LastTri = newTris[0];
    }

    // Locate a live triangle containing point p via a straight (Lawson) walk from
    // mesh.LastTri. At each triangle, if p lies strictly to the right of any
    // directed CCW edge (Orient2D < 0), step across that edge into the neighbour;
    // otherwise p is inside (or on the boundary of) the triangle. Returns the
    // containing triangle, or -1 if the walk leaves the mesh (p outside the
    // super-triangle, which should not happen for real input). Bounded by a guard
    // that falls back to a full scan, preserving correctness on degenerate links.
    private static int LocateTriangle(Mesh mesh, Vector2 p)
    {
        int cur = mesh.LastTri;
        if (cur < 0 || cur >= mesh.Tris.Count || !mesh.Tris[cur].Alive)
            cur = FindAnyLiveTri(mesh);
        if (cur < 0)
            return -1;

        int guard = mesh.Tris.Count * 3 + 16;
        while (guard-- > 0)
        {
            var tr = mesh.Tris[cur];
            int next = -1;
            for (int e = 0; e < 3; e++)
            {
                var (a, b) = mesh.EdgeVerts(cur, e);
                // CCW triangle: interior is to the LEFT of each directed edge a->b
                // (Orient2D > 0). If p is strictly to the right, cross this edge.
                if (Predicates.Orient2D(mesh.Verts[a], mesh.Verts[b], p) < 0)
                {
                    int nbr = tr.GetN(e);
                    if (nbr >= 0 && mesh.Tris[nbr].Alive)
                    {
                        next = nbr;
                        break;
                    }
                    // No neighbour across the edge p is outside of: p is outside
                    // the mesh. Fall back to a full scan (covers numeric edge
                    // cases without ever returning a wrong triangle).
                    return FindContainingTriangle(mesh, p);
                }
            }
            if (next < 0)
                return cur; // p is inside (left of every edge)
            cur = next;
        }
        // Walk did not converge (degenerate/cyclic links): fall back to a scan.
        return FindContainingTriangle(mesh, p);
    }

    private static int FindAnyLiveTri(Mesh mesh)
    {
        for (int t = 0; t < mesh.Tris.Count; t++)
            if (mesh.Tris[t].Alive)
                return t;
        return -1;
    }

    // Fallback point location: scan for any live triangle whose circumcircle
    // contains p (sufficient as a Bowyer-Watson seed; the containing triangle
    // always qualifies). Used only on the rare degenerate walk that fails.
    private static int FindContainingTriangle(Mesh mesh, Vector2 p)
    {
        for (int t = 0; t < mesh.Tris.Count; t++)
        {
            var tr = mesh.Tris[t];
            if (!tr.Alive) continue;
            if (InCircumcircle(mesh, tr, p))
                return t;
        }
        return -1;
    }

    // Is point p strictly inside triangle tr's circumcircle? Uses the robust
    // InCircle predicate, with the triangle's vertices ordered CCW (the mesh
    // maintains CCW). InCircle returns >0 when p is inside for CCW input.
    private static bool InCircumcircle(Mesh mesh, Tri tr, Vector2 p)
    {
        var a = mesh.Verts[tr.V0];
        var b = mesh.Verts[tr.V1];
        var c = mesh.Verts[tr.V2];
        // Ensure CCW for InCircle's sign convention.
        if (Predicates.Orient2D(a, b, c) < 0)
            (b, c) = (c, b);
        return Predicates.InCircle(a, b, c, p) > 0;
    }

    // Enforce that segment (va, vb) is an edge of the triangulation, via Sloan's
    // constraint-insertion strip walk:
    //   1. If (va,vb) already exists, just mark it constrained.
    //   2. Otherwise collect the ordered list of edges that segment va->vb
    //      properly crosses by walking triangle-to-triangle from va toward vb.
    //   3. Flip those crossing edges in order; if a crossing edge is not currently
    //      flippable (non-convex quad), defer it (re-queue to the back) and
    //      continue, repeating until the list is empty.
    //   4. The edge (va,vb) must now exist: assert it and mark it constrained;
    //      throw if it does not (never silently mark a non-existent edge).
    private static void EnforceConstraint(Mesh mesh, int va, int vb) => EnforceConstraint(mesh, va, vb, 0);

    private static void EnforceConstraint(Mesh mesh, int va, int vb, int depth)
    {
        if (va == vb)
            return;

        // Already present?
        if (FindTriangleWithEdge(mesh, va, vb, out _) >= 0)
        {
            MarkConstrained(mesh, va, vb);
            return;
        }

        // Recursion guard: each split strictly shortens the span (a distinct
        // strictly-between vertex), so depth is bounded by the vertex count; the
        // cap is a safety net against a tolerance-induced non-shrinking split.
        if (depth > mesh.Verts.Count + 8)
        {
            Service.Log.Warning($"CDT: constraint ({va},{vb}) hit recursion cap; skipped (one wall edge may be missing here).");
            return;
        }

        // A vertex lying ON the constraint segment makes (va,vb) unrealizable as a
        // single edge — the wall genuinely passes through it. Split the constraint
        // at that vertex and enforce each half (recursively handles multiple
        // on-segment vertices). The strip walk reports such a vertex via its
        // proper-cross test, which is robust to the float-noise that an exact
        // Orient2D==0 collinearity check misses.
        CollectCrossingEdges(mesh, va, vb, out int blocker);
        if (blocker >= 0 && blocker != va && blocker != vb)
        {
            EnforceConstraint(mesh, va, blocker, depth + 1);
            EnforceConstraint(mesh, blocker, vb, depth + 1);
            return;
        }

        // 3. Anglada's list-based constraint insertion. Collect the ordered list of
        //    edges the segment crosses (as live vertex pairs). Sweep the list
        //    repeatedly: for each crossing edge whose union quad is CONVEX, flip it;
        //    the flip replaces the crossing diagonal with the opposite-apex diagonal.
        //    If the new diagonal still properly crosses the segment, keep it in the
        //    list (it remains a crossing); otherwise remove it. A reflex (non-convex)
        //    crossing edge is left for a later sweep, where a neighbouring flip has
        //    made its quad convex. The list strictly shrinks once it is all-convex,
        //    guaranteeing termination.
        var crossings = CollectCrossingEdges(mesh, va, vb, out _);
        var pa = mesh.Verts[va];
        var pb = mesh.Verts[vb];
        // A valid Anglada insertion resolves the strip in O(crossings^2) flips
        // (each sweep flips >=1; the list shrinks). Bound the cap by the STRIP
        // length, NOT mesh.Tris.Count — the latter made a 4-edge strip burn ~18k
        // iterations on a 1000-triangle tile before bailing, stalling the build.
        long sc = crossings.Count + 8;
        long maxIter = sc * sc + 64;
        long iter = 0;
        while (crossings.Count > 0)
        {
            bool progressed = false;
            for (int ci = 0; ci < crossings.Count;)
            {
                if (iter++ > maxIter)
                {
                    // A non-terminating flip sequence on degenerate (grid-collinear/
                    // cocircular) input. Skip this one constraint rather than killing
                    // the whole build; a missing wall edge is recoverable.
                    Service.Log.Warning($"CDT: constraint ({va},{vb}) flip sequence hit the iteration cap; skipping (one wall edge may be missing here).");
                    return;
                }

                var (ea, eb) = crossings[ci];
                int t = FindTriangleWithEdge(mesh, ea, eb, out int e);
                if (t < 0)
                {
                    // Edge no longer exists (removed by an earlier flip): drop it.
                    crossings.RemoveAt(ci);
                    progressed = true;
                    continue;
                }
                var tr = mesh.Tris[t];
                int nbr = tr.GetN(e);
                if (nbr < 0) { ci++; continue; }
                var (sa, sb) = mesh.EdgeVerts(t, e);
                int apexT = OppositeVertex(tr, sa, sb);
                int apexN = OppositeVertex(mesh.Tris[nbr], sa, sb);
                if (apexT < 0 || apexN < 0) { ci++; continue; }

                if (!FlipEdge(mesh, t, e))
                {
                    // Reflex quad: defer to a later sweep.
                    ci++;
                    continue;
                }
                progressed = true;

                // The flip created diagonal (apexT,apexN). If it still crosses the
                // segment it remains a crossing; replace this list entry. Otherwise
                // remove the entry.
                if (SegmentsProperlyCross(pa, pb, mesh.Verts[apexT], mesh.Verts[apexN]))
                {
                    crossings[ci] = (apexT, apexN);
                    ci++;
                }
                else
                {
                    crossings.RemoveAt(ci);
                }
            }

            if (!progressed)
            {
                // A full sweep flipped nothing: every remaining crossing edge is
                // reflex and no neighbour flip can fix it (degenerate input, e.g. a
                // vertex on the segment). Stop; step 4 distinguishes fatal cases.
                break;
            }
        }

        // 4. The constraint edge MUST now exist, UNLESS the input is degenerate
        //    (a third vertex lies exactly on segment va->vb, so no edge can span
        //    it). Distinguish: if some input vertex is collinear-between va and vb,
        //    the segment is unrepresentable as a single edge and we leave the
        //    triangulation as-is (the exterior cull drops the zero-area pieces).
        if (FindTriangleWithEdge(mesh, va, vb, out _) < 0)
        {
            // The flip sweep didn't realise (va,vb). If a vertex lies on/near the
            // segment (tolerance-based, catching near-collinear cases the strip
            // walk didn't flag), split there and recurse. Otherwise this is a
            // genuine reflex/degenerate deadlock — skip this one constraint with a
            // warning rather than killing the entire navmesh build (a single
            // missing wall edge is recoverable; a failed build is not).
            int vm = FindVertexOnOrNearSegment(mesh, va, vb, CollectStripVertices(mesh, va, vb));
            if (vm >= 0)
            {
                EnforceConstraint(mesh, va, vm, depth + 1);
                EnforceConstraint(mesh, vm, vb, depth + 1);
                return;
            }
            Service.Log.Warning($"CDT: could not realise constraint ({va},{vb}); skipping (one wall edge may be missing here).");
            return;
        }

        MarkConstrained(mesh, va, vb);
    }

    private static int FindTriangleWithEdge(Mesh mesh, int a, int b, out int edge)
    {
        // Edge (a,b) belongs only to triangles incident to vertex a; scan that
        // small incident list instead of the whole mesh.
        if (a >= 0 && a < mesh.VertTris.Count)
        {
            var list = mesh.VertTris[a];
            for (int i = 0; i < list.Count; i++)
            {
                int t = list[i];
                if (!mesh.TriHasVertex(t, a)) continue;
                int e = mesh.FindEdge(t, a, b);
                if (e >= 0) { edge = e; return t; }
            }
        }
        edge = -1;
        return -1;
    }

    private static void MarkConstrained(Mesh mesh, int a, int b)
    {
        // Edge (a,b) is owned by at most two triangles, both incident to a. Mark
        // the edge constrained on each (and its symmetric neighbour link).
        if (a < 0 || a >= mesh.VertTris.Count) return;
        var list = mesh.VertTris[a];
        for (int i = 0; i < list.Count; i++)
        {
            int t = list[i];
            if (!mesh.TriHasVertex(t, a)) continue;
            int e = mesh.FindEdge(t, a, b);
            if (e < 0) continue;
            var tr = mesh.Tris[t];
            tr.SetC(e, true);
            mesh.Tris[t] = tr;
            int nbr = tr.GetN(e);
            if (nbr >= 0)
            {
                int ne = mesh.FindEdge(nbr, a, b);
                if (ne >= 0)
                {
                    var ntr = mesh.Tris[nbr];
                    ntr.SetC(ne, true);
                    mesh.Tris[nbr] = ntr;
                }
            }
        }
    }

    // Collect the ORDERED list of triangulation edges that segment va->vb properly
    // crosses, as undirected vertex pairs (a,b). Walks the strip of triangles from
    // va toward vb: starts at the triangle around va whose opposite edge the
    // segment enters, then repeatedly steps across the crossed edge into the
    // neighbour triangle, until the triangle containing vb (or owning an edge
    // touching vb) is reached. Side tests use the robust Orient2D via
    // SegmentsProperlyCross. The returned pairs are captured by vertex index (not
    // triangle/edge slot) so they survive the mesh mutations that flipping causes.
    private static List<(int a, int b)> CollectCrossingEdges(Mesh mesh, int va, int vb, out int blockingVertex)
    {
        blockingVertex = -1;
        var pa = mesh.Verts[va];
        var pb = mesh.Verts[vb];
        var crossings = new List<(int a, int b)>();

        // Find the first triangle around va whose opposite edge the segment
        // crosses. Only triangles incident to va can be the strip's entry, so scan
        // va's incident list instead of the whole mesh.
        int curTri = -1;
        int curEdge = -1;
        if (va >= 0 && va < mesh.VertTris.Count)
        {
            var incident = mesh.VertTris[va];
            for (int i = 0; i < incident.Count && curTri < 0; i++)
            {
                int t = incident[i];
                var tr = mesh.Tris[t];
                if (!tr.Alive) continue;
                int li = tr.V0 == va ? 0 : tr.V1 == va ? 1 : tr.V2 == va ? 2 : -1;
                if (li < 0) continue;
                int oppEdge = (li + 1) % 3; // edge opposite va: (li+1)%3 -> (li+2)%3
                var (ea, eb) = mesh.EdgeVerts(t, oppEdge);
                if (ea == vb || eb == vb)
                    return crossings; // segment terminates at this triangle's far edge; (va,vb) already an edge
                if (SegmentsProperlyCross(pa, pb, mesh.Verts[ea], mesh.Verts[eb]))
                {
                    curTri = t;
                    curEdge = oppEdge;
                }
            }
        }
        if (curTri < 0)
            return crossings; // no strip entry found (segment already an edge, or degenerate)

        // Walk across crossed edges into successive neighbours until we reach vb.
        int guard = mesh.Tris.Count * 4 + 16;
        while (guard-- > 0)
        {
            var (ca, cb) = mesh.EdgeVerts(curTri, curEdge);
            crossings.Add((ca, cb));

            int nbr = mesh.Tris[curTri].GetN(curEdge);
            if (nbr < 0)
                break; // strip ran into a mesh boundary; stop (constraint loop will assert)

            // Apex of the neighbour opposite the shared edge.
            int apex = OppositeVertex(mesh.Tris[nbr], ca, cb);
            if (apex < 0)
                break;
            if (apex == vb)
                break; // reached the segment's far endpoint; strip complete

            // Choose which of the neighbour's two other edges the segment crosses
            // next: (ca,apex) or (apex,cb). Test both directly with the robust
            // proper-crossing predicate rather than a side heuristic (which is
            // fragile when ca/cb winding relative to va->vb varies).
            int eCaApex = mesh.FindEdge(nbr, ca, apex);
            int eApexCb = mesh.FindEdge(nbr, apex, cb);
            int nextEdge = -1;
            if (eCaApex >= 0 &&
                SegmentsProperlyCross(pa, pb, mesh.Verts[ca], mesh.Verts[apex]))
                nextEdge = eCaApex;
            else if (eApexCb >= 0 &&
                SegmentsProperlyCross(pa, pb, mesh.Verts[apex], mesh.Verts[cb]))
                nextEdge = eApexCb;
            if (nextEdge < 0)
            {
                // Neither sub-edge properly crosses: the segment runs through (or
                // within float-noise of) the apex vertex. If the apex projects
                // strictly between va and vb, it's an on-segment vertex that blocks
                // (va,vb) from being a single edge — report it so the caller splits
                // the constraint there. (Exact-zero Orient2D is too strict for real
                // contour data, which is why the walk's proper-cross test, not a
                // collinearity equality, is the authority here.)
                if (IsStrictlyBetween(pa, pb, mesh.Verts[apex]))
                    blockingVertex = apex;
                break;
            }
            curTri = nbr;
            curEdge = nextEdge;
        }

        return crossings;
    }

    // Restore the Delaunay property on all UNCONSTRAINED edges (queue-based
    // legalization). For each interior unconstrained edge, if the opposite vertex
    // of the neighbour triangle lies inside this triangle's circumcircle, flip it;
    // re-enqueue the edges of the two new triangles. Constrained edges are exempt.
    private static void LegalizeAll(Mesh mesh)
    {
        var queue = new Queue<(int a, int b)>();
        var enqueued = new HashSet<(int, int)>();

        void Enqueue(int a, int b)
        {
            var key = (Math.Min(a, b), Math.Max(a, b));
            if (enqueued.Add(key))
                queue.Enqueue((a, b));
        }

        for (int t = 0; t < mesh.Tris.Count; t++)
        {
            if (!mesh.Tris[t].Alive) continue;
            var tr = mesh.Tris[t];
            Enqueue(tr.V0, tr.V1);
            Enqueue(tr.V1, tr.V2);
            Enqueue(tr.V2, tr.V0);
        }

        long maxIter = (long)(mesh.Tris.Count + 4) * 16 + 64;
        long iter = 0;
        while (queue.Count > 0)
        {
            if (iter++ > maxIter)
                throw new InvalidOperationException(
                    $"CDT LegalizeAll exceeded iteration cap ({maxIter}); non-terminating flip sequence.");

            var (a, b) = queue.Dequeue();
            enqueued.Remove((Math.Min(a, b), Math.Max(a, b)));

            int t = FindTriangleWithEdge(mesh, a, b, out int e);
            if (t < 0) continue;
            var tr = mesh.Tris[t];
            if (tr.GetC(e)) continue; // constrained edge: exempt
            int nbr = tr.GetN(e);
            if (nbr < 0) continue; // boundary edge

            var (sa, sb) = mesh.EdgeVerts(t, e);
            int apexT = OppositeVertex(tr, sa, sb);
            int apexN = OppositeVertex(mesh.Tris[nbr], sa, sb);
            if (apexT < 0 || apexN < 0) continue;

            // InCircle test: ordered CCW for the predicate's sign convention.
            var p0 = mesh.Verts[apexT];
            var p1 = mesh.Verts[sa];
            var p2 = mesh.Verts[sb];
            if (Predicates.Orient2D(p0, p1, p2) < 0)
                (p1, p2) = (p2, p1);
            if (Predicates.InCircle(p0, p1, p2, mesh.Verts[apexN]) <= 0)
                continue; // empty-circle holds: nothing to do

            // Violation: flip if the quad is convex.
            if (!FlipEdge(mesh, t, e))
                continue; // non-convex (e.g. collinear quad): leave as is

            // Re-examine the four boundary edges of the flipped quad.
            Enqueue(sa, apexT);
            Enqueue(apexT, sb);
            Enqueue(sb, apexN);
            Enqueue(apexN, sa);
        }
    }

    // Flip the diagonal shared between triangle t (edge e) and its neighbour.
    // Returns false if the union quad is not convex (flip would invert).
    private static bool FlipEdge(Mesh mesh, int t, int e)
    {
        var tr = mesh.Tris[t];
        if (tr.GetC(e)) return false; // never flip a constrained edge
        int nbr = tr.GetN(e);
        if (nbr < 0) return false;
        var ntr = mesh.Tris[nbr];

        // Shared edge vertices.
        var (sa, sb) = mesh.EdgeVerts(t, e);
        // Apex of t opposite the shared edge.
        int apexT = OppositeVertex(tr, sa, sb);
        int apexN = OppositeVertex(ntr, sa, sb);
        if (apexT < 0 || apexN < 0) return false;

        var pSa = mesh.Verts[sa];
        var pSb = mesh.Verts[sb];
        var pT = mesh.Verts[apexT];
        var pN = mesh.Verts[apexN];

        // Convexity: the two apices must be on opposite sides of the shared edge
        // (they are, since they belong to opposite triangles) AND the new diagonal
        // apexT-apexN must lie inside the quad. Test convexity via orientation of
        // the quad (apexT, sa, apexN, sb) in order.
        // The flip is valid iff both new triangles are non-degenerate and CCW-able.
        double o1 = Predicates.Orient2D(pT, pSa, pN);
        double o2 = Predicates.Orient2D(pT, pN, pSb);
        if (o1 == 0 || o2 == 0)
            return false;
        if ((o1 > 0) != (o2 > 0))
            return false; // reflex quad; cannot flip

        // Gather the four boundary neighbours/constraint flags of the quad before
        // rewiring. The quad boundary edges (in t and nbr) are the non-shared edges.
        // t edges: shared edge e, plus edges (sa->apexT region). Identify by vertices.
        // Edge sa-apexT and apexT-sb belong to t; edge sb-apexN and apexN-sa belong to nbr.
        var (nbr_sa_apexT, c_sa_apexT) = EdgeNeighbour(mesh, t, sa, apexT);
        var (nbr_apexT_sb, c_apexT_sb) = EdgeNeighbour(mesh, t, apexT, sb);
        var (nbr_sb_apexN, c_sb_apexN) = EdgeNeighbour(mesh, nbr, sb, apexN);
        var (nbr_apexN_sa, c_apexN_sa) = EdgeNeighbour(mesh, nbr, apexN, sa);

        // Rebuild the two triangles around the new diagonal apexT-apexN.
        // New tri 1: apexT, sa, apexN  ; New tri 2: apexT, apexN, sb
        // Ensure CCW.
        int n1v0 = apexT, n1v1 = sa, n1v2 = apexN;
        if (Predicates.Orient2D(mesh.Verts[n1v0], mesh.Verts[n1v1], mesh.Verts[n1v2]) < 0)
            (n1v1, n1v2) = (n1v2, n1v1);
        int n2v0 = apexT, n2v1 = apexN, n2v2 = sb;
        if (Predicates.Orient2D(mesh.Verts[n2v0], mesh.Verts[n2v1], mesh.Verts[n2v2]) < 0)
            (n2v1, n2v2) = (n2v2, n2v1);

        // Old vertex sets of the two rewritten slots, for incidence re-indexing.
        int oldT0 = tr.V0, oldT1 = tr.V1, oldT2 = tr.V2;
        int oldN0 = ntr.V0, oldN1 = ntr.V1, oldN2 = ntr.V2;

        var t1 = new Tri { V0 = n1v0, V1 = n1v1, V2 = n1v2, N0 = -1, N1 = -1, N2 = -1, Alive = true };
        var t2 = new Tri { V0 = n2v0, V1 = n2v1, V2 = n2v2, N0 = -1, N1 = -1, N2 = -1, Alive = true };
        mesh.Tris[t] = t1;
        mesh.Tris[nbr] = t2;

        // Slot t went (oldT*) -> (apexT, sa, apexN); slot nbr went (oldN*) ->
        // (apexT, apexN, sb). Refresh the incidence index for the four corner
        // vertices so the new memberships are recorded and stale ones scrubbed.
        mesh.ReindexTri(t, oldT0, oldT1, oldT2);
        mesh.ReindexTri(nbr, oldN0, oldN1, oldN2);

        // Re-establish neighbour links + constraint flags for all six edges of the
        // two new triangles.
        WireEdge(mesh, t, sa, apexT, nbr_sa_apexT, c_sa_apexT);
        WireEdge(mesh, t, apexN, sa, nbr_apexN_sa, c_apexN_sa);
        WireEdge(mesh, nbr, apexT, sb, nbr_apexT_sb, c_apexT_sb);
        WireEdge(mesh, nbr, sb, apexN, nbr_sb_apexN, c_sb_apexN);
        // The new shared diagonal apexT-apexN between t and nbr (non-constrained).
        WireEdge(mesh, t, apexT, apexN, nbr, false);

        return true;
    }

    private static int OppositeVertex(Tri tr, int a, int b)
    {
        if (tr.V0 != a && tr.V0 != b) return tr.V0;
        if (tr.V1 != a && tr.V1 != b) return tr.V1;
        if (tr.V2 != a && tr.V2 != b) return tr.V2;
        return -1;
    }

    // Return the neighbour triangle and constraint flag of edge (a,b) of triangle t.
    private static (int nbr, bool constrained) EdgeNeighbour(Mesh mesh, int t, int a, int b)
    {
        int e = mesh.FindEdge(t, a, b);
        if (e < 0) return (-1, false);
        var tr = mesh.Tris[t];
        return (tr.GetN(e), tr.GetC(e));
    }

    // Set triangle t's edge (a,b) to point at neighbour `nbr` with constraint
    // flag `constrained`, and fix nbr's back-link.
    private static void WireEdge(Mesh mesh, int t, int a, int b, int nbr, bool constrained)
    {
        int e = mesh.FindEdge(t, a, b);
        if (e < 0) return;
        var tr = mesh.Tris[t];
        tr.SetN(e, nbr);
        tr.SetC(e, constrained);
        mesh.Tris[t] = tr;
        if (nbr >= 0 && mesh.Tris[nbr].Alive)
        {
            int ne = mesh.FindEdge(nbr, a, b);
            if (ne >= 0)
            {
                var ntr = mesh.Tris[nbr];
                ntr.SetN(ne, t);
                ntr.SetC(ne, constrained);
                mesh.Tris[nbr] = ntr;
            }
        }
    }

    // Is some real input vertex (other than va/vb) collinear with and strictly
    // between va and vb? Such a vertex makes segment (va,vb) unrepresentable as a
    // single triangulation edge (it would have to pass through that vertex).
    // True iff p projects strictly between pa and pb along the segment (parameter
    // in (eps, 1-eps)). Used by the strip walk to decide whether a stalled apex is
    // an on-segment blocker (vs the segment merely exiting the mesh).
    private static bool IsStrictlyBetween(Vector2 pa, Vector2 pb, Vector2 p)
    {
        float dx = pb.X - pa.X, dy = pb.Y - pa.Y;
        float denom = dx * dx + dy * dy;
        if (denom <= 0f) return false;
        float t = ((p.X - pa.X) * dx + (p.Y - pa.Y) * dy) / denom;
        return t > 1e-4f && t < 1f - 1e-4f;
    }

    // Collect a LOCAL candidate set of vertices near segment (va,vb): the corners
    // of every triangle the segment's strip crosses, plus the corners of triangles
    // incident to va and vb. A vertex lying on segment (va,vb) is necessarily a
    // corner of one of the crossed triangles, so this set is sufficient for the
    // on-segment fallback without scanning all mesh vertices per failed constraint.
    private static HashSet<int> CollectStripVertices(Mesh mesh, int va, int vb)
    {
        var candidates = new HashSet<int>();

        void AddTriCorners(int t)
        {
            if (t < 0 || !mesh.Tris[t].Alive) return;
            var tr = mesh.Tris[t];
            candidates.Add(tr.V0);
            candidates.Add(tr.V1);
            candidates.Add(tr.V2);
        }

        void AddIncident(int v)
        {
            if (v < 0 || v >= mesh.VertTris.Count) return;
            var list = mesh.VertTris[v];
            for (int i = 0; i < list.Count; i++)
                if (mesh.TriHasVertex(list[i], v))
                    AddTriCorners(list[i]);
        }

        AddIncident(va);
        AddIncident(vb);

        // The crossing edges are undirected vertex pairs; both triangles sharing
        // each crossing own its corners, which include any on-segment blocker.
        var crossings = CollectCrossingEdges(mesh, va, vb, out _);
        foreach (var (ea, eb) in crossings)
        {
            candidates.Add(ea);
            candidates.Add(eb);
            int t = FindTriangleWithEdge(mesh, ea, eb, out int e);
            if (t >= 0)
            {
                AddTriCorners(t);
                int nbr = mesh.Tris[t].GetN(e);
                AddTriCorners(nbr);
            }
        }

        return candidates;
    }

    // Find an input vertex lying on or very near segment (va,vb) and projecting
    // strictly between its endpoints, nearest to va. Tolerance-based (perpendicular
    // distance), so it catches near-collinear contour vertices that an exact
    // Orient2D==0 test misses. Restricted to the supplied local candidate set (the
    // strip around the segment) so it never scans the whole mesh. Returns -1 if
    // none.
    private static int FindVertexOnOrNearSegment(Mesh mesh, int va, int vb, HashSet<int> candidates)
    {
        var pa = mesh.Verts[va];
        var pb = mesh.Verts[vb];
        float dx = pb.X - pa.X, dy = pb.Y - pa.Y;
        float denom = dx * dx + dy * dy;
        if (denom <= 0f) return -1;
        const float perpEps = 1e-2f; // world units (~1/25 of a 0.25y cell)
        int best = -1;
        float bestT = float.MaxValue;
        foreach (int v in candidates)
        {
            if (v == va || v == vb) continue;
            if (v < 0 || v >= mesh.Verts.Count) continue;
            var p = mesh.Verts[v];
            float t = ((p.X - pa.X) * dx + (p.Y - pa.Y) * dy) / denom;
            if (t <= 1e-4f || t >= 1f - 1e-4f) continue;
            float projX = pa.X + dx * t, projY = pa.Y + dy * t;
            float ex = p.X - projX, ey = p.Y - projY;
            if (ex * ex + ey * ey > perpEps * perpEps) continue;
            if (t < bestT) { bestT = t; best = v; }
        }
        return best;
    }

    // Proper segment crossing (interiors intersect). Endpoints touching do NOT
    // count as a proper crossing (the segment may legitimately share an endpoint).
    private static bool SegmentsProperlyCross(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        double o1 = Predicates.Orient2D(a, b, c);
        double o2 = Predicates.Orient2D(a, b, d);
        double o3 = Predicates.Orient2D(c, d, a);
        double o4 = Predicates.Orient2D(c, d, b);
        return ((o1 > 0) != (o2 > 0)) && ((o3 > 0) != (o4 > 0));
    }

    // Point-in-region: inside the outer loop and outside every hole loop. Loops
    // are lists of vertex indices into `verts` (the input vertices). The first
    // loop (by convention from the assembler) with largest |signed area| is the
    // outer boundary; the assembler passes loops already classified, but we
    // re-derive containment robustly via even-odd ray casting against all loops.
    private static bool PointInRegion(Vector2 p, IReadOnlyList<Vector2> verts, IReadOnlyList<List<int>> loops)
    {
        if (loops.Count == 0)
            return true; // no boundary classification: keep everything (convex hull)
        int crossings = 0;
        foreach (var loop in loops)
        {
            int n = loop.Count;
            for (int i = 0; i < n; i++)
            {
                var v1 = verts[loop[i]];
                var v2 = verts[loop[(i + 1) % n]];
                if ((v1.Y > p.Y) != (v2.Y > p.Y))
                {
                    float t = (p.Y - v1.Y) / (v2.Y - v1.Y);
                    float xCross = v1.X + t * (v2.X - v1.X);
                    if (xCross > p.X)
                        crossings++;
                }
            }
        }
        return (crossings & 1) == 1;
    }
}
