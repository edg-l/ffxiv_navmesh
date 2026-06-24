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

        public int AddTri(int v0, int v1, int v2)
        {
            Tris.Add(new Tri { V0 = v0, V1 = v1, V2 = v2, N0 = -1, N1 = -1, N2 = -1, Alive = true });
            return Tris.Count - 1;
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

        // 2. Incremental Bowyer-Watson insertion of each real input vertex.
        for (int vi = 0; vi < inputVerts.Count; vi++)
            InsertPoint(mesh, vi);

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

        // Find all triangles whose circumcircle contains p.
        var bad = new List<int>();
        for (int t = 0; t < mesh.Tris.Count; t++)
        {
            var tr = mesh.Tris[t];
            if (!tr.Alive) continue;
            if (InCircumcircle(mesh, tr, p))
                bad.Add(t);
        }
        if (bad.Count == 0)
            return; // numerically outside everything (shouldn't happen inside super-triangle)

        // Collect cavity boundary: an edge of a bad triangle is on the cavity
        // boundary iff its neighbour is not bad (or absent). Record (a, b,
        // neighbourTri, neighbourEdge, constrained).
        var badSet = new HashSet<int>(bad);
        var boundary = new List<(int a, int b, int nbr, bool constrained)>();
        foreach (int t in bad)
        {
            var tr = mesh.Tris[t];
            for (int e = 0; e < 3; e++)
            {
                int nbr = tr.GetN(e);
                if (nbr < 0 || !badSet.Contains(nbr))
                {
                    var (a, b) = mesh.EdgeVerts(t, e);
                    boundary.Add((a, b, nbr, tr.GetC(e)));
                }
            }
        }

        // Delete bad triangles.
        foreach (int t in bad)
        {
            var tr = mesh.Tris[t];
            tr.Alive = false;
            mesh.Tris[t] = tr;
        }

        // Re-triangulate the cavity: one new triangle per boundary edge, fanned
        // from p. Keep edge order (a -> b) so the new triangle (a, b, p) is CCW
        // when the boundary edge was CCW around the cavity.
        var newTris = new List<int>(boundary.Count);
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
        }

        // Link the new triangles to each other along their shared spokes (edges
        // incident to p). Match by shared vertices.
        for (int i = 0; i < newTris.Count; i++)
        {
            int ti = newTris[i];
            for (int j = i + 1; j < newTris.Count; j++)
            {
                int tj = newTris[j];
                // Find a shared edge (one incident to vi) between ti and tj.
                for (int e = 1; e < 3; e++) // edges 1 (b->p) and 2 (p->a) touch p
                {
                    var (ea, eb) = mesh.EdgeVerts(ti, e);
                    int je = mesh.FindEdge(tj, ea, eb);
                    if (je >= 0)
                    {
                        mesh.SetNeighbour(ti, e, tj);
                    }
                }
            }
        }
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
        long maxIter = (long)(crossings.Count + 4) * (mesh.Tris.Count + 4) * 2 + 64;
        long iter = 0;
        while (crossings.Count > 0)
        {
            bool progressed = false;
            for (int ci = 0; ci < crossings.Count;)
            {
                if (iter++ > maxIter)
                    throw new InvalidOperationException(
                        $"CDT EnforceConstraint exceeded iteration cap ({maxIter}) inserting segment ({va},{vb}); " +
                        "degenerate input or non-terminating flip sequence.");

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
            int vm = FindVertexOnOrNearSegment(mesh, va, vb);
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
        for (int t = 0; t < mesh.Tris.Count; t++)
        {
            if (!mesh.Tris[t].Alive) continue;
            int e = mesh.FindEdge(t, a, b);
            if (e >= 0) { edge = e; return t; }
        }
        edge = -1;
        return -1;
    }

    private static void MarkConstrained(Mesh mesh, int a, int b)
    {
        for (int t = 0; t < mesh.Tris.Count; t++)
        {
            if (!mesh.Tris[t].Alive) continue;
            int e = mesh.FindEdge(t, a, b);
            if (e >= 0)
            {
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

        // Find the first triangle around va whose opposite edge the segment crosses.
        int curTri = -1;
        int curEdge = -1;
        for (int t = 0; t < mesh.Tris.Count && curTri < 0; t++)
        {
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

        var t1 = new Tri { V0 = n1v0, V1 = n1v1, V2 = n1v2, N0 = -1, N1 = -1, N2 = -1, Alive = true };
        var t2 = new Tri { V0 = n2v0, V1 = n2v1, V2 = n2v2, N0 = -1, N1 = -1, N2 = -1, Alive = true };
        mesh.Tris[t] = t1;
        mesh.Tris[nbr] = t2;

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

    // Find an input vertex lying on or very near segment (va,vb) and projecting
    // strictly between its endpoints, nearest to va. Tolerance-based (perpendicular
    // distance), so it catches near-collinear contour vertices that an exact
    // Orient2D==0 test misses. Returns -1 if none.
    private static int FindVertexOnOrNearSegment(Mesh mesh, int va, int vb)
    {
        var pa = mesh.Verts[va];
        var pb = mesh.Verts[vb];
        float dx = pb.X - pa.X, dy = pb.Y - pa.Y;
        float denom = dx * dx + dy * dy;
        if (denom <= 0f) return -1;
        const float perpEps = 1e-2f; // world units (~1/25 of a 0.25y cell)
        int best = -1;
        float bestT = float.MaxValue;
        for (int v = 0; v < mesh.Verts.Count; v++)
        {
            if (v == va || v == vb) continue;
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
