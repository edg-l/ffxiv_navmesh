using Navmesh.GroundGraph.CDT;
using Navmesh.GroundGraph.Extraction;
using Navmesh.GroundGraph.Geometry;
using Navmesh.GroundGraph.Polyanya;
using System;
using System.Collections.Generic;
using System.Numerics;
using Xunit;

namespace Navmesh.Tests;

// Phase 4 Task 4.2: CDT validity tests.
//   - every triangle CCW (Orient2D > 0)
//   - Delaunay empty-circle on every unconstrained edge
//   - all input constraint segments present as edges
//   - watertight: every interior edge shared by exactly 2 triangles
//   - seeded fuzz over random simple polygons + a degenerate scene: no crash,
//     valid-or-empty.
[Collection("BackendTests")]
public class CdtTests : IClassFixture<ServiceFixture>
{
    // Build a closed-loop constraint set from a polygon (CCW vertex list).
    private static (List<Cdt.Constraint> constraints, List<List<int>> loops) LoopConstraints(int count, int offset = 0)
    {
        var constraints = new List<Cdt.Constraint>();
        var loop = new List<int>();
        for (int i = 0; i < count; i++)
        {
            loop.Add(offset + i);
            constraints.Add(new Cdt.Constraint(offset + i, offset + (i + 1) % count));
        }
        return (constraints, new List<List<int>> { loop });
    }

    [Fact]
    public void Square_ValidTriangulation()
    {
        var verts = new List<Vector2>
        {
            new(0, 0), new(4, 0), new(4, 4), new(0, 4),
        };
        var (constraints, loops) = LoopConstraints(4);
        var result = Cdt.Triangulate(verts, constraints, loops);
        Assert.NotNull(result);
        Assert.True(result!.Triangles.Count >= 2, "Square must triangulate into >= 2 triangles");
        AssertValid(result);
        AssertConstraintsPresent(result, constraints);
        AssertWatertight(result);
    }

    [Fact]
    public void ConstraintThroughInteriorVertices_SplitNotThrow()
    {
        // Regression for the in-game Limsa failure: a constraint segment passes
        // EXACTLY through other input vertices (cross-contour collinearity).
        // Verts 4 and 5 lie on constraint edge (0,1); the enforcer must split it
        // into (0,4),(4,5),(5,1) and recurse, not throw / kill the build.
        var verts = new List<Vector2>
        {
            new(0, 0), new(10, 0), new(10, 10), new(0, 10), // rectangle 0..3
            new(3, 0), new(7, 0),                            // on segment (0,1)
            new(5, 5),                                       // interior filler
        };
        var constraints = new List<Cdt.Constraint>
        {
            new(0, 1), new(1, 2), new(2, 3), new(3, 0),
        };
        var loops = new List<List<int>> { new() { 0, 1, 2, 3 } };

        var result = Cdt.Triangulate(verts, constraints, loops);
        Assert.NotNull(result);
        AssertValid(result!);
        AssertWatertight(result!);
        // The bottom constraint must be realised as the split chain 0-4-5-1.
        AssertConstraintsPresent(result!, new List<Cdt.Constraint> { new(0, 4), new(4, 5), new(5, 1) });
    }

    [Fact]
    public void SquareWithHole_ValidTriangulation()
    {
        // Outer 0..3, inner hole 4..7 (CW so it's a hole).
        var verts = new List<Vector2>
        {
            new(0, 0), new(10, 0), new(10, 10), new(0, 10),  // outer (CCW)
            new(4, 4), new(4, 6), new(6, 6), new(6, 4),        // hole (CW)
        };
        var constraints = new List<Cdt.Constraint>();
        var outer = new List<int> { 0, 1, 2, 3 };
        var hole = new List<int> { 4, 5, 6, 7 };
        for (int i = 0; i < 4; i++)
            constraints.Add(new Cdt.Constraint(outer[i], outer[(i + 1) % 4]));
        for (int i = 0; i < 4; i++)
            constraints.Add(new Cdt.Constraint(hole[i], hole[(i + 1) % 4]));
        var loops = new List<List<int>> { outer, hole };

        var result = Cdt.Triangulate(verts, constraints, loops);
        Assert.NotNull(result);
        AssertValid(result!);
        AssertConstraintsPresent(result!, constraints);
        AssertWatertight(result!);

        // No triangle centroid should fall inside the hole.
        foreach (var (a, b, c) in result!.Triangles)
        {
            var ctr = (result.Vertices[a] + result.Vertices[b] + result.Vertices[c]) / 3f;
            bool insideHole = ctr.X > 4.01f && ctr.X < 5.99f && ctr.Y > 4.01f && ctr.Y < 5.99f;
            Assert.False(insideHole, $"Triangle centroid {ctr} lies inside the hole");
        }
    }

    [Fact]
    public void Degenerate_Collinear_NoCrashValidOrEmpty()
    {
        // All points collinear -> no valid area; must return null or empty.
        var verts = new List<Vector2>
        {
            new(0, 0), new(1, 0), new(2, 0), new(3, 0),
        };
        var (constraints, loops) = LoopConstraints(4);
        var result = Cdt.Triangulate(verts, constraints, loops);
        if (result != null)
        {
            AssertValid(result);
            // Collinear loop encloses zero area -> expect no surviving triangles.
            Assert.Empty(result.Triangles);
        }
    }

    [Fact]
    public void Degenerate_Sliver_NoCrash()
    {
        // A near-degenerate sliver triangle plus a slightly off-axis point.
        var verts = new List<Vector2>
        {
            new(0, 0), new(10, 0), new(10, 0.0001f), new(0, 0.0001f),
        };
        var (constraints, loops) = LoopConstraints(4);
        var result = Cdt.Triangulate(verts, constraints, loops);
        if (result != null)
            AssertValid(result);
    }

    [Fact]
    public void Fuzz_RandomSimplePolygons_NoCrashValid()
    {
        var rng = new Random(20240624);
        for (int trial = 0; trial < 40; trial++)
        {
            int n = 5 + rng.Next(10);
            var verts = RandomStarShapedPolygon(rng, n, out var loop);
            var constraints = new List<Cdt.Constraint>();
            for (int i = 0; i < n; i++)
                constraints.Add(new Cdt.Constraint(loop[i], loop[(i + 1) % n]));
            var loops = new List<List<int>> { loop };

            var result = Cdt.Triangulate(verts, constraints, loops);
            if (result == null)
                continue;
            AssertValid(result);
            AssertWatertight(result);
            // A simple polygon of n vertices triangulates into n-2 triangles.
            Assert.True(result.Triangles.Count >= 1,
                $"Trial {trial}: star-shaped polygon should produce >= 1 triangle, got {result.Triangles.Count}");
        }
    }

    // Constraint-insertion flip path: an interior diagonal across a 5x5 grid of
    // points. The unconstrained Delaunay triangulation of a regular grid contains
    // NO corner-to-corner diagonal, so enforcing one FORCES a strip of edge flips
    // (Sloan's constraint-insertion walk). Asserts the constraint edge is present
    // after triangulation, the mesh is valid (CCW + empty-circle on unconstrained
    // edges, exercising the re-legalization sweep), and watertight.
    [Fact]
    public void InteriorDiagonal_AcrossGrid_ForcesFlips_ConstraintPresent()
    {
        var verts = new List<Vector2>();
        // 5x5 grid, indices row-major: idx = z*5 + x.
        for (int z = 0; z < 5; z++)
            for (int x = 0; x < 5; x++)
                verts.Add(new Vector2(x, z));

        // Boundary loop (the grid's outer square) as constraints so the cull keeps
        // the whole grid interior.
        var loop = new List<int> { 0, 1, 2, 3, 4, 9, 14, 19, 24, 23, 22, 21, 20, 15, 10, 5 };
        var constraints = new List<Cdt.Constraint>();
        for (int i = 0; i < loop.Count; i++)
            constraints.Add(new Cdt.Constraint(loop[i], loop[(i + 1) % loop.Count]));
        // The forced interior diagonal: (0,0)=idx0 to (4,3)=idx19. Slope 3/4 with
        // gcd(4,3)==1, so the segment passes through NO interior grid vertex; it is
        // representable as a single edge and FORCES a strip of edge flips (the
        // unconstrained Delaunay of the regular grid has no such diagonal).
        constraints.Add(new Cdt.Constraint(0, 19));
        var loops = new List<List<int>> { loop };

        var result = Cdt.Triangulate(verts, constraints, loops);
        Assert.NotNull(result);
        AssertValid(result!);
        AssertWatertight(result!);
        AssertConstraintsPresent(result!, constraints);

        // Explicitly assert the forced diagonal edge (0,19) exists.
        var present = new HashSet<(int, int)>();
        foreach (var (a, b, c) in result!.Triangles)
        {
            present.Add((Math.Min(a, b), Math.Max(a, b)));
            present.Add((Math.Min(b, c), Math.Max(b, c)));
            present.Add((Math.Min(c, a), Math.Max(c, a)));
        }
        Assert.True(present.Contains((0, 19)), "Forced interior diagonal (0,19) is missing after constraint insertion");
    }

    // A long boundary edge whose Delaunay strip wanders off both endpoints: a
    // jagged/concave polygon (comb teeth) whose boundary edges are NOT the
    // Delaunay edges, so enforcing them forces flips across multi-triangle strips.
    // Asserts all boundary constraints are present and the mesh is watertight.
    [Fact]
    public void JaggedConcavePolygon_BoundaryConstraints_AllPresent()
    {
        // A "comb" polygon: a base rectangle with several inward teeth. The
        // concavities mean the unconstrained Delaunay fills across the notches,
        // so boundary segments require flips to realise.
        var verts = new List<Vector2>
        {
            new(0, 0),    // 0
            new(10, 0),   // 1
            new(10, 6),   // 2
            new(8, 6),    // 3
            new(8, 2),    // 4  tooth notch
            new(6, 2),    // 5
            new(6, 6),    // 6
            new(4, 6),    // 7
            new(4, 2),    // 8  tooth notch
            new(2, 2),    // 9
            new(2, 6),    // 10
            new(0, 6),    // 11
        };
        var loop = new List<int>();
        for (int i = 0; i < verts.Count; i++)
            loop.Add(i);
        var (constraints, loops) = LoopFromExplicit(loop);

        var result = Cdt.Triangulate(verts, constraints, loops);
        Assert.NotNull(result);
        AssertValid(result!);
        AssertWatertight(result!);
        AssertConstraintsPresent(result!, constraints);
    }

    private static (List<Cdt.Constraint> constraints, List<List<int>> loops) LoopFromExplicit(List<int> loop)
    {
        var constraints = new List<Cdt.Constraint>();
        for (int i = 0; i < loop.Count; i++)
            constraints.Add(new Cdt.Constraint(loop[i], loop[(i + 1) % loop.Count]));
        return (constraints, new List<List<int>> { loop });
    }

    // CdtMeshBuilder over synthetic scenes: must produce a PolyMesh whose faces
    // are CCW and whose interior edges are bilateral.
    [Fact]
    public void MeshBuilder_FlatPlane_ValidPolyMesh()
    {
        var scene = SyntheticScenes.FlatPlane();
        var chf = CompactHeightfield.FromVoxelMap(scene.Volume, scene.BoundsMin, scene.BoundsMax, SceneHelper.AgentMaxClimb);
        var mesh = CdtMeshBuilder.Build(chf);
        Assert.True(mesh.Faces.Count > 0, "FlatPlane CDT mesh must have faces");
        AssertPolyMeshCcw(mesh);
        AssertPolyMeshBilateral(mesh);
    }

    [Fact]
    public void MeshBuilder_PlaneWithPillar_HoleExcluded()
    {
        var scene = SyntheticScenes.PlaneWithPillar();
        var chf = CompactHeightfield.FromVoxelMap(scene.Volume, scene.BoundsMin, scene.BoundsMax, SceneHelper.AgentMaxClimb);
        var mesh = CdtMeshBuilder.Build(chf);
        Assert.True(mesh.Faces.Count > 0);
        AssertPolyMeshCcw(mesh);
        // No GROUND-layer face centroid should fall inside the pillar footprint
        // [-2,2]x[-2,2]: the pillar is solid, so the ground (Y ~ 1.5) must have a
        // hole there. The pillar TOP (Y ~ 5.5) is a separate walkable layer and is
        // legitimately inside that XZ footprint, so we restrict the check to faces
        // at the ground floor height.
        const float groundY = 1.5f;
        foreach (var f in mesh.Faces)
        {
            if (MathF.Abs(f.Y - groundY) > 1.0f)
                continue; // not the ground layer (e.g. the pillar top)
            var a = mesh.Vertices[f.V0]; var b = mesh.Vertices[f.V1]; var c = mesh.Vertices[f.V2];
            float cx = (a.X + b.X + c.X) / 3f, cz = (a.Z + b.Z + c.Z) / 3f;
            bool insidePillar = cx > -1.7f && cx < 1.7f && cz > -1.7f && cz < 1.7f;
            Assert.False(insidePillar, $"Ground CDT face centroid ({cx:F2},{cz:F2}) Y={f.Y:F2} lies inside the pillar");
        }
    }

    // ---- Cross-tile stitching (BuildMerged) ----

    // Build TWO adjacent flat walkable tiles sharing a border at the same world
    // coordinates and merge them with CdtMeshBuilder.BuildMerged. Asserts:
    //   (a) the merged triangle-adjacency graph is a SINGLE connected component
    //       (the two tile halves are connected across the seam), and
    //   (b) coincident border vertices are welded (the merged vertex count is well
    //       below the naive sum of the two tiles' vertex counts; the shared seam
    //       column is not double-counted).
    [Fact]
    public void BuildMerged_TwoAdjacentTiles_SingleConnectedComponent()
    {
        const float cell = 1.0f;
        const float climb = 0.5f;
        const int w = 6, h = 6;
        const float floorY = 0.0f;

        // Tile A origin at X=0; tile B directly to +X so B's min-X face equals A's
        // max-X face at world X = w*cell. Same Z range and floor-Y so the seam
        // welds and the surface is continuous.
        var tileA = FlatTileChf(new Vector3(0, 0, 0), cell, climb, w, h, floorY);
        var tileB = FlatTileChf(new Vector3(w * cell, 0, 0), cell, climb, w, h, floorY);

        var merged = CdtMeshBuilder.BuildMerged(new[] { tileA, tileB });
        Assert.True(merged.Faces.Count > 0, "Merged CDT mesh must have faces");
        AssertPolyMeshCcw(merged);

        // (a) Single connected component over the triangle adjacency graph (the
        //     bilateral faceRight links produced by WireAdjacency).
        int components = CountAdjacencyComponents(merged);
        Assert.Equal(1, components);

        // (b) Border weld: the merged vertex count must be strictly less than the
        //     sum of the two tile meshes' vertex counts (the shared seam column is
        //     welded, not duplicated). Build the tiles in isolation to get a
        //     naive-sum baseline.
        int naiveSum = CdtMeshBuilder.Build(tileA).Vertices.Count
                     + CdtMeshBuilder.Build(tileB).Vertices.Count;
        Assert.True(merged.Vertices.Count < naiveSum,
            $"Border vertices not welded: merged {merged.Vertices.Count} >= naive sum {naiveSum}");
    }

    // A flat single-layer walkable tile: every cell has one walkable floor span at
    // floorY. BoundsMin sets the world origin so adjacent tiles share seam coords.
    private static CompactHeightfield FlatTileChf(Vector3 origin, float cell, float climb,
        int width, int height, float floorY)
    {
        int climbVoxels = Math.Max(1, (int)MathF.Floor(climb / cell));
        var chf = new CompactHeightfield(origin, cell, cell, width, height, 0, climbVoxels, climb);
        for (int z = 0; z < height; z++)
            for (int x = 0; x < width; x++)
                chf.AddSpanSorted(x, z, floorY, 1); // area 1 = walkable
        chf.FinalizeAllClearances();
        return chf;
    }

    // Count connected components of the face-adjacency graph induced by bilateral
    // (non-obstacle) edges in the PolyMesh.
    private static int CountAdjacencyComponents(PolyMesh mesh)
    {
        int n = mesh.Faces.Count;
        var seen = new bool[n];
        var stack = new Stack<int>();
        int components = 0;
        for (int start = 0; start < n; start++)
        {
            if (seen[start]) continue;
            components++;
            seen[start] = true;
            stack.Push(start);
            while (stack.Count > 0)
            {
                int f = stack.Pop();
                for (int e = 0; e < 3; e++)
                {
                    int nbr = mesh.Edges[f * 3 + e].FaceRight;
                    if (nbr >= 0 && !seen[nbr])
                    {
                        seen[nbr] = true;
                        stack.Push(nbr);
                    }
                }
            }
        }
        return components;
    }

    // ---- Validity helpers ----

    private static void AssertValid(Cdt.Result result)
    {
        // Every triangle CCW.
        for (int t = 0; t < result.Triangles.Count; t++)
        {
            var (a, b, c) = result.Triangles[t];
            double o = Predicates.Orient2D(result.Vertices[a], result.Vertices[b], result.Vertices[c]);
            Assert.True(o > 0, $"Triangle {t} ({a},{b},{c}) is not CCW (Orient2D={o})");
        }

        // Delaunay empty-circle on every UNCONSTRAINED interior edge: the apex of
        // the adjacent triangle across that edge must not be inside this
        // triangle's circumcircle.
        var edgeToTris = BuildEdgeMap(result);
        for (int t = 0; t < result.Triangles.Count; t++)
        {
            var (a, b, c) = result.Triangles[t];
            int[] vs = { a, b, c };
            for (int e = 0; e < 3; e++)
            {
                int va = vs[e], vb = vs[(e + 1) % 3];
                int apex = vs[(e + 2) % 3];
                bool constrained = result.ConstraintEdges[t * 3 + e];
                if (constrained) continue;
                var key = (Math.Min(va, vb), Math.Max(va, vb));
                if (!edgeToTris.TryGetValue(key, out var tris) || tris.Count != 2)
                    continue;
                int other = tris[0] == t ? tris[1] : tris[0];
                var (oa, ob, oc) = result.Triangles[other];
                int otherApex = oa != va && oa != vb ? oa : ob != va && ob != vb ? ob : oc;
                double inc = Predicates.InCircle(
                    result.Vertices[a], result.Vertices[b], result.Vertices[c],
                    result.Vertices[otherApex]);
                Assert.True(inc <= 1e-6,
                    $"Delaunay violated: edge ({va},{vb}) apex {otherApex} inside circumcircle of tri {t} (InCircle={inc})");
            }
        }
    }

    private static void AssertConstraintsPresent(Cdt.Result result, List<Cdt.Constraint> constraints)
    {
        var present = new HashSet<(int, int)>();
        foreach (var (a, b, c) in result.Triangles)
        {
            present.Add((Math.Min(a, b), Math.Max(a, b)));
            present.Add((Math.Min(b, c), Math.Max(b, c)));
            present.Add((Math.Min(c, a), Math.Max(c, a)));
        }
        foreach (var con in constraints)
        {
            if (con.A == con.B) continue;
            var key = (Math.Min(con.A, con.B), Math.Max(con.A, con.B));
            Assert.True(present.Contains(key),
                $"Constraint segment ({con.A},{con.B}) is not present as a triangulation edge");
        }
    }

    private static void AssertWatertight(Cdt.Result result)
    {
        var edgeToTris = BuildEdgeMap(result);
        foreach (var (key, tris) in edgeToTris)
        {
            // Each undirected edge is shared by 1 (boundary) or 2 (interior) tris.
            Assert.True(tris.Count == 1 || tris.Count == 2,
                $"Edge {key} is shared by {tris.Count} triangles (expected 1 or 2)");
        }
    }

    private static Dictionary<(int, int), List<int>> BuildEdgeMap(Cdt.Result result)
    {
        var map = new Dictionary<(int, int), List<int>>();
        for (int t = 0; t < result.Triangles.Count; t++)
        {
            var (a, b, c) = result.Triangles[t];
            int[] vs = { a, b, c };
            for (int e = 0; e < 3; e++)
            {
                int va = vs[e], vb = vs[(e + 1) % 3];
                var key = (Math.Min(va, vb), Math.Max(va, vb));
                if (!map.TryGetValue(key, out var list))
                    map[key] = list = new List<int>();
                list.Add(t);
            }
        }
        return map;
    }

    private static void AssertPolyMeshCcw(PolyMesh mesh)
    {
        for (int f = 0; f < mesh.Faces.Count; f++)
        {
            var face = mesh.Faces[f];
            var a = mesh.Vertices[face.V0]; var b = mesh.Vertices[face.V1]; var c = mesh.Vertices[face.V2];
            double o = Predicates.Orient2D(a.X, a.Z, b.X, b.Z, c.X, c.Z);
            Assert.True(o > 0, $"PolyMesh face {f} is not CCW in XZ (Orient2D={o})");
        }
    }

    private static void AssertPolyMeshBilateral(PolyMesh mesh)
    {
        for (int f = 0; f < mesh.Faces.Count; f++)
        {
            for (int e = 0; e < 3; e++)
            {
                var edge = mesh.Edges[f * 3 + e];
                int nbr = edge.FaceRight;
                if (nbr < 0) continue; // boundary/obstacle edge
                // The neighbour must have an edge whose FaceRight == f.
                bool backLink = false;
                for (int ne = 0; ne < 3; ne++)
                    if (mesh.Edges[nbr * 3 + ne].FaceRight == f) { backLink = true; break; }
                Assert.True(backLink, $"Face {f} edge {e} links to {nbr} but no back-link exists");
            }
        }
    }

    // Random star-shaped polygon: angularly sorted points around a center, so the
    // boundary is a simple (non-self-intersecting) CCW loop.
    private static List<Vector2> RandomStarShapedPolygon(Random rng, int n, out List<int> loop)
    {
        var angles = new List<float>();
        for (int i = 0; i < n; i++)
            angles.Add((float)(rng.NextDouble() * 2.0 * Math.PI));
        angles.Sort();
        var verts = new List<Vector2>();
        loop = new List<int>();
        for (int i = 0; i < n; i++)
        {
            float r = 2f + (float)rng.NextDouble() * 6f;
            verts.Add(new Vector2(MathF.Cos(angles[i]) * r, MathF.Sin(angles[i]) * r));
            loop.Add(i);
        }
        return verts;
    }
}
