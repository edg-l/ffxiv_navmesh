using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading;
using Navmesh.GroundGraph.Geometry;

namespace Navmesh.GroundGraph.Polyanya;

// Polyanya: any-angle pathfinding on a convex navmesh (Cui, Harabor, Grastien,
// IJCAI 2017, https://www.ijcai.org/proceedings/2017/0070.pdf). Nodes are
// intervals (root, edge, [a,b]) where root is the search root point, edge is a
// triangle edge, and [a,b] is the sub-interval of that edge visible from root.
//
// f = g(root) + dist(root -> interval) + h_lowerbound(interval -> target)  (§3.2)
// Successors are observable (the interval can see the target; path completes
// root -> target) or non-observable (the interval's neighbour face is entered,
// producing new visible sub-intervals of its edges). Cul-de-sac pruning (§4.1)
// drops successors whose neighbour face has no non-obstacle exit other than the
// edge back to the current face. Intermediate pruning (§4.2) drops successors
// that would re-derive an already-closed interval (same face, same entry edge,
// better-or-equal g). Off-mesh links are discrete outer-A* successors
// (cost = 3D link distance x area multiplier), resuming from toFace with a
// fresh root at toPos.
public class PolyanyaSearch
{
    private readonly PolyMesh _mesh;
    internal int _expandedCount;
    internal int _goalPushCount;

    // Optional per-source-quad flags. When set, the search skips any face whose
    // SourceQuad has the unreachable flag bit set (mirrors QuadGraph's
    // FLAG_UNREACHABLE flood-reachability filter). Null = no filtering.
    private int[]? _quadFlags;
    private int _unreachableFlag;

    // Open-list node. An interval lives on an edge of a face; root is the point
    // from which the interval is visible. gRoot is the accumulated path cost to
    // reach root. f is the priority key.
    private struct Node
    {
        public Vector2 Root;       // XZ position of the search root
        public float GRoot;        // cost to reach root
        public int Face;           // face the interval's edge belongs to
        public int Edge;           // edge index within the face (0..2)
        public Vector2 A;          // interval endpoint A (edge param)
        public Vector2 B;          // interval endpoint B (edge param)
        public float F;            // priority = GRoot + dist(root->closest) + h
        public int Parent;         // index into _nodes for path reconstruction
        public int OpenHeapIndex;  // -1 if closed/not in open
        public bool IsGoal;        // observable-to-target node (terminal)
        public Vector2 GoalPoint;  // the target point (for IsGoal nodes)
    }

    private readonly List<Node> _nodes = new();
    private readonly List<int> _open = new();
    private int _bestGoal = -1;
    private Vector2 _target;
    private float _targetY;
    private float _goalRange;

    public PolyanyaSearch(PolyMesh mesh)
    {
        _mesh = mesh;
    }

    // Set the per-source-quad flags array used to skip unreachable faces during
    // search. `unreachableFlag` is the bit to test (e.g. QuadGraph.FLAG_UNREACHABLE).
    public void SetQuadFlags(int[] quadFlags, int unreachableFlag)
    {
        _quadFlags = quadFlags;
        _unreachableFlag = unreachableFlag;
    }

    private bool FaceIsReachable(int face)
    {
        if (_quadFlags == null)
            return true;
        int sq = _mesh.SourceQuad[face];
        if (sq < 0 || sq >= _quadFlags.Length)
            return true;
        return (_quadFlags[sq] & _unreachableFlag) == 0;
    }

    // Find an any-angle path from `from` to `to` in XZ; Y is carried from the
    // face floor. Returns world-space waypoints (from prepended, to appended).
    // Returns empty list if no path. range > 0 terminates early when within range.
    public List<Vector3> FindPath(Vector3 from, Vector3 to, float range, CancellationToken cancel)
    {
        _goalRange = range;
        _target = new Vector2(to.X, to.Z);
        _targetY = to.Y;
        _nodes.Clear();
        _open.Clear();
        _bestGoal = -1;
        _expandedCount = 0;
        _goalPushCount = 0;

        int startFace = FindFaceContaining(new Vector2(from.X, from.Z));
        if (startFace < 0)
            return [];

        // Seed: the root is `from` itself, inside the start face. The initial
        // "interval" is the whole face (we model this as a sentinel node whose
        // successors are the three edges of the start face, all fully visible
        // from root). We push a synthetic node with Edge = -1 to denote
        // "inside face"; its successors expand each edge.
        var startNode = new Node
        {
            Root = new Vector2(from.X, from.Z),
            GRoot = 0f,
            Face = startFace,
            Edge = -1,
            A = Vector2.Zero,
            B = Vector2.Zero,
            F = Heuristic(new Vector2(from.X, from.Z)),
            Parent = -1,
            OpenHeapIndex = -1,
            IsGoal = false,
        };
        _nodes.Add(startNode);
        AddToOpen(0);

        Execute(cancel);

        if (_bestGoal < 0)
            return [];
        return ReconstructPath(_bestGoal, from, to);
    }

    private void Execute(CancellationToken cancel, int maxSteps = 2_000_000)
    {
        for (int i = 0; i < maxSteps; ++i)
        {
            if (_open.Count == 0)
                return;
            int nodeIdx = PopMinOpen();
            ref var node = ref CollectionsMarshal.AsSpan(_nodes)[nodeIdx];

            // If this is a goal node, record it and keep the best (lowest g).
            if (node.IsGoal)
            {
                if (_bestGoal < 0 || node.GRoot < _nodes[_bestGoal].GRoot)
                    _bestGoal = nodeIdx;
                // A goal node has no further successors.
                continue;
            }

            // Already-expanded nodes are skipped (lazy deletion). Mark closed.
            if (node.OpenHeapIndex != -2)
            {
                // not closed yet; process
            }
            node.OpenHeapIndex = -2; // closed

            if ((i & 0x3ff) == 0)
                cancel.ThrowIfCancellationRequested();

            Expand(nodeIdx);
        _expandedCount++;
        }
    }

    private void Expand(int nodeIdx)
    {
        ref var node = ref CollectionsMarshal.AsSpan(_nodes)[nodeIdx];
        int face = node.Face;

        if (node.Edge < 0)
        {
            // Seed: root inside `face`. If target is in this face, direct path.
            if (FaceContainsXZ(face, _target))
                PushGoal(nodeIdx, node.Root, _target);
            // Expand all three edges as exit candidates into their neighbours.
            for (int e = 0; e < 3; ++e)
                PushEdgeSuccessors(nodeIdx, face, e, node.Root, node.GRoot);
            PushOffMeshSuccessors(nodeIdx, face, node.Root);
        }
        else
        {
            // Interval node: search is in `face`, entered through `node.Edge`.
            // The target is a goal candidate iff it lies in `face`. If root->target
            // crosses the entry interval [a,b], the path is straight (observable,
            // §3.2). Otherwise the path bends through the closest point on [a,b]
            // to the target (non-observable goal); the turning point is that
            // closest point.
            if (FaceContainsXZ(face, _target))
                PushGoalForFace(nodeIdx, face, node.Root, node.A, node.B, node.GRoot);
            // Non-observable successors: expand the OTHER edges of `face` (not
            // the entry edge) into their neighbours.
            for (int e = 0; e < 3; ++e)
            {
                if (e == node.Edge)
                    continue;
                PushEdgeSuccessors(nodeIdx, face, e, node.Root, node.GRoot);
            }
            PushOffMeshSuccessors(nodeIdx, face, node.Root);
        }
    }

    // Push a goal candidate for a face containing the target. If root->target
    // crosses the entry interval [a,b], the path is straight (observable): cost
    // = gRoot + |root->target|, single segment. Otherwise the path bends through
    // the closest point on [a,b] to the target: cost = gRoot + |root->turn| +
    // |turn->target|, two segments, with the turning point inserted.
    private void PushGoalForFace(int parent, int face, Vector2 root, Vector2 a, Vector2 b, float gRoot)
    {
        _goalPushCount++;
        if (SegmentCrossesInterval(root, _target, a, b))
        {
            // Observable: straight root -> target.
            PushGoal(parent, root, _target);
            return;
        }
        // Non-observable: bend through the closest point on [a,b] to the target.
        Vector2 turn = ProjectOntoSegment(_target, a, b);
        float gTurn = gRoot + Vector2.Distance(root, turn);
        float distTurnToTarget = Vector2.Distance(turn, _target);
        // range termination
        if (_goalRange > 0)
        {
            Vector2 endPoint;
            float gEnd;
            if (distTurnToTarget <= _goalRange)
            {
                // turn is within range of the target; stop at turn.
                endPoint = turn;
                gEnd = gTurn;
            }
            else
            {
                // Stop at the point `range` from the target along turn->target.
                Vector2 dir = (_target - turn) / distTurnToTarget;
                endPoint = _target - dir * _goalRange;
                gEnd = gTurn + (distTurnToTarget - _goalRange);
            }
            var n = new Node
            {
                Root = endPoint,
                GRoot = gEnd,
                Face = face,
                Edge = -1,
                F = gEnd,
                Parent = parent,
                OpenHeapIndex = -1,
                IsGoal = true,
                GoalPoint = endPoint,
            };
            int idx = _nodes.Count;
            _nodes.Add(n);
            AddToOpen(idx);
            return;
        }
        float gTotal = gTurn + distTurnToTarget;
        var node = new Node
        {
            Root = turn,
            GRoot = gTotal,
            Face = face,
            Edge = -1,
            F = gTotal,
            Parent = parent,
            OpenHeapIndex = -1,
            IsGoal = true,
            GoalPoint = _target,
        };
        int id = _nodes.Count;
        _nodes.Add(node);
        AddToOpen(id);
    }

    // Push a goal node: root -> target directly observable. Cost = g(root) + |root->target|.
    // With range > 0, the path can terminate at a point within `range` of the
    // target along the root->target line. If the root itself is within range,
    // terminate at root. Otherwise, the termination point is `range` away from
    // the target along the root->target direction.
    private void PushGoal(int parent, Vector2 root, Vector2 target)
    {
        _goalPushCount++;
        float dist = Vector2.Distance(root, target);
        float parentG = _nodes[parent].GRoot;

        if (_goalRange > 0)
        {
            Vector2 endPoint;
            float gEnd;
            if (dist <= _goalRange)
            {
                // Root is already within range of the target; stop at root.
                endPoint = root;
                gEnd = parentG;
            }
            else
            {
                // Stop at the point `range` from the target along root->target.
                Vector2 dir = (target - root) / dist;
                endPoint = target - dir * _goalRange;
                gEnd = parentG + (dist - _goalRange);
            }
            var rangeNode = new Node
            {
                Root = endPoint,
                GRoot = gEnd,
                Face = _nodes[parent].Face,
                Edge = -1,
                F = gEnd,
                Parent = parent,
                OpenHeapIndex = -1,
                IsGoal = true,
                GoalPoint = endPoint,
            };
            int rid = _nodes.Count;
            _nodes.Add(rangeNode);
            AddToOpen(rid);
            return;
        }

        float g = parentG + dist;
        var node = new Node
        {
            Root = root,
            GRoot = g,
            Face = _nodes[parent].Face,
            Edge = -1,
            F = g,
            Parent = parent,
            OpenHeapIndex = -1,
            IsGoal = true,
            GoalPoint = target,
        };
        int id = _nodes.Count;
        _nodes.Add(node);
        AddToOpen(id);
    }

    // Cross from `face` through edge `edge` into the neighbour face. Push an
    // interval node in the neighbour: the entry edge is the neighbour's edge that
    // points back to `face`; the visible sub-interval is the full shared edge
    // (for triangulated convex quads, root sees the whole shared edge iff root
    // and the neighbour are on opposite sides of the shared edge, which holds by
    // construction).
    private void PushEdgeSuccessors(int parent, int face, int edge, Vector2 root, float gRoot)
    {
        int edgeIdx = face * 3 + edge;
        var e = _mesh.Edges[edgeIdx];
        int neighbour = e.FaceRight;
        if (neighbour < 0)
            return; // obstacle edge
        if (!FaceIsReachable(neighbour))
            return; // unreachable face (FLAG_UNREACHABLE source quad)

        // Find the entry edge in `neighbour` (the edge whose FaceRight == face).
        int entryEdge = -1;
        for (int ne = 0; ne < 3; ++ne)
        {
            if (_mesh.Edges[neighbour * 3 + ne].FaceRight == face)
            {
                entryEdge = ne;
                break;
            }
        }
        if (entryEdge < 0)
            return; // adjacency not wired symmetrically; skip

        // Cul-de-sac pruning (§4.1): drop the successor if the neighbour face has
        // no non-obstacle exit other than the edge back to `face`. A true dead-end
        // face only leads back to where we came from, so expanding it cannot
        // improve the path. This does NOT prune faces that have a second
        // non-obstacle edge leading elsewhere (the common case for triangles
        // adjacent to an obstacle but connected onward through the mesh), nor
        // faces that contain the target (the target is observable from the
        // successor, so the path completes there even if the face has no other
        // exit).
        bool targetInNeighbour = FaceContainsXZ(neighbour, _target);
        if (!targetInNeighbour)
        {
            bool neighbourHasOtherExit = false;
            for (int ne = 0; ne < 3; ++ne)
            {
                if (ne == entryEdge)
                    continue;
                var nEdge = _mesh.Edges[neighbour * 3 + ne];
                if (!nEdge.IsObstacleEdge && nEdge.FaceRight >= 0)
                {
                    neighbourHasOtherExit = true;
                    break;
                }
            }
            if (!neighbourHasOtherExit)
                return; // cul-de-sac: neighbour only leads back to face
        }

        // Intermediate pruning (§4.2): drop the successor if it would re-derive
        // an already-processed interval. The search uses lazy deletion (closed
        // nodes are marked OpenHeapIndex == -2 on pop), so we check whether an
        // existing node for (neighbour, entryEdge) is already closed with a
        // better-or-equal g. If so, the successor is redundant.
        if (HasClosedInterval(neighbour, entryEdge, gRoot))
            return;

        var (a, b) = EdgeEndpoints(neighbour, entryEdge);
        PushInterval(parent, neighbour, entryEdge, a, b, root, gRoot);
    }

    // Intermediate pruning helper (§4.2): returns true if there is already a
    // closed node for (face, edge) with g <= the candidate's g. A closed node
    // has OpenHeapIndex == -2. We scan _nodes linearly; the search is bounded
    // by maxSteps and the mesh is small (synthetic tests), so this is acceptable.
    // For a production-sized mesh, a (face,edge)->nodeIndex map would be added.
    private bool HasClosedInterval(int face, int edge, float gRoot)
    {
        var span = CollectionsMarshal.AsSpan(_nodes);
        for (int i = 0; i < span.Length; ++i)
        {
            ref var n = ref span[i];
            if (n.OpenHeapIndex == -2 && n.Face == face && n.Edge == edge && n.GRoot <= gRoot + 0.0001f)
                return true;
        }
        return false;
    }

    private void PushInterval(int parent, int face, int edge, Vector2 a, Vector2 b, Vector2 root, float gRoot)
    {
        // Closest point on [a,b] to root.
        Vector2 closest = ProjectOntoSegment(root, a, b);
        float distToInterval = Vector2.Distance(root, closest);

        // Range termination: if the closest point on this interval to root is
        // within `_goalRange` of the target, the search can stop here, building
        // a path to `closest`. This mirrors the original ground pathfinder's
        // range semantics (range>0 stops early within range of the goal;
        // range==0 = exact).
        if (_goalRange > 0 && Vector2.Distance(closest, _target) <= _goalRange)
        {
            float gClosest = gRoot + distToInterval;
            var rangeNode = new Node
            {
                Root = closest,
                GRoot = gClosest,
                Face = face,
                Edge = -1,
                F = gClosest,
                Parent = parent,
                OpenHeapIndex = -1,
                IsGoal = true,
                GoalPoint = closest,
            };
            int rid = _nodes.Count;
            _nodes.Add(rangeNode);
            AddToOpen(rid);
            // Still push the interval node so the search can continue past it
            // if a better (exact) path exists. The open list orders by f; the
            // range goal has a lower f (no h) and will be popped first.
        }

        float h = Vector2.Distance(closest, _target);
        float f = gRoot + distToInterval + h * 0.999f;
        var node = new Node
        {
            Root = root,
            GRoot = gRoot,
            Face = face,
            Edge = edge,
            A = a,
            B = b,
            F = f,
            Parent = parent,
            OpenHeapIndex = -1,
            IsGoal = false,
        };
        int id = _nodes.Count;
        _nodes.Add(node);
        AddToOpen(id);
    }

    // Off-mesh links attached to `face`: discrete successors. Cost = 3D link
    // distance x area multiplier. The search resumes from the link's toFace
    // with a fresh root at toPos (XZ).
    private void PushOffMeshSuccessors(int parent, int face, Vector2 root)
    {
        ref var p = ref CollectionsMarshal.AsSpan(_nodes)[parent];
        for (int i = 0; i < _mesh.OffMeshLinks.Count; ++i)
        {
            var link = _mesh.OffMeshLinks[i];
            if (link.FromFace != face)
                continue;
            if (!FaceIsReachable(link.FromFace) || !FaceIsReachable(link.ToFace))
                continue;
            float areaMult = AreaMultiplier(link.Area);
            float linkDist3D = Vector3.Distance(link.FromPos, link.ToPos);
            float linkCost = linkDist3D * areaMult;
            // Distance from root to the link's fromPos (within the face).
            float approach = Vector2.Distance(root, new Vector2(link.FromPos.X, link.FromPos.Z));
            float gRootNew = p.GRoot + approach + linkCost;
            Vector2 newRoot = new(link.ToPos.X, link.ToPos.Z);
            // The link lands in toFace; seed a fresh expansion from there.
            float h = Vector2.Distance(newRoot, _target);
            float f = gRootNew + h * 0.999f;
            var node = new Node
            {
                Root = newRoot,
                GRoot = gRootNew,
                Face = link.ToFace,
                Edge = -1, // fresh seed in toFace
                A = Vector2.Zero,
                B = Vector2.Zero,
                F = f,
                Parent = parent,
                OpenHeapIndex = -1,
                IsGoal = false,
            };
            // Stash the link's toPos Y so path reconstruction can insert the link
            // endpoint. We use GoalPoint as scratch (harmless for non-goal nodes).
            node.GoalPoint = newRoot;
            int id = _nodes.Count;
            _nodes.Add(node);
            AddToOpen(id);
        }
    }

    // ---- Geometry helpers ----

    private int FindFaceContaining(Vector2 p)
    {
        for (int i = 0; i < _mesh.Faces.Count; ++i)
        {
            if (!FaceIsReachable(i))
                continue;
            if (FaceContainsXZ(i, p))
                return i;
        }
        return -1;
    }

    private bool FaceContainsXZ(int face, Vector2 p)
    {
        var f = _mesh.Faces[face];
        var v0 = ToXZ(_mesh.Vertices[f.V0]);
        var v1 = ToXZ(_mesh.Vertices[f.V1]);
        var v2 = ToXZ(_mesh.Vertices[f.V2]);
        return PointInTriangle(p, v0, v1, v2);
    }

    private static Vector2 ToXZ(Vector3 v) => new(v.X, v.Z);

    private static bool PointInTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
    {
        double d1 = Predicates.Orient2D(a, b, p);
        double d2 = Predicates.Orient2D(b, c, p);
        double d3 = Predicates.Orient2D(c, a, p);
        bool hasNeg = d1 < 0 || d2 < 0 || d3 < 0;
        bool hasPos = d1 > 0 || d2 > 0 || d3 > 0;
        return !(hasNeg && hasPos);
    }

    // Does the segment root->target cross the interval [a,b]? Used for observable
    // successor detection: the target is observable from the interval iff the
    // root->target segment passes through [a,b].
    private static bool SegmentCrossesInterval(Vector2 root, Vector2 target, Vector2 a, Vector2 b)
    {
        // Find intersection of segment root->target with segment a->b.
        if (SegmentSegmentIntersect(root, target, a, b, out var hit))
            return true;
        // If root or target lies on a->b, treat as crossing (degenerate).
        return false;
    }

    private static bool SegmentSegmentIntersect(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, out Vector2 hit)
    {
        hit = default;
        var d1 = p2 - p1;
        var d2 = p4 - p3;
        float denom = d1.X * d2.Y - d1.Y * d2.X;
        if (MathF.Abs(denom) < 1e-9f)
            return false; // parallel
        var diff = p3 - p1;
        float t = (diff.X * d2.Y - diff.Y * d2.X) / denom;
        float u = (diff.X * d1.Y - diff.Y * d1.X) / denom;
        if (t >= -1e-6f && t <= 1f + 1e-6f && u >= -1e-6f && u <= 1f + 1e-6f)
        {
            hit = p1 + d1 * Math.Clamp(t, 0f, 1f);
            return true;
        }
        return false;
    }

    private (Vector2 a, Vector2 b) EdgeEndpoints(int face, int edge)
    {
        var f = _mesh.Faces[face];
        int vA = edge == 0 ? f.V0 : edge == 1 ? f.V1 : f.V2;
        int vB = edge == 0 ? f.V1 : edge == 1 ? f.V2 : f.V0;
        return (ToXZ(_mesh.Vertices[vA]), ToXZ(_mesh.Vertices[vB]));
    }

    private static Vector2 ProjectOntoSegment(Vector2 p, Vector2 a, Vector2 b)
    {
        var ab = b - a;
        float lenSq = ab.LengthSquared();
        if (lenSq < 1e-12f)
            return a;
        float t = Vector2.Dot(p - a, ab) / lenSq;
        t = Math.Clamp(t, 0f, 1f);
        return a + ab * t;
    }

    private static float AreaMultiplier(Navmesh.AreaId area)
    {
        // Area multipliers for off-mesh links mirror the original QuadGraph
        // pathfinder's cost weights.
        if (area == Navmesh.AreaId.Warp) return 1f;
        if (area == Navmesh.AreaId.ClientPath) return 3f;
        if (area == Navmesh.AreaId.Shortcut) return 8f;
        if (area == Navmesh.AreaId.Default) return 1f;
        return 10f;
    }

    private float Heuristic(Vector2 root)
    {
        return Vector2.Distance(root, _target) * 0.999f;
    }

    // ---- Path reconstruction ----

    private List<Vector3> ReconstructPath(int goalNode, Vector3 from, Vector3 to)
    {
        // The Polyanya path is a sequence of straight segments between turning
        // points. For an observable (straight) goal, the path is [from, to]. For
        // a non-observable (bent) goal, the path is [from, turn, to] where turn
        // is the goal node's root (the bending point on the entry interval).
        // Off-mesh link transitions insert their toPos as intermediate waypoints.
        var waypoints = new List<Vector3>();
        int cur = goalNode;
        var span = CollectionsMarshal.AsSpan(_nodes);
        // Collect off-mesh link landing points and the goal turning point in
        // root-to-goal order (reverse of parent walk).
        while (cur > 0)
        {
            ref var n = ref span[cur];
            // An off-mesh link fresh-seed node (Edge == -1, parent in a different
            // face) contributes its root as a landing-point waypoint.
            if (n.Edge < 0 && n.Parent > 0 && span[n.Parent].Face != n.Face)
            {
                float y = _mesh.Faces[n.Face].Y;
                waypoints.Add(new Vector3(n.Root.X, y, n.Root.Y));
            }
            cur = n.Parent;
        }
        // The goal node itself: if it is a bent goal (root != goal start and root
        // is a turning point on an entry interval), insert the turning point.
        ref var goal = ref span[goalNode];
        if (goal.IsGoal)
        {
            Vector2 goalStart = new(from.X, from.Z);
            Vector2 goalEnd = goal.GoalPoint;
            // If the goal's root differs from the start and from the goal point,
            // it is a turning point (bent path).
            if (Vector2.Distance(goal.Root, goalStart) > 1e-3f && Vector2.Distance(goal.Root, goalEnd) > 1e-3f)
            {
                float y = _mesh.Faces[goal.Face].Y;
                waypoints.Add(new Vector3(goal.Root.X, y, goal.Root.Y));
            }
        }
        waypoints.Reverse();

        var result = new List<Vector3> { from };
        foreach (var wp in waypoints)
        {
            if (Vector3.Distance(result[^1], wp) > 1e-4f)
                result.Add(wp);
        }
        Vector2 finalPt = goal.IsGoal ? goal.GoalPoint : new Vector2(to.X, to.Z);
        var end = new Vector3(finalPt.X, to.Y, finalPt.Y);
        if (Vector3.Distance(result[^1], end) > 1e-4f)
            result.Add(end);
        return result;
    }

    // ---- Binary heap ----

    private void AddToOpen(int nodeIndex)
    {
        ref var node = ref CollectionsMarshal.AsSpan(_nodes)[nodeIndex];
        if (node.OpenHeapIndex < 0)
        {
            node.OpenHeapIndex = _open.Count;
            _open.Add(nodeIndex);
        }
        PercolateUp(node.OpenHeapIndex);
    }

    private int PopMinOpen()
    {
        var span = CollectionsMarshal.AsSpan(_nodes);
        int nodeIndex = _open[0];
        _open[0] = _open[^1];
        _open.RemoveAt(_open.Count - 1);
        span[nodeIndex].OpenHeapIndex = -1;
        if (_open.Count > 0)
        {
            span[_open[0]].OpenHeapIndex = 0;
            PercolateDown(0);
        }
        return nodeIndex;
    }

    private void PercolateUp(int heapIndex)
    {
        var span = CollectionsMarshal.AsSpan(_nodes);
        int nodeIndex = _open[heapIndex];
        int parent = (heapIndex - 1) >> 1;
        while (heapIndex > 0 && span[nodeIndex].F < span[_open[parent]].F)
        {
            _open[heapIndex] = _open[parent];
            span[_open[heapIndex]].OpenHeapIndex = heapIndex;
            heapIndex = parent;
            parent = (heapIndex - 1) >> 1;
        }
        _open[heapIndex] = nodeIndex;
        span[nodeIndex].OpenHeapIndex = heapIndex;
    }

    private void PercolateDown(int heapIndex)
    {
        var span = CollectionsMarshal.AsSpan(_nodes);
        int nodeIndex = _open[heapIndex];
        int maxSize = _open.Count;
        while (true)
        {
            int child1 = (heapIndex << 1) + 1;
            if (child1 >= maxSize)
                break;
            int child2 = child1 + 1;
            int bestChild = child2 == maxSize || span[_open[child1]].F <= span[_open[child2]].F ? child1 : child2;
            if (span[_open[bestChild]].F < span[nodeIndex].F)
            {
                _open[heapIndex] = _open[bestChild];
                span[_open[heapIndex]].OpenHeapIndex = heapIndex;
                heapIndex = bestChild;
            }
            else
                break;
        }
        _open[heapIndex] = nodeIndex;
        span[nodeIndex].OpenHeapIndex = heapIndex;
    }
}