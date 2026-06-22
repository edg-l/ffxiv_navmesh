using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading;

namespace Navmesh.GroundGraph;

public class QuadPathfind
{
    public struct QNode
    {
        public float GScore;
        public float HScore;
        public int QuadId;
        public int ParentIndex;
        public int OpenHeapIndex;
        public Vector3 EnterPos;
    }

    private readonly QuadGraph _graph;
    private readonly List<QNode> _nodes = new();
    private int[] _lookupTable = Array.Empty<int>();
    private int _lookupMask;
    private readonly List<int> _openList = new();
    private int _bestNodeIndex;
    private int _goalQuad;
    private Vector3 _goalPos;
    private bool _useRaycast;
    private float _raycastLimitSq = float.MaxValue;
    private float _goalRange;

    public QuadPathfind(QuadGraph graph)
    {
        _graph = graph;
    }

    public List<(int quad, Vector3 p)> FindPath(int fromQuad, int toQuad, Vector3 fromPos, Vector3 toPos, bool useRaycast, CancellationToken cancel)
    {
        _useRaycast = useRaycast;
        _goalRange = 0;
        Start(fromQuad, toQuad, fromPos, toPos);
        Execute(cancel);
        return BuildPathToVisitedNode(_bestNodeIndex);
    }

    public List<(int quad, Vector3 p)> FindPathWithRange(int fromQuad, int toQuad, Vector3 fromPos, Vector3 toPos, float range, bool useRaycast, CancellationToken cancel)
    {
        _useRaycast = useRaycast;
        _goalRange = range;
        Start(fromQuad, toQuad, fromPos, toPos);
        Execute(cancel);
        return BuildPathToVisitedNode(_bestNodeIndex);
    }

    private void Start(int fromQuad, int toQuad, Vector3 fromPos, Vector3 toPos)
    {
        _nodes.Clear();
        InitLookup(Math.Max(4096, _graph.Count));
        _openList.Clear();
        _bestNodeIndex = 0;
        _goalQuad = toQuad;
        _goalPos = toPos;

        _nodes.Add(new() { HScore = HeuristicDistance(fromPos), QuadId = fromQuad, ParentIndex = 0, OpenHeapIndex = -1, EnterPos = fromPos });
        LookupSet(fromQuad, 0);
        AddToOpen(0);
    }

    private void Execute(CancellationToken cancel, int maxSteps = 500000)
    {
        for (int i = 0; i < maxSteps; ++i)
        {
            if (!ExecuteStep())
                return;
            if ((i & 0x3ff) == 0)
                cancel.ThrowIfCancellationRequested();
        }
    }

    private bool ExecuteStep()
    {
        var nodeSpan = CollectionsMarshal.AsSpan(_nodes);
        if (_openList.Count == 0 || nodeSpan[_bestNodeIndex].HScore <= 0)
            return false;

        var curNodeIndex = PopMinOpen();
        ref var curNode = ref nodeSpan[curNodeIndex];
        var curQuad = curNode.QuadId;

        if (_goalRange > 0 && (curNode.EnterPos - _goalPos).Length() <= _goalRange)
        {
            _bestNodeIndex = curNodeIndex;
            return false;
        }

        if (curQuad == _goalQuad)
        {
            _bestNodeIndex = curNodeIndex;
            return false;
        }

        foreach (var neighbourQuad in _graph.Adjacency[curQuad])
            VisitNeighbour(curNodeIndex, neighbourQuad);

        for (int p = 0; p < _graph.Portals.Count; ++p)
        {
            var portal = _graph.Portals[p];
            if (portal.IsOffMesh && portal.FromQuad == curQuad)
                VisitNeighbour(curNodeIndex, portal.ToQuad, portal.Area);
        }

        return true;
    }

    private void VisitNeighbour(int parentIndex, int destQuad, Navmesh.AreaId portalArea = Navmesh.AreaId.Default)
    {
        var nodeIndex = LookupGet(destQuad);
        if (nodeIndex < 0)
        {
            nodeIndex = _nodes.Count;
            _nodes.Add(new() { GScore = float.MaxValue, HScore = float.MaxValue, QuadId = destQuad, ParentIndex = parentIndex, OpenHeapIndex = -1 });
            LookupSet(destQuad, nodeIndex);
            if (_nodes.Count > _lookupTable.Length / 2)
                LookupGrow();
        }
        else
        {
            ref var existing = ref CollectionsMarshal.AsSpan(_nodes)[nodeIndex];
            if (existing.OpenHeapIndex < 0)
                return;
        }

        var nodeSpan = CollectionsMarshal.AsSpan(_nodes);
        ref var parentNode = ref nodeSpan[parentIndex];
        var destQuadRecord = _graph.Quads[destQuad];
        var enterPos = destQuad == _goalQuad ? _goalPos : ClampToQuad(destQuadRecord, parentNode.EnterPos);
        var parentIndexCopy = parentIndex;
        var nodeG = CalculateGScore(ref parentNode, destQuad, enterPos, ref parentIndexCopy, portalArea);
        ref var curNode = ref nodeSpan[nodeIndex];
        if (nodeG + 0.00001f < curNode.GScore)
        {
            curNode.GScore = nodeG;
            curNode.HScore = HeuristicDistance(enterPos);
            curNode.ParentIndex = parentIndexCopy;
            curNode.EnterPos = enterPos;
            AddToOpen(nodeIndex);

            if (curNode.HScore < nodeSpan[_bestNodeIndex].HScore)
                _bestNodeIndex = nodeIndex;
        }
    }

    private float CalculateGScore(ref QNode parent, int destQuad, Vector3 destPos, ref int parentIndex, Navmesh.AreaId portalArea)
    {
        float baseDistance;
        float parentBaseG;
        Vector3 fromPos;

        if (_useRaycast && parent.ParentIndex != parentIndex)
        {
            int grandParentIndex = parent.ParentIndex;
            ref var grandParentNode = ref CollectionsMarshal.AsSpan(_nodes)[grandParentIndex];
            var distSq = (grandParentNode.EnterPos - destPos).LengthSquared();
            if (distSq <= _raycastLimitSq && HasLineOfSight(grandParentNode.EnterPos, destPos))
            {
                parentIndex = grandParentIndex;
                baseDistance = MathF.Sqrt(distSq);
                parentBaseG = grandParentNode.GScore;
                fromPos = grandParentNode.EnterPos;
            }
            else
            {
                baseDistance = (parent.EnterPos - destPos).Length();
                parentBaseG = parent.GScore;
                fromPos = parent.EnterPos;
            }
        }
        else
        {
            baseDistance = (parent.EnterPos - destPos).Length();
            parentBaseG = parent.GScore;
            fromPos = parent.EnterPos;
        }

        float verticalPenalty = 0.2f * MathF.Abs(fromPos.Y - destPos.Y);

        float areaMultiplier = portalArea switch
        {
            Navmesh.AreaId.Warp => 1f,
            Navmesh.AreaId.ClientPath => 3f,
            Navmesh.AreaId.Shortcut => 8f,
            _ => 10f
        };
        if (portalArea != Navmesh.AreaId.Default)
            baseDistance *= areaMultiplier;

        return parentBaseG + baseDistance + verticalPenalty;
    }

    private bool HasLineOfSight(Vector3 from, Vector3 to)
    {
        var dir = to - from;
        var dist = dir.Length();
        if (dist < 0.001f)
            return true;
        var dirNorm = dir / dist;
        foreach (var quad in _graph.Quads)
        {
            if (SegmentQuadXZIntersect(from, to, quad))
                return false;
        }
        return true;
    }

    private static bool SegmentQuadXZIntersect(Vector3 a, Vector3 b, Quad q)
    {
        float minX = q.MinX, maxX = q.MaxX, minZ = q.MinZ, maxZ = q.MaxZ;
        var d = b - a;
        float tMin = 0f, tMax = 1f;
        for (int axis = 0; axis < 2; ++axis)
        {
            float aVal = axis == 0 ? a.X : a.Z;
            float dVal = axis == 0 ? d.X : d.Z;
            float lo = axis == 0 ? minX : minZ;
            float hi = axis == 0 ? maxX : maxZ;
            if (MathF.Abs(dVal) < 1e-6f)
            {
                if (aVal < lo || aVal > hi)
                    return false;
            }
            else
            {
                float t1 = (lo - aVal) / dVal;
                float t2 = (hi - aVal) / dVal;
                if (t1 > t2) (t1, t2) = (t2, t1);
                tMin = MathF.Max(tMin, t1);
                tMax = MathF.Min(tMax, t2);
                if (tMin > tMax)
                    return false;
            }
        }
        return tMin <= tMax;
    }

    private static Vector3 ClampToQuad(Quad q, Vector3 p)
    {
        return new Vector3(
            Math.Clamp(p.X, q.MinX, q.MaxX),
            q.MinY,
            Math.Clamp(p.Z, q.MinZ, q.MaxZ));
    }

    private float HeuristicDistance(Vector3 pos) => (pos - _goalPos).Length() * 0.999f;

    private List<(int quad, Vector3 p)> BuildPathToVisitedNode(int nodeIndex)
    {
        var res = new List<(int quad, Vector3 p)>();
        if (nodeIndex < _nodes.Count)
        {
            var nodeSpan = CollectionsMarshal.AsSpan(_nodes);
            ref var lastNode = ref nodeSpan[nodeIndex];
            res.Add((lastNode.QuadId, lastNode.EnterPos));
            while (nodeSpan[nodeIndex].ParentIndex != nodeIndex)
            {
                ref var prevNode = ref nodeSpan[nodeIndex];
                var nextIndex = prevNode.ParentIndex;
                ref var nextNode = ref nodeSpan[nextIndex];
                res.Add((nextNode.QuadId, nextNode.EnterPos));
                nodeIndex = nextIndex;
            }
            res.Reverse();
        }
        return res;
    }

    private static int QuadHash(int id)
    {
        uint v = (uint)id;
        v ^= v >> 16;
        v *= 0x45d9f3bU;
        v ^= v >> 16;
        return (int)v;
    }

    private void InitLookup(int capacity)
    {
        int size = 1;
        while (size < capacity * 2)
            size <<= 1;
        if (_lookupTable.Length < size)
            _lookupTable = new int[size];
        Array.Fill(_lookupTable, -1);
        _lookupMask = size - 1;
    }

    private int LookupGet(int quadId)
    {
        int i = QuadHash(quadId) & _lookupMask;
        while (_lookupTable[i] >= 0)
        {
            if (_nodes[_lookupTable[i]].QuadId == quadId)
                return _lookupTable[i];
            i = (i + 1) & _lookupMask;
        }
        return -1;
    }

    private void LookupSet(int quadId, int nodeIndex)
    {
        int i = QuadHash(quadId) & _lookupMask;
        while (_lookupTable[i] >= 0)
            i = (i + 1) & _lookupMask;
        _lookupTable[i] = nodeIndex;
    }

    private void LookupGrow()
    {
        var oldTable = _lookupTable;
        int newSize = oldTable.Length << 1;
        _lookupTable = new int[newSize];
        Array.Fill(_lookupTable, -1);
        _lookupMask = newSize - 1;
        var nodeSpan = CollectionsMarshal.AsSpan(_nodes);
        for (int n = 0; n < _nodes.Count; ++n)
            LookupSet(nodeSpan[n].QuadId, n);
    }

    private void AddToOpen(int nodeIndex)
    {
        ref var node = ref CollectionsMarshal.AsSpan(_nodes)[nodeIndex];
        if (node.OpenHeapIndex < 0)
        {
            node.OpenHeapIndex = _openList.Count;
            _openList.Add(nodeIndex);
        }
        PercolateUp(node.OpenHeapIndex);
    }

    private int PopMinOpen()
    {
        var nodeSpan = CollectionsMarshal.AsSpan(_nodes);
        int nodeIndex = _openList[0];
        _openList[0] = _openList[^1];
        _openList.RemoveAt(_openList.Count - 1);
        nodeSpan[nodeIndex].OpenHeapIndex = -1;
        if (_openList.Count > 0)
        {
            nodeSpan[_openList[0]].OpenHeapIndex = 0;
            PercolateDown(0);
        }
        return nodeIndex;
    }

    private void PercolateUp(int heapIndex)
    {
        var nodeSpan = CollectionsMarshal.AsSpan(_nodes);
        int nodeIndex = _openList[heapIndex];
        int parent = (heapIndex - 1) >> 1;
        while (heapIndex > 0 && HeapLess(ref nodeSpan[nodeIndex], ref nodeSpan[_openList[parent]]))
        {
            _openList[heapIndex] = _openList[parent];
            nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
            heapIndex = parent;
            parent = (heapIndex - 1) >> 1;
        }
        _openList[heapIndex] = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    private void PercolateDown(int heapIndex)
    {
        var nodeSpan = CollectionsMarshal.AsSpan(_nodes);
        int nodeIndex = _openList[heapIndex];
        int maxSize = _openList.Count;
        while (true)
        {
            int child1 = (heapIndex << 1) + 1;
            if (child1 >= maxSize)
                break;
            int child2 = child1 + 1;
            if (child2 == maxSize || HeapLess(ref nodeSpan[_openList[child1]], ref nodeSpan[_openList[child2]]))
            {
                if (HeapLess(ref nodeSpan[_openList[child1]], ref nodeSpan[nodeIndex]))
                {
                    _openList[heapIndex] = _openList[child1];
                    nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
                    heapIndex = child1;
                }
                else
                    break;
            }
            else if (HeapLess(ref nodeSpan[_openList[child2]], ref nodeSpan[nodeIndex]))
            {
                _openList[heapIndex] = _openList[child2];
                nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
                heapIndex = child2;
            }
            else
                break;
        }
        _openList[heapIndex] = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    private static bool HeapLess(ref QNode nodeL, ref QNode nodeR)
    {
        var fl = nodeL.GScore + nodeL.HScore;
        var fr = nodeR.GScore + nodeR.HScore;
        if (fl + 0.00001f < fr)
            return true;
        if (fr + 0.00001f < fl)
            return false;
        return nodeL.GScore > nodeR.GScore;
    }
}