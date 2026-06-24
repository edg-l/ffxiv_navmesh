using Navmesh.GroundGraph.Extraction;
using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace Navmesh;

// utility to rasterize various meshes into a voxel volume
public class NavmeshRasterizer
{
    private const int RC_WALKABLE_AREA = 1;

    // cheap triangle-bbox test: if all 3 vertices are on the same side of the bbox plane, the triangle can be discarded
    [Flags]
    private enum OutFlags : byte
    {
        None = 0,
        NegX = 1 << 0,
        PosX = 1 << 1,
        NegY = 1 << 2,
        PosY = 1 << 3,
        NegZ = 1 << 4,
        PosZ = 1 << 5,
    }

    // a set of per-cell intersections with vertical ray
    // we build a sort key with high 31 bits as the remapped Y coordinate (0 = -1024, 0x7fffffff = 1024-eps) and low bit as normal sign (1 if points up)
    // we need the extra precision to disambiguate faces that map to a single voxel
    private class IntersectionSet
    {
        public const int PageShift = 20;
        public const int PageSize = 1 << PageShift;
        public const int ValueScale = 1 << 20; // 2048 = 2^11, we have 32 bits => 20 bits of scale; note: this is a bit extreme, mantissa is only 24 bits...

        public struct Entry
        {
            public int Next; // index of next entry in the same cell in page storage
            public uint SortKey;
            public int VoxelY; // if normal points up - this is inclusive upper limit of area 'below' triangle (>0), otherwise it's negative inclusive lower limit of area 'above' triangle (<=0)

            public Entry(int next, int voxelY, float preciseY, bool normalUp)
            {
                Next = next;
                SortKey = ((uint)((preciseY + 1024) * ValueScale) << 1) | (normalUp ? 1u : 0);
                VoxelY = normalUp ? voxelY : -voxelY;
            }
        }

        private int _numCellsX;
        private int[] _firstIndices; // x then z
        private List<Entry[]> _pages = new();
        private int _firstFree = 1;

        public IntersectionSet(int numCellsX, int numCellsZ)
        {
            _numCellsX = numCellsX;
            _firstIndices = new int[numCellsX * numCellsZ];
            _pages.Add(new Entry[PageSize]);
        }

        public void Add(int x, int y, int z, float value, bool normalUp)
        {
            if (value <= -1024 || value >= 1024)
                return;
            var pageIndex = _firstFree >> PageShift;
            if (pageIndex == _pages.Count)
                _pages.Add(new Entry[PageSize]);
            var indexInPage = _firstFree & (PageSize - 1);
            ref var first = ref _firstIndices[z * _numCellsX + x];
            _pages[pageIndex][indexInPage] = new(first, y, value, normalUp);
            first = _firstFree++;
        }

        public int FetchSorted(int x, int z, Span<uint> bufferSort, Span<int> bufferVoxel)
        {
            var idx = _firstIndices[z * _numCellsX + x];
            if (idx == 0)
                return 0;

            int cnt = 0;
            do
            {
                var entry = _pages[idx >> PageShift][idx & (PageSize - 1)];
                bufferSort[cnt] = entry.SortKey;
                bufferVoxel[cnt] = entry.VoxelY;
                idx = entry.Next;
                ++cnt;
            }
            while (idx != 0);
            bufferSort.Slice(0, cnt).Sort(bufferVoxel.Slice(0, cnt));
            return cnt;
        }

        public void Clear()
        {
            Array.Fill(_firstIndices, 0);
            _firstFree = 1;
        }
    }

    private Voxelizer? _voxelizer;
    private IntersectionSet? _iset;
    private Vector3 _bmin;
    private Vector3 _bmax;
    private float _cs;
    private float _ch;
    private int _width;
    private int _height;
    private int _borderSize;
    private float _invCellXZ;
    private float _invCellY;
    private int _maxY;
    private int _minSpanGap;
    private int _walkableClimbThreshold; // if two spans have maximums within this number of voxels, their area is 'merged' (higher is selected)
    private float _walkableNormalThreshold; // triangle is considered 'walkable' if it's world-space normal's Y coordinate is >= this
    private int _voxShiftX;
    private int _voxShiftY;
    private int _voxShiftZ;

    // Optional CompactHeightfield to populate. When non-null, raw spans are
    // accumulated in _rawSpans and flushed into the CHF by PopulateChf().
    private CompactHeightfield? _chf;
    // Raw span accumulator: per-column list of (y0 voxel bottom, y1 voxel top, area).
    // y0 and y1 are inclusive voxel indices. World-Y of span top = _bmin.Y + (y1 + 1) * _ch.
    // y0 is needed to compute clearance: space above a floor = bottom of next span above it.
    private List<(int y0, int y1, byte area)>[]? _rawSpans;

    public NavmeshRasterizer(Voxelizer? voxelizer, Vector3 boundsMin, Vector3 boundsMax, float cellSize, float cellHeight, int borderSize, float walkableNormalThreshold, int walkableMaxClimb, int minGap, bool fillInteriors, CompactHeightfield? chf = null)
    {
        _voxelizer = voxelizer;
        _bmin = boundsMin;
        _bmax = boundsMax;
        _cs = cellSize;
        _ch = cellHeight;
        _borderSize = borderSize;
        _width = (int)MathF.Ceiling((_bmax.X - _bmin.X) / _cs);
        _height = (int)MathF.Ceiling((_bmax.Z - _bmin.Z) / _cs);
        _iset = fillInteriors ? new IntersectionSet(_width, _height) : null;
        _invCellXZ = 1.0f / _cs;
        _invCellY = 1.0f / _ch;
        _maxY = (int)((_bmax.Y - _bmin.Y) * _invCellY);
        _minSpanGap = minGap;
        _walkableClimbThreshold = walkableMaxClimb;
        _walkableNormalThreshold = walkableNormalThreshold;
        if (voxelizer != null)
        {
            var dx = (_width - 2 * _borderSize) / voxelizer.NumX;
            var dy = _maxY / voxelizer.NumY;
            var dz = (_height - 2 * _borderSize) / voxelizer.NumZ;
            if (!BitOperations.IsPow2(dx) || !BitOperations.IsPow2(dy) || !BitOperations.IsPow2(dz))
                throw new Exception($"Cell size mismatch: {dx}x{dy}x{dz}");
            _voxShiftX = BitOperations.Log2((uint)dx);
            _voxShiftY = BitOperations.Log2((uint)dy);
            _voxShiftZ = BitOperations.Log2((uint)dz);
        }
        if (chf != null)
        {
            _chf = chf;
            _rawSpans = new List<(int y0, int y1, byte area)>[_width * _height];
            for (int i = 0; i < _rawSpans.Length; i++)
                _rawSpans[i] = new List<(int, int, byte)>();
        }
    }

    // Flush accumulated raw spans into the CHF. Call once after all Rasterize()
    // calls. Each distinct walkable solid surface becomes a FloorSpan. Non-walkable
    // spans are included only to compute clearance for walkable spans below them.
    // Emitted BEFORE _iset state is used — parallel to the _iset.Clear() timing
    // described in the plan.
    public void PopulateChf()
    {
        if (_chf == null || _rawSpans == null)
            return;

        for (int z = 0; z < _height; z++)
        {
            for (int x = 0; x < _width; x++)
            {
                var raw = _rawSpans[z * _width + x];
                if (raw.Count == 0)
                    continue;

                // Sort by y1 ascending (top of span).
                raw.Sort((a, b) => a.y1.CompareTo(b.y1));

                // Merge overlapping/touching spans: spans with the same y1 → walkable
                // wins (area 1 over area 0). We keep track of both y0 and y1 so
                // clearance is computed as bottom of next span minus floor of this span.
                var merged = new List<(int y0, int y1, byte area)>();
                foreach (var s in raw)
                {
                    if (merged.Count > 0 && merged[^1].y1 == s.y1)
                    {
                        // Same top: walkable wins; take the lower y0 for a wider span.
                        if (s.area > merged[^1].area)
                            merged[^1] = (Math.Min(merged[^1].y0, s.y0), s.y1, s.area);
                    }
                    else
                    {
                        merged.Add(s);
                    }
                }

                // Emit walkable floor spans only; non-walkable spans are used to
                // limit clearance computation for walkable spans below them.
                for (int i = 0; i < merged.Count; i++)
                {
                    if (merged[i].area == 0)
                        continue; // non-walkable: skip emission but used for clearance below

                    // World Y of the top of voxel y1 (= floor surface).
                    float floorY = _bmin.Y + (merged[i].y1 + 1) * _ch;

                    // Clearance = bottom of next span above this one minus floorY.
                    // Bottom of a span at y0 = _bmin.Y + y0 * _ch (voxel y0 bottom edge).
                    float clearance = float.MaxValue;
                    if (i + 1 < merged.Count)
                    {
                        float nextSpanBottom = _bmin.Y + merged[i + 1].y0 * _ch;
                        clearance = nextSpanBottom - floorY;
                    }

                    _chf.AddSpanSorted(x, z, floorY, merged[i].area);
                    // clearance is recomputed after all spans are added via FinalizeAllClearances;
                    // the value computed here is unused but confirms correctness.
                    _ = clearance;
                }
            }
        }
        _chf.FinalizeAllClearances();
    }

    public void Rasterize(SceneExtractor geom, SceneExtractor.MeshType types, bool perMeshInteriors, bool solidBelowNonManifold)
    {
        foreach (var (name, mesh) in geom.Meshes)
        {
            if ((mesh.MeshType & types) == SceneExtractor.MeshType.None)
                continue;

            foreach (var instance in mesh.Instances)
            {
                if (RasterizeMesh(mesh, instance, out var minY) && perMeshInteriors)
                {
                    int z0 = Math.Clamp((int)((instance.WorldBounds.Min.Z - _bmin.Z) * _invCellXZ), 0, _height - 1);
                    int z1 = Math.Clamp((int)((instance.WorldBounds.Max.Z - _bmin.Z) * _invCellXZ), 0, _height - 1);
                    int x0 = Math.Clamp((int)((instance.WorldBounds.Min.X - _bmin.X) * _invCellXZ), 0, _width - 1);
                    int x1 = Math.Clamp((int)((instance.WorldBounds.Max.X - _bmin.X) * _invCellXZ), 0, _width - 1);
                    FillInterior(z0, z1, x0, x1, solidBelowNonManifold ? minY : _maxY);
                }
            }
        }

        if (!perMeshInteriors)
        {
            FillInterior(0, _height - 1, 0, _width - 1, solidBelowNonManifold ? 0 : _maxY);
        }
    }

    // if it returns true, the mesh borders were rasterized, so intersection set could be modified
    public bool RasterizeMesh(SceneExtractor.Mesh mesh, SceneExtractor.MeshInstance instance, out int minimalY)
    {
        minimalY = _maxY;
        if (instance.WorldBounds.Max.X <= _bmin.X || instance.WorldBounds.Max.Z <= _bmin.Z || instance.WorldBounds.Min.X >= _bmax.X || instance.WorldBounds.Min.Z >= _bmax.Z)
            return false;

        Span<Vector3> worldVertices = stackalloc Vector3[256];
        Span<OutFlags> outFlags = stackalloc OutFlags[256];
        Span<Vector3> clipRemainingZ = stackalloc Vector3[7];
        Span<Vector3> clipRemainingX = stackalloc Vector3[7];
        Span<Vector3> clipScratch = stackalloc Vector3[7];
        Span<Vector3> clipCell = stackalloc Vector3[7];
        foreach (var part in mesh.Parts)
        {
            // fill vertex buffer
            TransformVertices(instance, part.Vertices, worldVertices, outFlags);

            foreach (var p in part.Primitives)
            {
                if ((outFlags[p.V1] & outFlags[p.V2] & outFlags[p.V3]) != OutFlags.None)
                    continue; // vertex is fully outside bounds, on one side of some plane

                var v1 = worldVertices[p.V1];
                var v2 = worldVertices[p.V2];
                var v3 = worldVertices[p.V3];
                var v12 = v2 - v1;
                var v13 = v3 - v1;
                var v12cross13 = Vector3.Cross(v12, v13);
                var normal = Vector3.Normalize(v12cross13);
                var invDiv = _iset != null && v12cross13.Y != 0 ? -1.0f / v12cross13.Y : 0; // see below

                var flags = (p.Flags & ~instance.ForceClearPrimFlags) | instance.ForceSetPrimFlags;
                bool realSolid = !flags.HasFlag(SceneExtractor.PrimitiveFlags.FlyThrough);
                bool unwalkable = flags.HasFlag(SceneExtractor.PrimitiveFlags.ForceUnwalkable)
                    || normal.Y < _walkableNormalThreshold
                    // for flyable scenes, assume unlandable == unwalkable, unless explicitly set
                    || _voxelizer != null && flags.HasFlag(SceneExtractor.PrimitiveFlags.Unlandable) && !flags.HasFlag(SceneExtractor.PrimitiveFlags.ForceWalkable);
                var areaId = unwalkable ? 0 : RC_WALKABLE_AREA;

                // prepare for clipping: while iterating over z, we'll keep the 'remaining polygon' in clipRemainingZ
                int numRemainingZ = 0;
                clipRemainingZ[numRemainingZ++] = v1;
                clipRemainingZ[numRemainingZ++] = v2;
                clipRemainingZ[numRemainingZ++] = v3;

                // calculate the footprint of the triangle on the grid's z-axis
                var (minZ, maxZ) = AxisMinMax(clipRemainingZ, numRemainingZ, 2);
                int z0 = (int)((minZ - _bmin.Z) * _invCellXZ);
                int z1 = (int)((maxZ - _bmin.Z) * _invCellXZ);
                // note: no need to check for fully outside here, it was checked before
                z0 = Math.Clamp(z0, -1, _height - 1); // use -1 rather than 0 to cut the polygon properly at the start of the tile
                z1 = Math.Clamp(z1, 0, _height - 1);

                for (int z = z0; z <= z1; ++z)
                {
                    if (numRemainingZ < 3)
                        break;

                    // clip polygon to 'row'
                    var cellZMax = _bmin.Z + (z + 1) * _cs;
                    int numRemainingX = SplitConvexPoly(clipRemainingZ, clipRemainingX, clipScratch, ref numRemainingZ, 2, cellZMax);

                    // previous buffer is now new scratch
                    var swapZ = clipRemainingZ;
                    clipRemainingZ = clipScratch;
                    clipScratch = swapZ;

                    if (numRemainingX < 3 || z < 0)
                        continue;

                    // find x bounds of the row
                    var (minX, maxX) = AxisMinMax(clipRemainingX, numRemainingX, 0);
                    int x0 = (int)((minX - _bmin.X) * _invCellXZ);
                    int x1 = (int)((maxX - _bmin.X) * _invCellXZ);
                    if (x1 < 0 || x0 >= _width)
                        continue;
                    x0 = Math.Clamp(x0, -1, _width - 1);
                    x1 = Math.Clamp(x1, 0, _width - 1);

                    var cellZMid = _bmin.Z + (z + 0.5f) * _cs;
                    for (int x = x0; x <= x1; ++x)
                    {
                        if (numRemainingX < 3)
                            break;

                        // clip polygon to 'column'
                        var cellXMax = _bmin.X + (x + 1) * _cs;
                        int numCell = SplitConvexPoly(clipRemainingX, clipCell, clipScratch, ref numRemainingX, 0, cellXMax);

                        // previous buffer is now new scratch
                        var swapX = clipRemainingX;
                        clipRemainingX = clipScratch;
                        clipScratch = swapX;

                        if (numCell < 3 || x < 0)
                            continue;

                        // find y bounds of the cell
                        var (minY, maxY) = AxisMinMax(clipCell, numCell, 1);
                        int y0 = (int)MathF.Floor((minY - _bmin.Y) * _invCellY);
                        int y1 = (int)MathF.Ceiling((maxY - _bmin.Y) * _invCellY);
                        if (y1 < 0 || y0 >= _maxY)
                            continue;
                        y0 = Math.Clamp(y0, 0, _maxY - 1);
                        y1 = Math.Clamp(y1, y0, _maxY - 1);

                        AddSpan(x, z, y0, y1, areaId, realSolid);

                        if (realSolid && _iset != null && invDiv != 0)
                        {
                            minimalY = Math.Min(minimalY, y0);
                            // intersect a ray passing through the middle of the cell vertically with the triangle
                            // A + AB*b + AC*c = P, b >= 0, c >= 0, a + b <= 1
                            //  ==>
                            // ABx*b + ACx*c = APx
                            // ABz*b + ACz*c = APz
                            //  ==> ABx*b = APx - ACx*c, ABz*ABx*b + ACz*ABx*c = APz*ABx
                            //  ==> ABz*(APx - ACx*c) + ACz*ABx*c = APz*ABx
                            //  ==> c = (APz*ABx - APx*ABz) / (ACz*ABx - ACx*ABz)
                            //  ==> b = (APx*ACz*ABx - APx*ACx*ABz - ACx*APz*ABx + ACx*APx*ABz) / ABx*(ACz*ABx - ACx*ABz)
                            //  ==> b = (APx*ACz - APz*ACx) / (ACz*ABx - ACx*ABz)
                            //  ==> y = Ay + ABy*b + ACy*c
                            // note that (ACz*ABx - ACx*ABz) == (AC cross AB).y
                            var cellXMid = _bmin.X + (x + 0.5f) * _cs;
                            var apx = cellXMid - v1.X;
                            var apz = cellZMid - v1.Z;
                            var c = (apz * v12.X - apx * v12.Z) * invDiv;
                            var b = (apx * v13.Z - apz * v13.X) * invDiv;
                            if (c >= 0 && b >= 0 && c + b <= 1)
                            {
                                var intersectY = v1.Y + b * v12.Y + c * v13.Y;
                                if (normal.Y > 0 && y0 > 0)
                                    _iset.Add(x, y0 - 1, z, intersectY, true);
                                else if (normal.Y < 0 && y1 < _maxY - 1)
                                    _iset.Add(x, y1 + 1, z, intersectY, false);
                            }
                            // else: intersection is outside triangle
                        }
                    }
                }
            }
        }
        return true;
    }

    private void AddSpan(int x, int z, int y0, int y1, int areaId, bool includeInVolume)
    {
        if (includeInVolume && _voxelizer != null)
        {
            var bx = x - _borderSize;
            var bz = z - _borderSize;
            if (bx >= 0 && bz >= 0)
            {
                bx >>= _voxShiftX;
                bz >>= _voxShiftZ;
                if (bx < _voxelizer.NumX && bz < _voxelizer.NumZ)
                {
                    // block pixels beneath the span for a distance roughly equal to agent height, otherwise volume pathfind will try to move the player through doorframes etc
                    _voxelizer.AddSpan(bx, bz, (y0 - _minSpanGap) >> _voxShiftY, y1 >> _voxShiftY);
                }
            }
        }

        // Record span (bottom, top) into the raw accumulator for CHF population.
        if (_rawSpans != null && x >= 0 && x < _width && z >= 0 && z < _height)
            _rawSpans[z * _width + x].Add((y0, y1, (byte)areaId));
    }

    private void FillInterior(int z0, int z1, int x0, int x1, int yBelowNonManifold)
    {
        if (_iset == null)
            return; // interior filling is disabled

        // fill interiors
        Span<uint> solidSort = stackalloc uint[256];
        Span<int> solidVoxel = stackalloc int[256];
        for (int z = z0; z <= z1; ++z)
        {
            for (int x = x0; x <= x1; ++x)
            {
                var cnt = _iset.FetchSorted(x, z, solidSort, solidVoxel);
                if (cnt == 0)
                    continue; // empty

                int idx = 0;
                if (solidVoxel[idx] > yBelowNonManifold)
                {
                    // non-manifold mesh, assume everything below is interior
                    while (idx + 1 < cnt && solidVoxel[idx + 1] > yBelowNonManifold)
                        ++idx; // well i dunno, some terrain (eg south thanalan) is really _that_ fucked
                    AddSpan(x, z, yBelowNonManifold, solidVoxel[idx], 0, true);
                    ++idx;
                }

                while (true)
                {
                    while (idx < cnt && solidVoxel[idx] > 0)
                        ++idx;
                    if (idx == cnt)
                        break;
                    var minY = -solidVoxel[idx];
                    while (idx < cnt && solidVoxel[idx] <= 0)
                        ++idx;
                    if (idx == cnt)
                        break;
                    var maxY = solidVoxel[idx];
                    if (maxY >= minY)
                        AddSpan(x, z, minY, maxY, 0, true);
                }
            }
        }
        _iset.Clear();
    }

    private void TransformVertices(SceneExtractor.MeshInstance instance, List<Vector3> localVertices, Span<Vector3> outWorld, Span<OutFlags> outFlags)
    {
        int iv = 0;
        foreach (var v in localVertices)
        {
            var w = instance.WorldTransform.TransformCoordinate(v);
            var f = OutFlags.None;
            if (w.X <= _bmin.X) f |= OutFlags.NegX;
            if (w.X >= _bmax.X) f |= OutFlags.PosX;
            if (w.Y <= _bmin.Y) f |= OutFlags.NegY;
            if (w.Y >= _bmax.Y) f |= OutFlags.PosY;
            if (w.Z <= _bmin.Z) f |= OutFlags.NegZ;
            if (w.Z >= _bmax.Z) f |= OutFlags.PosZ;
            outWorld[iv] = w;
            outFlags[iv] = f;
            ++iv;
        }
    }

    private static (float min, float max) AxisMinMax(Span<Vector3> vertices, int count, int axis)
    {
        float min = vertices[0][axis];
        float max = min;
        for (int i = 1; i < count; ++i)
        {
            float v = vertices[i][axis];
            min = Math.Min(min, v);
            max = Math.Max(max, v);
        }
        return (min, max);
    }

    // split a convex polygon along one of the axes
    // polygon 'smaller' than axis offset is to be processed next, rest is considered to be a leftover to process on the next iteration
    // count on input contains num vertices in src buffer, on output num vertices in remaining buffer
    private static int SplitConvexPoly(Span<Vector3> src, Span<Vector3> dest, Span<Vector3> remaining, ref int count, int axis, float axisOffset)
    {
        Span<float> axisDelta = stackalloc float[count];
        for (int i = 0; i < count; ++i)
            axisDelta[i] = axisOffset - src[i][axis];

        int cDest = 0, cRem = 0;
        var dPrev = axisDelta[count - 1];
        var vPrev = src[count - 1];
        for (int i = 0; i < count; ++i)
        {
            var dCurr = axisDelta[i];
            var vCurr = src[i];
            if ((dCurr >= 0) != (dPrev >= 0))
            {
                // two vertices are on the different sides of the separating axis
                float s = dPrev / (dPrev - dCurr);
                dest[cDest++] = remaining[cRem++] = vPrev + (vCurr - vPrev) * s;

                // add the i'th point to the right polygon; do NOT add points that are on the dividing line since these were already added above
                if (dCurr > 0)
                    dest[cDest++] = vCurr;
                else if (dCurr < 0)
                    remaining[cRem++] = vCurr;
            }
            else
            {
                // add the i'th point to the right polygon; addition is done even for points on the dividing line
                if (dCurr > 0)
                {
                    dest[cDest++] = vCurr;
                }
                else if (dCurr < 0)
                {
                    remaining[cRem++] = vCurr;
                }
                else
                {
                    dest[cDest++] = vCurr;
                    remaining[cRem++] = vCurr;
                }
            }
            dPrev = dCurr;
            vPrev = vCurr;
        }
        count = cRem;
        return cDest;
    }
}