using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace Navmesh.GroundGraph.Extraction;

// A floor span in a column of the fine walkable grid. Floor Y is the world-space
// Y of the solid span's top surface (fixes GAP2: derived from solid-span TOP, not
// voxel center). Area is the Recast walkable area id (0 = unwalkable, 1 =
// walkable). TopClearance is the vertical space above this floor up to the next
// obstacle span above it (in world units); set to float.MaxValue when nothing
// blocks above.
public readonly record struct FloorSpan(float FloorY, byte Area, float TopClearance);

// Per-tile fine walkable grid derived directly from the rasterizer's solid spans
// BEFORE any downsampling into the 2y VoxelMap octree. Columns are indexed by
// (cellX, cellZ) in the rasterizer's fine grid. Each column holds zero or more
// floor spans sorted ascending by FloorY.
//
// Grid origin: BoundsMin of the tile (the rasterizer's bounding box).
// Grid dimensions: Width x Height cells in XZ at CellSize resolution.
// A border strip of width BorderSize cells on each side comes from overlapping
// neighbour tiles and must be stripped when stitching tiles together.
public class CompactHeightfield
{
    // World-space bottom-left corner of the grid (same as rasterizer BoundsMin).
    public Vector3 BoundsMin { get; }
    // Cell size in XZ (= rasterizer CellSize).
    public float CellSize { get; }
    // Cell height in Y (= rasterizer CellHeight).
    public float CellHeight { get; }
    // Number of columns in X.
    public int Width { get; }
    // Number of columns in Z.
    public int Height { get; }
    // Border strip width (cells) that must be dropped when stitching.
    public int BorderSize { get; }
    // Max climb (in voxels) used for layer partitioning and link detection.
    public int WalkableClimbVoxels { get; }
    // Max climb in world units. This is the authoritative threshold for BFS propagation;
    // using world units avoids floor-rounding errors from voxel-count conversion.
    public float WalkableClimbWorld { get; }

    // F1: per-column span lists; index = z * Width + x. Nullable for lazy alloc.
    private readonly List<FloorSpan>?[] _columns;
    // Shared empty list returned by GetSpans for uninitialized (empty) columns.
    private static readonly IReadOnlyList<FloorSpan> _emptySpans = Array.Empty<FloorSpan>();

    public CompactHeightfield(Vector3 boundsMin, float cellSize, float cellHeight,
        int width, int height, int borderSize, int walkableClimbVoxels, float walkableClimbWorld = -1f)
    {
        BoundsMin = boundsMin;
        CellSize = cellSize;
        CellHeight = cellHeight;
        Width = width;
        Height = height;
        BorderSize = borderSize;
        WalkableClimbVoxels = walkableClimbVoxels;
        WalkableClimbWorld = walkableClimbWorld > 0f ? walkableClimbWorld : walkableClimbVoxels * cellHeight;
        // F1: allocate the outer array only; inner lists are lazily created.
        _columns = new List<FloorSpan>?[width * height];
    }

    // Add a floor span to the column at (x, z). Spans should be added in top-Y
    // order to compute clearance; call FinalizeColumn after all spans for a column
    // are added. Alternatively, call AddSpanSorted to insert in sorted order.
    public void AddSpanSorted(int x, int z, float floorY, byte area)
    {
        int ci = z * Width + x;
        // F1: lazily allocate the column list on first use.
        _columns[ci] ??= new List<FloorSpan>();
        var col = _columns[ci]!;
        // Insert sorted by floorY ascending, with clearance computed after.
        int ins = col.Count;
        for (int i = 0; i < col.Count; i++)
        {
            if (col[i].FloorY > floorY)
            {
                ins = i;
                break;
            }
        }
        col.Insert(ins, new FloorSpan(floorY, area, float.MaxValue));
    }

    // Recompute TopClearance for all spans in all columns. Must be called once
    // after all spans have been inserted. For each span i, clearance =
    // col[i+1].FloorY - col[i].FloorY if there is a span above, else float.MaxValue.
    public void FinalizeAllClearances()
    {
        foreach (var col in _columns)
        {
            if (col == null) continue; // F1: skip empty columns
            for (int i = 0; i < col.Count; i++)
            {
                float clearance = i + 1 < col.Count
                    ? col[i + 1].FloorY - col[i].FloorY
                    : float.MaxValue;
                col[i] = col[i] with { TopClearance = clearance };
            }
        }
    }

    // Return spans for cell (x, z). F1: returns empty for null (uninitialized) columns.
    public IReadOnlyList<FloorSpan> GetSpans(int x, int z) => _columns[z * Width + x] ?? _emptySpans;

    // Internal mutable access for seam healing (same assembly only). F1: allocates lazily.
    internal List<FloorSpan> GetSpansMutable(int x, int z)
    {
        int ci = z * Width + x;
        _columns[ci] ??= new List<FloorSpan>();
        return _columns[ci]!;
    }

    // Convert world X coordinate to cell X index (clamped to [0, Width-1]).
    public int WorldToCell_X(float worldX)
        => Math.Clamp((int)((worldX - BoundsMin.X) / CellSize), 0, Width - 1);

    // Convert world Z coordinate to cell Z index (clamped to [0, Height-1]).
    public int WorldToCell_Z(float worldZ)
        => Math.Clamp((int)((worldZ - BoundsMin.Z) / CellSize), 0, Height - 1);

    // World position of the center of cell (x, z) at the given floorY.
    public Vector3 CellCenter(int x, int z, float floorY)
        => new(BoundsMin.X + (x + 0.5f) * CellSize, floorY, BoundsMin.Z + (z + 0.5f) * CellSize);

    // World X of the left edge (min-X) of cell x.
    public float CellMinX(int x) => BoundsMin.X + x * CellSize;
    // World X of the right edge (max-X) of cell x.
    public float CellMaxX(int x) => BoundsMin.X + (x + 1) * CellSize;
    // World Z of the bottom edge (min-Z) of cell z.
    public float CellMinZ(int z) => BoundsMin.Z + z * CellSize;
    // World Z of the top edge (max-Z) of cell z.
    public float CellMaxZ(int z) => BoundsMin.Z + (z + 1) * CellSize;

    // Build a CompactHeightfield by scanning a VoxelMap for walkable surfaces.
    // Used only in tests; the production pipeline uses NavmeshRasterizer.PopulateChf().
    // Scans leaf voxels: an empty voxel above a solid voxel → walkable floor at
    // solid-voxel top Y. Uses the VoxelMap's finest leaf cell size for the CHF grid.
    internal static CompactHeightfield FromVoxelMap(VoxelMap volume, Vector3 boundsMin, Vector3 boundsMax, float agentMaxClimb)
    {
        var leafLevel = volume.Levels[^1];
        var leafCellSize = leafLevel.CellSize;
        var origin = volume.RootTile.BoundsMin;

        int totalCellsX = 1, totalCellsZ = 1;
        foreach (var lvl in volume.Levels)
        {
            totalCellsX *= lvl.NumCellsX;
            totalCellsZ *= lvl.NumCellsZ;
        }

        int climbVoxels = (int)MathF.Floor(agentMaxClimb / leafCellSize.Y);
        var chf = new CompactHeightfield(
            origin, leafCellSize.X, leafCellSize.Y,
            totalCellsX, totalCellsZ, 0, climbVoxels, agentMaxClimb);

        // F2: hoist totalCellsY computation out of the per-(x,z) loop.
        int totalCellsY = 1;
        foreach (var lvl in volume.Levels)
            totalCellsY *= lvl.NumCellsY;

        // Scan all leaf voxels: empty above solid → walkable floor.
        for (int z = 0; z < totalCellsZ; z++)
        {
            for (int x = 0; x < totalCellsX; x++)
            {
                for (int y = totalCellsY - 1; y > 0; y--)
                {
                    var probePos = origin + new Vector3((x + 0.5f) * leafCellSize.X, (y + 0.5f) * leafCellSize.Y, (z + 0.5f) * leafCellSize.Z);
                    var (_, aboveEmpty) = volume.FindLeafVoxel(probePos);
                    if (!aboveEmpty) continue;

                    var belowPos = probePos - new Vector3(0, leafCellSize.Y, 0);
                    var (_, belowEmpty) = volume.FindLeafVoxel(belowPos);
                    if (belowEmpty) continue;

                    // Walkable floor: top of the solid voxel below.
                    float floorY = belowPos.Y + leafCellSize.Y * 0.5f; // center of solid voxel + half height = top
                    chf.AddSpanSorted(x, z, floorY, 1); // area=1 = walkable
                }
            }
        }

        chf.FinalizeAllClearances();
        return chf;
    }
}
