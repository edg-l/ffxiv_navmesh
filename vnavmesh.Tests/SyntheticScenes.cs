using Navmesh.NavVolume;
using System.Collections.Generic;
using System.Numerics;

namespace Navmesh.Tests;

public record struct Scene(VoxelMap Volume, List<Vector3> WalkablePoints, Vector3 BoundsMin, Vector3 BoundsMax, string Name);

public static class SyntheticScenes
{
    // Tile config: 3 levels, each with 4 cells per axis.
    // With bounds [-20,-10,-20] to [20,10,20]:
    //   L0 cell: 10 x 5 x 10
    //   L1 cell: 2.5 x 1.25 x 2.5
    //   L2 (leaf): 0.625 x 0.3125 x 0.625
    private static readonly int[] TilesPerLevel = [4, 4, 4];
    private static readonly Vector3 BMin = new(-20, -10, -20);
    private static readonly Vector3 BMax = new(20, 10, 20);

    // Floor thickness sufficient to ensure solid voxels at the leaf level.
    private const float FloorThickness = 1.0f;
    // Surface Y used for walkable points (just above floor top).
    private const float WalkableYOffset = 0.5f;

    private static VoxelMap CreateMap() => new(BMin, BMax, TilesPerLevel);

    public static Scene FlatPlane()
    {
        var vol = CreateMap();
        // Floor: full XZ extent, 1 unit thick at Y=0
        var floorMin = new Vector3(BMin.X, 0, BMin.Z);
        var floorMax = new Vector3(BMax.X, FloorThickness, BMax.Z);
        vol.FillBox(floorMin, floorMax);

        float surfY = FloorThickness + WalkableYOffset;
        var walkable = new List<Vector3>
        {
            new(-15, surfY, -15),
            new(-15, surfY,   0),
            new(-15, surfY,  15),
            new(  0, surfY, -15),
            new(  0, surfY,   0),
            new(  0, surfY,  15),
            new( 15, surfY, -15),
            new( 15, surfY,   0),
            new( 15, surfY,  15),
        };
        return new Scene(vol, walkable, BMin, BMax, nameof(FlatPlane));
    }

    public static Scene PlaneWithPillar()
    {
        var vol = CreateMap();
        var floorMin = new Vector3(BMin.X, 0, BMin.Z);
        var floorMax = new Vector3(BMax.X, FloorThickness, BMax.Z);
        vol.FillBox(floorMin, floorMax);

        // Pillar: 4x4 XZ footprint at center, height 4
        var pillarMin = new Vector3(-2, FloorThickness, -2);
        var pillarMax = new Vector3(2, FloorThickness + 4, 2);
        vol.FillBox(pillarMin, pillarMax);

        float surfY = FloorThickness + WalkableYOffset;
        var walkable = new List<Vector3>
        {
            new(-15, surfY, -15),
            new(-15, surfY,  15),
            new( 15, surfY, -15),
            new( 15, surfY,  15),
            new( -8, surfY,   0),
            new(  8, surfY,   0),
            new(  0, surfY, -8),
            new(  0, surfY,  8),
        };
        return new Scene(vol, walkable, BMin, BMax, nameof(PlaneWithPillar));
    }

    public static Scene Overpass()
    {
        var vol = CreateMap();
        // Lower road: strip along Z at Y=0
        vol.FillBox(new Vector3(-6, 0, BMin.Z), new Vector3(6, FloorThickness, BMax.Z));
        // Upper road: strip along X at Y=6, crossing the lower road (> climb gap)
        vol.FillBox(new Vector3(BMin.X, 6, -6), new Vector3(BMax.X, 6 + FloorThickness, 6));

        float lowerSurf = FloorThickness + WalkableYOffset;
        float upperSurf = 6 + FloorThickness + WalkableYOffset;
        var walkable = new List<Vector3>
        {
            new(0, lowerSurf, -15),
            new(0, lowerSurf, 15),
            new(-15, upperSurf, 0),
            new(15, upperSurf, 0),
        };
        return new Scene(vol, walkable, BMin, BMax, nameof(Overpass));
    }

    public static Scene BridgeOnramp()
    {
        var vol = CreateMap();
        // Main deck at Y=0..FloorThickness, surface at Y=FloorThickness
        vol.FillBox(new Vector3(BMin.X, 0, -8), new Vector3(BMax.X, FloorThickness, 8));
        // Ramp: 2 steps, each 0.3125 high (one leaf voxel). Quantized diffs = 0.3125 < AgentMaxClimb=0.5.
        // Step1 top at Y = FloorThickness + 0.3125 = 1.3125
        vol.FillBox(new Vector3(8, 0, -4), new Vector3(10, FloorThickness + 0.3125f, 4));
        // Step2 top at Y = FloorThickness + 0.625 = 1.625
        vol.FillBox(new Vector3(10, 0, -4), new Vector3(12, FloorThickness + 0.625f, 4));
        // Bridge deck top at same level as step2
        vol.FillBox(new Vector3(12, 0, -4), new Vector3(BMax.X, FloorThickness + 0.625f, 4));

        float mainSurf = FloorThickness + WalkableYOffset;
        float step1Surf = FloorThickness + 0.3125f + WalkableYOffset;
        float bridgeSurf = FloorThickness + 0.625f + WalkableYOffset;
        var walkable = new List<Vector3>
        {
            new(-15, mainSurf, 0),
            new( 5, mainSurf, 0),
            new( 9, step1Surf, 0),
            new(16, bridgeSurf, 0),
        };
        return new Scene(vol, walkable, BMin, BMax, nameof(BridgeOnramp));
    }

    public static Scene NarrowCorridor(float width)
    {
        var vol = CreateMap();
        // Floor slab
        vol.FillBox(new Vector3(BMin.X, 0, -width / 2f), new Vector3(BMax.X, FloorThickness, width / 2f));
        // Left wall
        vol.FillBox(new Vector3(BMin.X, FloorThickness, -10), new Vector3(BMax.X, FloorThickness + 4, -width / 2f));
        // Right wall
        vol.FillBox(new Vector3(BMin.X, FloorThickness, width / 2f), new Vector3(BMax.X, FloorThickness + 4, 10));

        float surfY = FloorThickness + WalkableYOffset;
        var walkable = new List<Vector3>
        {
            new(-15, surfY, 0),
            new(  0, surfY, 0),
            new( 15, surfY, 0),
        };
        return new Scene(vol, walkable, BMin, BMax, nameof(NarrowCorridor));
    }

    public static Scene Staircase(float stepHeight)
    {
        var vol = CreateMap();
        // Build a staircase along +X direction, each step is 2 units wide and stepHeight tall
        float stepWidth = 2.0f;
        int numSteps = 8;
        for (int i = 0; i < numSteps; i++)
        {
            float xStart = BMin.X + i * stepWidth;
            float yTop = FloorThickness + i * stepHeight;
            vol.FillBox(
                new Vector3(xStart, 0, -6),
                new Vector3(xStart + stepWidth, yTop, 6));
        }

        float lastStepTop = FloorThickness + (numSteps - 1) * stepHeight;
        var walkable = new List<Vector3>();
        for (int i = 0; i < numSteps; i++)
        {
            float xCenter = BMin.X + i * stepWidth + stepWidth * 0.5f;
            float surfY = FloorThickness + i * stepHeight + WalkableYOffset;
            walkable.Add(new Vector3(xCenter, surfY, 0));
        }
        return new Scene(vol, walkable, BMin, BMax, nameof(Staircase));
    }
}
