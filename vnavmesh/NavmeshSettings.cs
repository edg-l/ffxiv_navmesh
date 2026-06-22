using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Components;
using Dalamud.Interface.Utility.Raii;
using System;

namespace Navmesh;

public class NavmeshSettings
{
    [Flags]
    public enum Filter
    {
        None = 0,
        LowHangingObstacles = 1 << 0,
        LedgeSpans = 1 << 1,
        WalkableLowHeightSpans = 1 << 2,
        Interiors = 1 << 3,
    }

    public float CellSize = 0.25f;
    public float CellHeight = 0.25f;
    public float AgentHeight = 2.0f;
    public float AgentRadius = 0.5f;
    public float AgentMaxClimb = 0.5f;
    public float AgentMaxSlopeDeg = 55f;
    public Filter Filtering = Filter.LowHangingObstacles | Filter.LedgeSpans | Filter.WalkableLowHeightSpans;

    // we assume that bounds are constant -1024 to 1024 along each axis (since that's the quantization range of position in some packets)
    // there is some code that relies on tiling being power-of-2
    // current values mean 128x128x128 L1 tiles -> 16x16x16 L2 tiles -> 2x2x2 voxels
    public int[] NumTiles = [16, 8, 8];


    public void Draw()
    {
        DrawConfigFloat(ref CellSize, 0.1f, 1.0f, 0.01f, "Rasterization: Cell Size (#cs)", """
            The xz-plane cell size to use for fields. [Limit: > 0] [Units: world]

            The voxelization cell size #cs defines the voxel size along both axes of
            the ground plane: x and z in Recast. This value is usually derived from the
            character radius `r`. A recommended starting value for #cs is either `r/2`
            or `r/3`. Smaller values of #cs will increase rasterization resolution and
            navmesh detail, but total generation time will increase exponentially.  In
            outdoor environments, `r/2` is often good enough.  For indoor scenes with
            tight spaces you might want the extra precision, so a value of `r/3` or
            smaller may give better results.

            The initial instinct is to reduce this value to something very close to zero
            to maximize the detail of the generated navmesh. This quickly becomes a case
            of diminishing returns, however. Beyond a certain point there's usually not
            much perceptable difference in the generated navmesh, but huge increases in
            generation time.  This hinders your ability to quickly iterate on level
            designs and provides little benefit.  The general recommendation here is to
            use as large a value for #cs as you can get away with.

            #cs and #ch define voxel/grid/cell size.  So their values have significant
            side effects on all parameters defined in voxel units.

            The minimum value for this parameter depends on the platform's floating point
            accuracy, with the practical minimum usually around 0.05.
            """);
        DrawConfigFloat(ref CellHeight, 0.1f, 1.0f, 0.01f, "Rasterization: Cell Height (#ch)", """
            The y-axis cell size to use for fields. [Limit: > 0] [Units: world]

            The voxelization cell height #ch is defined separately in order to allow for
            greater precision in height tests. A good starting point for #ch is half the
            #cs value. Smaller #ch values ensure that the navmesh properly connects areas
            that are only separated by a small curb or ditch.  If small holes are generated
            in your navmesh around where there are discontinuities in height (for example,
            stairs or curbs), you may want to decrease the cell height value to increase
            the vertical rasterization precision of Recast.

            #cs and #ch define voxel/grid/cell size.  So their values have significant
            side effects on all parameters defined in voxel units.

            The minimum value for this parameter depends on the platform's floating point
            accuracy, with the practical minimum usually around 0.05.
            """);
        DrawConfigFloat(ref AgentHeight, 0.1f, 5.0f, 0.1f, "Agent: Height", """
            Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3 * CellHeight] [Units: world]

            This value defines the worldspace height `h` of the agent in voxels. The value
            of #walkableHeight should be calculated as `ceil(h / ch)`.  Note this is based
            on #ch not #cs since it's a height value.

            Permits detection of overhangs in the source geometry that make the geometry
            below un-walkable. The value is usually set to the maximum agent height.
            """);
        DrawConfigFloat(ref AgentRadius, 0.0f, 5.0f, 0.1f, "Agent: Radius", """
            The distance to erode/shrink the walkable area of the heightfield away from obstructions. [Limit: >= 0] [Units: world]

            The parameter #walkableRadius defines the worldspace agent radius `r` in voxels.
            Most often, this value of #walkableRadius should be calculated as `ceil(r / cs)`.
            Note this is based on #cs since the agent radius is always parallel to the ground
            plane.

            If the #walkableRadius value is greater than zero, the edges of the navmesh will
            be pushed away from all obstacles by this amount.

            A non-zero #walkableRadius allows for much simpler runtime navmesh collision checks.
            The game only needs to check that the center point of the agent is contained within
            a navmesh polygon.  Without this erosion, runtime navigation checks need to collide
            the geometric projection of the agent's logical cylinder onto the navmesh with the
            boundary edges of the navmesh polygons.

            In general, this is the closest any part of the final mesh should get to an
            obstruction in the source geometry.  It is usually set to the maximum agent
            radius.

            If you want to have tight-fitting navmesh, or want to reuse the same navmesh for
            multiple agents with differing radii, you can use a `walkableRadius` value of zero.
            Be advised though that you will need to perform your own collisions with the navmesh
            edges, and odd edge cases issues in the mesh generation can potentially occur.  For
            these reasons, specifying a radius of zero is allowed but is not recommended.
            """);
        DrawConfigFloat(ref AgentMaxClimb, 0.1f, 5.0f, 0.1f, "Agent: Max Climb", """
            Maximum ledge height that is considered to still be traversable. [Limit: >= 0] [Units: world]

            The #walkableClimb value defines the maximum height of ledges and steps that
            the agent can walk up. Given a designer-defined `maxClimb` distance in world
            units, the value of #walkableClimb should be calculated as `ceil(maxClimb / ch)`.
            Note that this is using #ch not #cs because it's a height-based value.

            Allows the mesh to flow over low lying obstructions such as curbs and
            up/down stairways. The value is usually set to how far up/down an agent can step.
            """);
        DrawConfigFloat(ref AgentMaxSlopeDeg, 0.0f, 90.0f, 1.0f, "Agent: Max Slope", """
            The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees]

            The parameter #walkableSlopeAngle is to filter out areas of the world where
            the ground slope would be too steep for an agent to traverse. This value is
            defined as a maximum angle in degrees that the surface normal of a polgyon
            can differ from the world's up vector.  This value must be within the range
            `[0, 90]`.

            The practical upper limit for this parameter is usually around 85 degrees.
            """);
        DrawConfigFilteringCombo(ref Filtering, "Filtering", """
            Select which filtering passes to apply to voxelized geometry to remove some classes of artifacts.
            """);
        DrawConfigInt(ref NumTiles[0], 1, 32, 1, "L1 Tile count", """
            Number of tiles per axis for first-level subdivision. Has to be power-of-2. [Limit: 1 <= value <= 32]
            Affects both navmesh and nav volume.
            """);
        DrawConfigInt(ref NumTiles[1], 1, 32, 1, "L2 Tile count", """
            Number of tiles per axis for second-level subdivision. Has to be power-of-2. [Limit: 1 <= value <= 32]
            Affects only nav volume.
            """);
        DrawConfigInt(ref NumTiles[2], 1, 32, 1, "L3 Voxel count", """
            Number of leaf voxels per axis per tile. Has to be power-of-2. [Limit: 1 <= value <= 32]
            Affects only nav volume.
            """);
    }

    private void DrawConfigFloat(ref float value, float min, float max, float increment, string label, string help)
    {
        ImGui.SetNextItemWidth(300);
        ImGui.InputFloat(label, ref value);
        ImGuiComponents.HelpMarker(help);
    }

    private void DrawConfigInt(ref int value, int min, int max, int increment, string label, string help)
    {
        ImGui.SetNextItemWidth(300);
        ImGui.InputInt(label, ref value);
        ImGuiComponents.HelpMarker(help);
    }

    private void DrawConfigFilteringCombo(ref Filter value, string label, string help)
    {
        ImGui.SetNextItemWidth(300);
        using var combo = ImRaii.Combo(label, value.ToString());
        if (!combo)
        {
            ImGuiComponents.HelpMarker(help);
            return;
        }
        DrawConfigFilteringEnum(ref value, Filter.LowHangingObstacles, "Low-hanging obstacles", """
            Marks non-walkable spans as walkable if their maximum is within #walkableClimb of the span below them.

            This removes small obstacles and rasterization artifacts that the agent would be able to walk over
            such as curbs.  It also allows agents to move up terraced structures like stairs.

            Obstacle spans are marked walkable if: obstacleSpan.smax - walkableSpan.smax < walkableClimb
            """);
        DrawConfigFilteringEnum(ref value, Filter.LedgeSpans, "Ledge spans", """
            Marks spans that are ledges as not-walkable.

            A ledge is a span with one or more neighbors whose maximum is further away than #walkableClimb
            from the current span's maximum.
            This method removes the impact of the overestimation of conservative voxelization 
            so the resulting mesh will not have regions hanging in the air over ledges.

            A span is a ledge if: abs(currentSpan.smax - neighborSpan.smax) > walkableClimb
            """);
        DrawConfigFilteringEnum(ref value, Filter.WalkableLowHeightSpans, "Walkable low-height spans", """
            Marks walkable spans as not walkable if the clearance above the span is less than the specified #walkableHeight.

            For this filter, the clearance above the span is the distance from the span's 
            maximum to the minimum of the next higher span in the same column.
            If there is no higher span in the column, the clearance is computed as the
            distance from the top of the span to the maximum heightfield height.
            """);
        DrawConfigFilteringEnum(ref value, Filter.Interiors, "Interiors", """
            Marks spans inside manifold geometry (or below non-manifold) as non-walkable.
            """);
    }

    private void DrawConfigFilteringEnum(ref Filter value, Filter mask, string label, string help)
    {
        bool set = value.HasFlag(mask);
        if (ImGui.Checkbox(label, ref set))
            value ^= mask;
        ImGuiComponents.HelpMarker(help);
    }
}