using Navmesh.GroundGraph;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;

namespace Navmesh.Customizations;

[CustomizationTerritory(1252)]
internal class Z1252OccultCrescentSouthHorn : NavmeshCustomization
{
    public override int Version => 5;

    public override void CustomizeScene(SceneExtractor scene)
    {
        if (scene.Meshes.TryGetValue("bg/ex5/03_ocn_o6/btl/o6b1/collision/o6b1_a5_stc02.pcb", out var mesh))
        {
            var verts = CollectionsMarshal.AsSpan(mesh.Parts[221].Vertices);
            verts[8].X += 1;
            verts[16].X += 1;
        }
    }

    public override void CustomizeGround(QuadGraph graph, List<uint> festivalLayers)
    {
        graph.AddOffMesh(new Vector3(295.64f, 101.79f, 322.61f), new Vector3(293.91f, 82.02f, 355.45f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(307.39f, 102.88f, 311.06f), new Vector3(339.73f, 69.75f, 321.51f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(309.04f, 102.88f, 314.50f), new Vector3(321.17f, 76.74f, 335.64f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(331.43f, 96.00f, 111.11f), new Vector3(342.42f, 88.90f, 91.92f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(-337.27f, 47.34f, -419.95f), new Vector3(-333.29f, 7.06f, -451.97f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(-175.51f, 6.5f, -607.24f), new Vector3(-183.04f, 3.85f, -607.21f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(-416f, 3.8f, -562.77f), new Vector3(-439.071f, -0.3f, -556.1f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(-500.08f, 3.5f, -552.53f), new Vector3(-509.95f, -0.3f, -552.91f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(5.23f, 106.65f, -390.92f), new Vector3(16.14f, 25.44f, -437.46f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(-801.5f, 53.95f, 313.05f), new Vector3(-799.7f, 46.73f, 299.39f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(778.16f, 110f, 533.72f), new Vector3(770.54f, 80.3f, 534.08f), Navmesh.AreaId.Shortcut);
        graph.AddOffMesh(new Vector3(831.25f, 98f, 722.47f), new Vector3(822.42f, 80f, 722.53f), Navmesh.AreaId.Shortcut);
    }
}