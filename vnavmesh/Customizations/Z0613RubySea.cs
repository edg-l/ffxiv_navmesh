using Navmesh.GroundGraph;
using System.Collections.Generic;

namespace Navmesh.Customizations;

[CustomizationTerritory(613)]
internal class Z0613RubySea : NavmeshCustomization
{
	public override int Version => 1;

	public override void CustomizeGround(QuadGraph graph, List<uint> festivalLayers)
	{
		// the tunnel into the island containing tamamizu has some floor that is unlandable
		LinkQuads(graph, new(643.7f, 3.4f, -58.9f), new(636.6f, 3.9f, -63.3f), Navmesh.AreaId.Shortcut);
	}
}