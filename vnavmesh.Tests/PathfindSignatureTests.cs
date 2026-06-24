using Navmesh.GroundGraph;
using System.Collections.Generic;
using System.Numerics;
using System.Reflection;
using System.Threading;
using Xunit;

namespace Navmesh.Tests;

// Phase 2 checkpoint: reflection test asserting QuadGraph.Pathfind's signature
// is byte-identical to the upstream IPC contract. The 6-arg Pathfind is the
// seam NavmeshQuery.PathfindMesh calls; changing it would break consumers.
public class PathfindSignatureTests
{
    [Fact]
    public void QuadGraph_Pathfind_SignatureUnchanged()
    {
        var method = typeof(QuadGraph).GetMethod(
            nameof(QuadGraph.Pathfind),
            BindingFlags.Public | BindingFlags.Instance);
        Assert.NotNull(method);
        var parameters = method!.GetParameters();
        Assert.Equal(6, parameters.Length);
        Assert.Equal(typeof(Vector3), parameters[0].ParameterType);
        Assert.Equal(typeof(Vector3), parameters[1].ParameterType);
        Assert.Equal(typeof(bool), parameters[2].ParameterType);
        Assert.Equal(typeof(bool), parameters[3].ParameterType);
        Assert.Equal(typeof(float), parameters[4].ParameterType);
        Assert.Equal(typeof(System.Threading.CancellationToken), parameters[5].ParameterType);
        Assert.Equal(typeof(List<Vector3>), method.ReturnType);
    }
}