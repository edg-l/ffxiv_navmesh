using Newtonsoft.Json.Linq;
using Xunit;
namespace Navmesh.Tests;
public class ConfigSerializationTests
{
    [Fact]
    public void UseCdtMesh_NotSerialized()
    {
        var cfg = new Config();
        var jo = JObject.FromObject(cfg);
        Assert.False(jo.ContainsKey("UseCdtMesh"), "UseCdtMesh must not appear in the serialized config payload");
    }
}
