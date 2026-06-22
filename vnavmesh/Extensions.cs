using System;
using System.Numerics;

namespace Navmesh;

public static class Extensions
{
    public static Vector3 Floor(this Vector3 v) => new(MathF.Floor(v.X), MathF.Floor(v.Y), MathF.Floor(v.Z));
    public static Vector3 Ceiling(this Vector3 v) => new(MathF.Ceiling(v.X), MathF.Ceiling(v.Y), MathF.Ceiling(v.Z));
}