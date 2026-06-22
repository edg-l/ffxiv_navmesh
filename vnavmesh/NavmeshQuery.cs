using Navmesh.GroundGraph;
using Navmesh.Movement;
using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading;

namespace Navmesh;

public class NavmeshQuery
{
	public VoxelPathfind? VolumeQuery;
	private readonly QuadGraph? _ground;

	public NavmeshQuery(Navmesh navmesh)
	{
		if (navmesh.Volume != null)
			VolumeQuery = new(navmesh.Volume);
		_ground = navmesh.Ground;
	}

	public List<Waypoint> PathfindMesh(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, float range, CancellationToken cancel)
	{
		if (_ground == null)
		{
			Service.Log.Error($"Failed to find a ground path from {from} to {to}: ground graph is missing");
			return [];
		}

		var groundTimer = Timer.Create();
		var path = _ground.Pathfind(from, to, useRaycast, useStringPulling, range, cancel);
		Service.Log.Debug($"[pathfind] ground {from} -> {to}: {path.Count} waypoints in {groundTimer.Value().TotalSeconds:f3}s");
		if (path.Count == 0)
		{
			Service.Log.Error($"Failed to find a ground path from {from} to {to}: quad graph returned no path");
			return [];
		}
		return path.Select(p => new Waypoint(p, GetAreaIdForPos(p))).Append(new Waypoint(to)).ToList();
	}

	private Navmesh.AreaId GetAreaIdForPos(Vector3 p)
	{
		if (_ground == null)
			return Navmesh.AreaId.Default;
		var q = _ground.NearestQuad(p, float.MaxValue, true);
		return q >= 0 ? _ground.Quads[q].Area : Navmesh.AreaId.Default;
	}

	public List<Waypoint> PathfindVolume(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, CancellationToken cancel)
	{
		if (VolumeQuery == null)
		{
			Service.Log.Error($"Nav volume was not built");
			return [];
		}

		var startVoxel = FindNearestVolumeVoxel(from);
		var endVoxel = FindNearestVolumeVoxel(to);
		Service.Log.Debug($"[pathfind] voxel {startVoxel:X} -> {endVoxel:X}");
		if (startVoxel == VoxelMap.InvalidVoxel || endVoxel == VoxelMap.InvalidVoxel)
		{
			Service.Log.Error($"Failed to find a path from {from} ({startVoxel:X}) to {to} ({endVoxel:X}): failed to find empty voxel");
			return [];
		}

		var timer = Timer.Create();
		var voxelPath = VolumeQuery.FindPath(startVoxel, endVoxel, from, to, useRaycast, false, cancel);
		if (voxelPath.Count == 0)
		{
			Service.Log.Error($"Failed to find a path from {from} ({startVoxel:X}) to {to} ({endVoxel:X}): failed to find path on volume");
			return [];
		}
		Service.Log.Debug($"Pathfind took {timer.Value().TotalSeconds:f3}s: {string.Join(", ", voxelPath.Select(r => $"{r.p} {r.voxel:X}"))}");

		if (useStringPulling)
		{
			var straighten = new VoxelStraighten(VolumeQuery.Volume);
			var simplified = straighten.Simplify(voxelPath, to);
			return simplified.Select(p => new Waypoint(p)).ToList();
		}
		else
		{
			var res = voxelPath.Select(r => new Waypoint(r.p)).ToList();
			res.Add(new(to));
			return res;
		}
	}

	public long FindNearestMeshPoly(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5, bool allowUnreachable = true)
	{
		if (_ground == null)
			return 0;
		var q = _ground.NearestQuad(p, float.MaxValue, allowUnreachable);
		return q >= 0 ? (long)q : 0;
	}

	public List<long> FindIntersectingMeshPolys(Vector3 p, Vector3 halfExtent, bool allowUnreachable = true)
	{
		var result = new List<long>();
		if (_ground == null)
			return result;
		for (int i = 0; i < _ground.Quads.Count; ++i)
		{
			if (!allowUnreachable && i < _ground.Flags.Length && (_ground.Flags[i] & QuadGraph.FLAG_UNREACHABLE) != 0)
				continue;
			var q = _ground.Quads[i];
			if (q.MaxX < p.X - halfExtent.X || q.MinX > p.X + halfExtent.X)
				continue;
			if (q.MaxZ < p.Z - halfExtent.Z || q.MinZ > p.Z + halfExtent.Z)
				continue;
			if (q.MinY < p.Y - halfExtent.Y || q.MinY > p.Y + halfExtent.Y)
				continue;
			result.Add(i);
		}
		return result;
	}

	public Vector3? FindNearestPointOnMeshPoly(Vector3 p, long poly)
	{
		if (_ground != null && poly >= 0 && poly < _ground.Quads.Count)
		{
			var q = _ground.Quads[(int)poly];
			var x = Math.Clamp(p.X, q.MinX, q.MaxX);
			var z = Math.Clamp(p.Z, q.MinZ, q.MaxZ);
			return new Vector3(x, q.MinY, z);
		}
		return null;
	}

	public Vector3? FindNearestPointOnMesh(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5, bool allowUnreachable = true) => FindNearestPointOnMeshPoly(p, FindNearestMeshPoly(p, halfExtentXZ, halfExtentY, allowUnreachable));

	public Vector3? FindPointOnFloor(Vector3 p, float halfExtentXZ = 5, bool allowUnreachable = true)
	{
		if (_ground == null)
			return null;
		Vector3? best = null;
		for (int i = 0; i < _ground.Quads.Count; ++i)
		{
			if (!allowUnreachable && i < _ground.Flags.Length && (_ground.Flags[i] & QuadGraph.FLAG_UNREACHABLE) != 0)
				continue;
			var q = _ground.Quads[i];
			if (p.X < q.MinX - halfExtentXZ || p.X > q.MaxX + halfExtentXZ)
				continue;
			if (p.Z < q.MinZ - halfExtentXZ || p.Z > q.MaxZ + halfExtentXZ)
				continue;
			if (q.MinY > p.Y)
				continue;
			if (best == null || q.MinY > best.Value.Y)
				best = new Vector3(Math.Clamp(p.X, q.MinX, q.MaxX), q.MinY, Math.Clamp(p.Z, q.MinZ, q.MaxZ));
		}
		return best;
	}

	public ulong FindNearestVolumeVoxel(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5) => VolumeQuery != null ? VoxelSearch.FindNearestEmptyVoxel(VolumeQuery.Volume, p, new(halfExtentXZ, halfExtentY, halfExtentXZ)) : VoxelMap.InvalidVoxel;

	public HashSet<long> FindReachableMeshPolys(params long[] starting)
	{
		if (_ground == null)
			return [];
		var seeds = starting.Select(s => (int)s).Where(s => s >= 0 && s < _ground.Quads.Count);
		return _ground.FloodReachable(seeds).Select(i => (long)i).ToHashSet();
	}
}