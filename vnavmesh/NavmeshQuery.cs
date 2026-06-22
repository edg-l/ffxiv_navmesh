using DotRecast.Core.Numerics;
using DotRecast.Detour;
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
	private class IntersectQuery : IDtPolyQuery
	{
		public readonly List<long> Result = [];
		public void Process(DtMeshTile tile, DtPoly poly, long refs) => Result.Add(refs);
	}

	public class GoalRadiusHeuristic(float tolerance) : IDtQueryHeuristic
	{
		float IDtQueryHeuristic.GetCost(RcVec3f neighbourPos, RcVec3f endPos)
		{
			var dist = RcVec3f.Distance(neighbourPos, endPos) * DtDefaultQueryHeuristic.H_SCALE;
			return dist < tolerance ? -1 : dist;
		}
	}

	public class TeleportAwareFilter : IDtQueryFilter
	{
		private readonly DtQueryDefaultFilter _f = new();

		public float GetCost(RcVec3f pa, RcVec3f pb, long prevRef, DtMeshTile prevTile, DtPoly prevPoly, long curRef, DtMeshTile curTile, DtPoly curPoly, long nextRef, DtMeshTile nextTile, DtPoly nextPoly)
		{
			var cst = _f.GetCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly);

			var costMulti = 10f;

			var curArea = (Navmesh.AreaId)curPoly.GetArea();
			var nextArea = (Navmesh.AreaId)(nextPoly?.GetArea() ?? 0);

			if ((curArea ^ nextArea) == Navmesh.AreaId.Endpoint)
				costMulti = curArea switch
				{
					Navmesh.AreaId.Warp => 1,
					Navmesh.AreaId.ClientPath => 3,
					Navmesh.AreaId.Shortcut => 8,
					_ => costMulti
				};

			return cst * costMulti;
		}


		public virtual bool PassFilter(long refs, DtMeshTile tile, DtPoly poly) => true;
	}

	public class FloodFillAwareFilter : TeleportAwareFilter
	{
		public override bool PassFilter(long refs, DtMeshTile tile, DtPoly poly)
		{
			return (poly.flags & Navmesh.FLAG_UNREACHABLE) == 0;
		}
	}

	public DtNavMeshQuery MeshQuery;
	public VoxelPathfind? VolumeQuery;
	private readonly QuadGraph? _ground;
	private readonly IDtQueryFilter _filter = new DtQueryDefaultFilter();
	private readonly IDtQueryFilter _pathFilter = new TeleportAwareFilter();
	private readonly IDtQueryFilter _reachableFilter = new FloodFillAwareFilter();

	public List<long> LastPath => _lastPath;
	private List<long> _lastPath = [];

	public NavmeshQuery(Navmesh navmesh)
	{
		MeshQuery = new(navmesh.Mesh/*, s => Service.Log.Debug(s)*/);
		if (navmesh.Volume != null)
			VolumeQuery = new(navmesh.Volume);
		_ground = navmesh.Ground;
	}

	public List<Waypoint> PathfindMesh(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, float range, CancellationToken cancel)
	{
		if (_ground != null)
		{
			var groundTimer = Timer.Create();
			var path = _ground.Pathfind(from, to, useRaycast, useStringPulling, range, cancel);
			Service.Log.Debug($"[pathfind] ground {from} -> {to}: {path.Count} waypoints in {groundTimer.Value().TotalSeconds:f3}s");
			if (path.Count == 0)
			{
				Service.Log.Error($"Failed to find a ground path from {from} to {to}: quad graph returned no path");
				return [];
			}
			_lastPath = [];
			return path.Select(p => new Waypoint(p, GetAreaIdForPos(p))).Append(new Waypoint(to)).ToList();
		}

		var startRef = FindNearestMeshPoly(from);
		var endRef = FindNearestMeshPoly(to);
		Service.Log.Debug($"[pathfind] poly {startRef:X} -> {endRef:X}");
		if (startRef == 0 || endRef == 0)
		{
			Service.Log.Error($"Failed to find a path from {from} ({startRef:X}) to {to} ({endRef:X}): failed to find polygon on a mesh");
			return [];
		}

		var timer = Timer.Create();
		_lastPath.Clear();
		var opt = new DtFindPathOption(range > 0 ? new GoalRadiusHeuristic(range) : DtDefaultQueryHeuristic.Default, useRaycast ? DtFindPathOptions.DT_FINDPATH_ANY_ANGLE : 0, useRaycast ? float.MaxValue : 0);
		MeshQuery.FindPath(startRef, endRef, from.SystemToRecast(), to.SystemToRecast(), _pathFilter, ref _lastPath, opt);
		if (_lastPath.Count == 0)
		{
			Service.Log.Error($"Failed to find a path from {from} ({startRef:X}) to {to} ({endRef:X}): failed to find path on mesh");
			return [];
		}
		Service.Log.Debug($"Pathfind took {timer.Value().TotalSeconds:f3}s: {string.Join(", ", _lastPath.Select(r => r.ToString("X")))}");

		var endPos = to.SystemToRecast();

		cancel.ThrowIfCancellationRequested();

		if (useStringPulling)
		{
			var straightPath = new List<DtStraightPath>();
			var success = MeshQuery.FindStraightPath(from.SystemToRecast(), endPos, _lastPath, ref straightPath, 1024, 0);
			if (success.Failed())
				Service.Log.Error($"Failed to find a path from {from} ({startRef:X}) to {to} ({endRef:X}): failed to find straight path ({success.Value:X})");
			var res = straightPath.Select(p => new Waypoint(p.pos.RecastToSystem(), GetAreaId(p.refs))).ToList();
			res.Add(new(endPos.RecastToSystem()));
			return res;
		}
		else
		{
			var res = _lastPath.Select(r => new Waypoint(MeshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem(), GetAreaId(r))).ToList();
			res.Add(new(endPos.RecastToSystem()));
			return res;
		}
	}

	private Navmesh.AreaId GetAreaIdForPos(Vector3 p)
	{
		if (_ground == null)
			return Navmesh.AreaId.Default;
		var q = _ground.NearestQuad(p, float.MaxValue, true);
		return q >= 0 ? _ground.Quads[q].Area : Navmesh.AreaId.Default;
	}

	private Navmesh.AreaId GetAreaId(long refs)
	{
		if (_ground != null && refs >= 0 && refs < _ground.Quads.Count)
			return _ground.Quads[(int)refs].Area;
		MeshQuery.GetAttachedNavMesh().GetPolyArea(refs, out var area);
		return (Navmesh.AreaId)area;
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
		if (_ground != null)
		{
			var q = _ground.NearestQuad(p, float.MaxValue, allowUnreachable);
			return q >= 0 ? (long)q : 0;
		}
		MeshQuery.FindNearestPoly(p.SystemToRecast(), new(halfExtentXZ, halfExtentY, halfExtentXZ), allowUnreachable ? _filter : _reachableFilter, out var nearestRef, out _, out _);
		return nearestRef;
	}

	public List<long> FindIntersectingMeshPolys(Vector3 p, Vector3 halfExtent, bool allowUnreachable = true)
	{
		if (_ground != null)
		{
			var result = new List<long>();
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
		IntersectQuery query = new();
		MeshQuery.QueryPolygons(p.SystemToRecast(), halfExtent.SystemToRecast(), allowUnreachable ? _filter : _reachableFilter, query);
		return query.Result;
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
		return MeshQuery.ClosestPointOnPoly(poly, p.SystemToRecast(), out var closest, out _).Succeeded() ? closest.RecastToSystem() : null;
	}

	public Vector3? FindNearestPointOnMesh(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5, bool allowUnreachable = true) => FindNearestPointOnMeshPoly(p, FindNearestMeshPoly(p, halfExtentXZ, halfExtentY, allowUnreachable));

	public Vector3? FindPointOnFloor(Vector3 p, float halfExtentXZ = 5, bool allowUnreachable = true)
	{
		if (_ground != null)
		{
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
		IEnumerable<long> polys = FindIntersectingMeshPolys(p, new(halfExtentXZ, 2048, halfExtentXZ), allowUnreachable);
		return polys.Select(poly => FindNearestPointOnMeshPoly(p, poly)).Where(pt => pt != null && pt.Value.Y <= p.Y).MaxBy(pt => pt!.Value.Y);
	}

	public ulong FindNearestVolumeVoxel(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5) => VolumeQuery != null ? VoxelSearch.FindNearestEmptyVoxel(VolumeQuery.Volume, p, new(halfExtentXZ, halfExtentY, halfExtentXZ)) : VoxelMap.InvalidVoxel;

	public HashSet<long> FindReachableMeshPolys(params long[] starting)
	{
		if (_ground != null)
		{
			var seeds = starting.Select(s => (int)s).Where(s => s >= 0 && s < _ground.Quads.Count);
			return _ground.FloodReachable(seeds).Select(i => (long)i).ToHashSet();
		}
		HashSet<long> result = [];

		List<long> queue = [.. starting];
		queue.RemoveAll(s => s == 0);

		while (queue.Count > 0)
		{
			var next = queue[^1];
			queue.RemoveAt(queue.Count - 1);

			if (!result.Add(next))
				continue;

			MeshQuery.GetAttachedNavMesh().GetTileAndPolyByRefUnsafe(next, out var nextTile, out var nextPoly);
			for (int i = nextTile.polyLinks[nextPoly.index]; i != DtNavMesh.DT_NULL_LINK; i = nextTile.links[i].next)
			{
				long neighbourRef = nextTile.links[i].refs;
				if (neighbourRef != 0)
					queue.Add(neighbourRef);
			}
		}

		return result;
	}
}
