using Dalamud.Game.ClientState.Conditions;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using Navmesh.GroundGraph;
using Navmesh.Movement;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;

namespace Navmesh;

// manager that loads navmesh matching current zone and performs async pathfinding queries
public sealed class NavmeshManager : IDisposable
{
	public bool UseRaycasts = true;
	public bool UseStringPulling = true;

	public string CurrentKey { get; private set; } = ""; // unique string representing currently loaded navmesh
	public Navmesh? Navmesh { get; private set; }
	public NavmeshQuery? Query { get; private set; }
	public event Action<Navmesh?, NavmeshQuery?>? OnNavmeshChanged;

	private volatile float _loadTaskProgress = -1;
	public float LoadTaskProgress => _loadTaskProgress; // negative if load task is not running, otherwise in [0, 1] range

	private CancellationTokenSource? _currentCTS; // this is signalled when mesh is unloaded, all pathfinding tasks that use it are then cancelled
	private Task _lastLoadQueryTask; // we limit the concurrency to max 1 running task (otherwise we'd need multiple Query objects, which aren't lightweight); note that each task completes on main thread!

	private int _numActivePathfinds;
	public bool PathfindInProgress => _numActivePathfinds > 0;
	public int NumQueuedPathfindRequests => _numActivePathfinds > 0 ? _numActivePathfinds - 1 : 0;

	private DirectoryInfo _cacheDir;

	public unsafe NavmeshManager(DirectoryInfo cacheDir)
	{
		_cacheDir = cacheDir;
		cacheDir.Create(); // ensure directory exists

		// prepare a task with correct task scheduler that other tasks can be chained off
		_lastLoadQueryTask = Service.Framework.Run(() => Log("Tasks kicked off"));
	}

	public void Dispose()
	{
		Log("Disposing");
		ClearState();
	}

	public void Update()
	{
		var curKey = GetCurrentKey();
		if (curKey != CurrentKey)
		{
			// navmesh needs to be reloaded
			if (!Service.Config.AutoLoadNavmesh)
			{
				if (CurrentKey.Length == 0)
					return; // nothing is loaded, and auto-load is forbidden
				curKey = ""; // just unload existing mesh
			}
			Log($"Starting transition from '{CurrentKey}' to '{curKey}'");
			CurrentKey = curKey;
			Reload(true);
			// mesh load is now in progress
		}
	}

	public bool Reload(bool allowLoadFromCache)
	{
		ClearState();
		if (CurrentKey.Length > 0)
		{
			var cts = _currentCTS = new();
			ExecuteWhenIdle(async cancel =>
			{
				_loadTaskProgress = 0;

				using var resetLoadProgress = new OnDispose(() => _loadTaskProgress = -1);

				var waitStart = DateTime.Now;

				while (InCutscene)
				{
					if ((DateTime.Now - waitStart).TotalSeconds >= 5)
					{
						waitStart = DateTime.Now;
						Log("waiting for cutscene");
					}
					await Service.Framework.DelayTicks(1, cancel);
				}

				var (cacheKey, scene) = await Service.Framework.Run(() =>
				{
					var scene = new SceneDefinition();
					scene.FillFromActiveLayout();
					var cacheKey = GetCacheKey(scene);
					return (cacheKey, scene);
				}, cancel);

				Log($"Kicking off build for '{cacheKey}' (reload={allowLoadFromCache})");
				var navmesh = await Task.Run(() => BuildNavmesh(scene, cacheKey, allowLoadFromCache, cancel), cancel);
				Log($"Mesh loaded: '{cacheKey}'");
				Navmesh = navmesh;
				Query = new(Navmesh);

			var ff = await FloodFill.GetAsync();
			if (ff.TryLookup(scene.TerritoryID, out var points))
			{
				if (Navmesh?.Ground != null)
					PruneGround(Navmesh.Ground, points);
			}

				OnNavmeshChanged?.Invoke(Navmesh, Query);
			}, cts.Token);
		}
		return true;
	}

	// Cancel an in-progress load/build. Signals the current build's token; the
	// build loop (tile rasterization + CDT phase) checks it and throws
	// OperationCanceledException, which the task chain swallows and resets
	// progress. Safe to call when nothing is building (no-op).
	public void CancelBuild() => _currentCTS?.Cancel();

	internal void ReplaceMesh(Navmesh mesh)
	{
		Navmesh = mesh;
		Query = new(Navmesh);
		Log($"Mesh replaced");
		OnNavmeshChanged?.Invoke(Navmesh, Query);
	}

	// [dev] capture the raw collision geometry for the CURRENT territory and write
	// it to a .scene file for offline replay of the whole build pipeline. Extracts
	// + customizes the scene exactly as BuildNavmesh does (minus the actual build),
	// so the captured scene is already customized. Must run on the framework thread
	// (the game extraction is unsafe). Returns the written path, or null on failure.
	public unsafe string? ExportSceneCapture()
	{
		try
		{
			var scene = new SceneDefinition();
			scene.FillFromActiveLayout();
			var cacheKey = GetCacheKey(scene);
			var customization = NavmeshCustomizationRegistry.ForTerritory(scene.TerritoryID);

			var extractor = new SceneExtractor(scene);
			customization.CustomizeScene(extractor);

			var dir = new DirectoryInfo($"{Service.PluginInterface.GetPluginConfigDirectory()}/scenecapture");
			dir.Create();
			var path = $"{dir.FullName}/{cacheKey}.scene";
			SceneExtractorSerialization.SerializeToFile(extractor, path);
			Service.Log.Debug($"Exported scene capture for '{cacheKey}' ({extractor.Meshes.Count} meshes) to {path}");
			return path;
		}
		catch (Exception ex)
		{
			Service.Log.Error($"Failed to export scene capture: {ex}");
			return null;
		}
	}

	private static bool InCutscene => Service.Condition[ConditionFlag.WatchingCutscene] || Service.Condition[ConditionFlag.OccupiedInCutSceneEvent];

	public Task<List<Waypoint>> QueryPath(Vector3 from, Vector3 to, bool flying, float range = 0, CancellationToken externalCancel = default)
	{
		if (_currentCTS == null)
			throw new Exception($"Can't initiate query - navmesh is not loaded");

		// task can be cancelled either by internal request (i.e. when navmesh is reloaded) or external
		var combined = CancellationTokenSource.CreateLinkedTokenSource(_currentCTS.Token, externalCancel);
		++_numActivePathfinds;
		return ExecuteWhenIdle(async cancel =>
		{
			using var autoDisposeCombined = combined;
			using var autoDecrementCounter = new OnDispose(() => --_numActivePathfinds);
			LogInfo($"Kicking off pathfind from {from} to {to}");
			var path = await Task.Run(() =>
			{
				combined.Token.ThrowIfCancellationRequested();
				if (Query == null)
					throw new Exception($"Can't pathfind, navmesh did not build successfully");
				Log($"Executing pathfind from {from} to {to}");
				return flying ? Query.PathfindVolume(from, to, UseRaycasts, UseStringPulling, combined.Token) : Query.PathfindMesh(from, to, UseRaycasts, UseStringPulling, range, combined.Token);
			}, combined.Token);
			Log($"Pathfinding done: {path.Count} waypoints");
			return path;
		}, combined.Token);
	}

	public async Task<List<Vector3>> QueryPathBasic(Vector3 from, Vector3 to, bool flying, float range = 0, CancellationToken externalCancel = default)
	{
		var result = await QueryPath(from, to, flying, range, externalCancel);
		return [.. result.Select(w => w.Position)];
	}

	// note: pixelSize should be power-of-2
	public (Vector3 min, Vector3 max) BuildBitmap(Vector3 startingPos, string filename, float pixelSize, AABB? mapBounds = null)
	{
		if (Navmesh == null || Query == null || Navmesh.Ground == null)
			throw new InvalidOperationException($"Can't build bitmap - navmesh creation is in progress or ground graph is missing");

		var ground = Navmesh.Ground;
		bool inBounds(Quad q) => mapBounds is not AABB aabb || q.MinX >= aabb.Min.X && q.MinY >= aabb.Min.Y && q.MinZ >= aabb.Min.Z && q.MaxX <= aabb.Max.X && q.MaxY <= aabb.Max.Y && q.MaxZ <= aabb.Max.Z;

		var startPoly = Query.FindNearestMeshPoly(startingPos);
		var reachablePolys = Query.FindReachableMeshPolys(startPoly);

		HashSet<long> polysInbounds = [];

		Vector3 min = new(1024), max = new(-1024);
		foreach (var p in reachablePolys)
		{
			if (p < 0 || p >= ground.Quads.Count)
				continue;
			var q = ground.Quads[(int)p];
			if (!inBounds(q))
				continue;

			min = Vector3.Min(min, new(q.MinX, q.MinY, q.MinZ));
			max = Vector3.Max(max, new(q.MaxX, q.MaxY, q.MaxZ));
			polysInbounds.Add(p);
		}

		var bitmap = new NavmeshBitmap(min, max, pixelSize);
		foreach (var p in polysInbounds)
			bitmap.RasterizePolygon(ground.Quads[(int)p]);
		bitmap.Save(filename);
		Service.Log.Debug($"Generated nav bitmap '{filename}' @ {startingPos}: {bitmap.MinBounds}-{bitmap.MaxBounds}");
		return (bitmap.MinBounds, bitmap.MaxBounds);
	}

	// if non-empty string is returned, active layout is ready
	private unsafe string GetCurrentKey()
	{
		var layout = LayoutWorld.Instance()->ActiveLayout;
		if (layout == null || layout->InitState != 7 || layout->FestivalStatus is > 0 and < 5)
			return ""; // layout not ready

		var filter = LayoutUtils.FindFilter(layout);
		var filterKey = filter != null ? filter->Key : 0;

		var terrRow = Service.LuminaRow<Lumina.Excel.Sheets.TerritoryType>(filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId);

		// CE always has a festival layer (i hope). the non-festival layout is briefly loaded when entering the zone, which triggers a useless mesh build (which is also expensive because the zone is large)
		if (terrRow?.TerritoryIntendedUse.RowId == 60)
		{
			var fest = layout->ActiveFestivals[0];
			if (fest.Id == 0 && fest.Phase == 0)
				return "";
		}

		var sgs = LayoutUtils.GetZoneSharedGroupsEnabled(filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId);

		return $"{terrRow?.Bg}//{filterKey:X}//{LayoutUtils.FestivalsString(layout->ActiveFestivals)}//{string.Join('.', sgs)}";
	}

	internal static unsafe string GetCacheKey(SceneDefinition scene)
	{
		// note: festivals are active globally, but majority of zones don't have festival-specific layers, so we only want real ones in the cache key
		var layout = LayoutWorld.Instance()->ActiveLayout;
		var filter = LayoutUtils.FindFilter(layout);
		var filterKey = filter != null ? filter->Key : 0;
		var terrId = filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId;
		var terrRow = Service.LuminaRow<Lumina.Excel.Sheets.TerritoryType>(terrId);

		static string numbers<T>(IEnumerable<T> nums) where T : INumber<T> => string.Join('.', nums.Select(n => n.ToString("X", CultureInfo.InvariantCulture)));

		return $"{terrRow?.Bg.ToString().Replace('/', '_')}__{filterKey:X}__{numbers(scene.FestivalLayers)}__{numbers(scene.ZoneSGs)}";
	}

	private void ClearState()
	{
		if (_currentCTS == null)
			return; // already cleared

		var cts = _currentCTS;
		_currentCTS = null;
		cts.Cancel();
		Log("Queueing state clear");
		ExecuteWhenIdle(() =>
		{
			Log("Clearing state");
			_numActivePathfinds = 0;
			cts.Dispose();
			OnNavmeshChanged?.Invoke(null, null);
			Query = null;
			Navmesh = null;
		}, default);
	}

	private Navmesh BuildNavmesh(SceneDefinition scene, string cacheKey, bool allowLoadFromCache, CancellationToken cancel)
	{
		Log($"Build task started: '{cacheKey}'");
		var customization = NavmeshCustomizationRegistry.ForTerritory(scene.TerritoryID);
		Log($"Customization for '{scene.TerritoryID}': {customization.GetType()}");

		var layers = scene.FestivalLayers.ToList();

		// try reading from cache
		var cache = new FileInfo($"{_cacheDir.FullName}/{cacheKey}.navmesh");
		if (allowLoadFromCache && cache.Exists)
		{
			try
			{
				Log($"Loading cache: {cache.FullName}");
				using var stream = cache.OpenRead();
				using var reader = new BinaryReader(stream);
			var mesh = Navmesh.Deserialize(reader, customization.Version);
			if (mesh.Ground != null)
				customization.CustomizeGround(mesh.Ground, layers);
			return mesh;
			}
			catch (Exception ex)
			{
				Log($"Failed to load cache: {ex}");
			}
		}
		cancel.ThrowIfCancellationRequested();

		// cache doesn't exist or can't be used for whatever reason - build navmesh from scratch
		var builder = new NavmeshBuilder(scene, customization);
		var deltaProgress = 0.99f / (builder.NumTilesX * builder.NumTilesZ);
		builder.BuildTiles(() =>
		{
			_loadTaskProgress += deltaProgress;
			cancel.ThrowIfCancellationRequested();
		}, cancel);

		// write results to cache
		{
			Service.Log.Debug($"Writing cache: {cache.FullName}");
			using var stream = cache.Open(FileMode.Create, FileAccess.Write, FileShare.None);
			using var writer = new BinaryWriter(stream);
			builder.Navmesh.Serialize(writer);
		}
		if (builder.Navmesh.Ground != null)
			customization.CustomizeGround(builder.Navmesh.Ground, layers);
		deltaProgress += 0.01f;
		return builder.Navmesh;
	}

	private void ExecuteWhenIdle(Action task, CancellationToken token)
	{
		var prev = _lastLoadQueryTask;
		_lastLoadQueryTask = Service.Framework.Run(async () =>
		{
			await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
			_ = prev.Exception;
			task();
		}, token);
	}

	private void ExecuteWhenIdle(Func<CancellationToken, Task> task, CancellationToken token)
	{
		var prev = _lastLoadQueryTask;
		_lastLoadQueryTask = Service.Framework.Run(async () =>
		{
			await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
			_ = prev.Exception;
			var t = task(token);
			await t.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
			LogTaskError(t);
		}, token);
	}

	private Task<T> ExecuteWhenIdle<T>(Func<CancellationToken, Task<T>> task, CancellationToken token)
	{
		var prev = _lastLoadQueryTask;
		var res = Service.Framework.Run(async () =>
		{
			await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
			_ = prev.Exception;
			var t = task(token);
			await ((Task)t).ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
			LogTaskError(t);
			return t.Result;
		}, token);
		_lastLoadQueryTask = res;
		return res;
	}

	private static void Log(string message) => Service.Log.Debug($"[NavmeshManager] [{Environment.CurrentManagedThreadId}] {message}");
	private static void LogInfo(string message) => Service.Log.Info($"[NavmeshManager] [{Environment.CurrentManagedThreadId}] {message}");
	private static void LogTaskError(Task task)
	{
		if (task.IsFaulted)
			Service.Log.Error($"[NavmeshManager] Task failed with error: {task.Exception}");
	}

	public void PruneGround(GroundGraph.QuadGraph graph, IEnumerable<Vector3> points)
	{
		var seeds = points.Select(p => graph.NearestQuad(p)).Where(q => q >= 0).ToList();
		Log($"ground prune: seeding from {seeds.Count} quads");
		var reachable = graph.FloodReachable(seeds);
		graph.ApplyReachable(reachable);
		Log($"ground pruned {graph.Count - reachable.Count} unreachable quads (of {graph.Count})");
	}
}
