# PLAN2 — Replace DotRecast with a custom ground quad-graph pathfinder

This plan supersedes the previous voxel-only PLAN2. Flying pathfinding (voxel
A\* + LoS string-pull) is **done and committed** (Phases 0+1, see
`SESSION_NOTES.md`); it stays as-is. This plan replaces the DotRecast ground
mesh with a hand-written polygon/quad graph derived from the same `VoxelMap`
that already powers flight.

## Overview

Replace the DotRecast `DtNavMesh` ground pathfinder with a custom
polygon/quad graph built by greedy-meshing the walkable voxels of the
existing `VoxelMap` into axis-aligned quads, running A\* on the quad graph,
and string-pulling the result with the Simple Stupid Funnel Algorithm. The
`VoxelMap` (`Voxelizer` + `VoxelMap`) remains the single source of geometry
data; the quad graph is derived from it. The existing voxel A\*
(`VoxelPathfind`) + LoS string-pull (`VoxelStraighten`) continue to serve
flying pathfinding unchanged. After the quad pathfinder is verified, delete
the entire DotRecast submodule and every file that depends on it.

## Requirements

### Explicit
- Ground pathfinding uses a lightweight polygon/quad graph: greedy-mesh
  walkable voxels into quads, A\* on the quad graph, funnel string-pull.
- Flying pathfinding keeps the existing voxel A\* + LoS string-pull
  (committed Phases 0+1). Do not touch `VoxelPathfind.cs` /
  `VoxelStraighten.cs` / `VoxelSearch.cs` beyond reading them.
- The voxel grid (`Voxelizer` + `VoxelMap`) is the single data source. The
  quad graph is derived from `VoxelMap`; no separate rasterization.
- IPC surface is byte-for-byte stable (see AGENTS.md). Every registered
  `vnavmesh.*` method keeps its name, argument types, and return types.
- Each phase is a separate commit.
- No deferring, no aborting on size. Prefer complete implementations.
- No code comments unless asked.
- New files live under `vnavmesh/GroundGraph/` and `vnavmesh/Movement/`.

### Inferred
- `NavmeshQuery` public method names/signatures stay identical so
  `NavmeshManager`, `IPCProvider`, `DebugNavmeshManager`,
  `DebugNavmeshCustom`, `BuildBitmap`, and `Prune` keep compiling.
  Internally the ground methods switch from `DtNavMesh` to `QuadGraph`.
- The opaque polygon handle type stays `long` (used by
  `FindNearestMeshPoly`, `FindReachableMeshPolys`, `BuildBitmap`,
  `Prune`, `GetAreaId`). Quad indices are cast to `long`; the high bits are
  zero. This preserves every signature.
- `Navmesh.AreaId` enum stays (Warp/ClientPath/Shortcut/Endpoint/Default).
  Quads carry an `AreaId`; off-mesh connections are synthetic adjacency edges
  with an `AreaId` and a cost multiplier.
- Build still produces a `DtNavMesh` during Phases 1–9 (it is unused for
  ground pathfinding from Phase 8 on, but keeps the Recast build path and
  customizations compiling until Phase 10 deletes it). Build time stays slow
  until Phase 10; correctness first.
- `NavmeshCustomization.CustomizeScene` (scene collider tweaks) and
  constructor `Settings` tweaks that affect voxelization (AgentRadius,
  AgentHeight, AgentMaxClimb, AgentMaxSlope, Filtering flags) remain
  relevant because the `Voxelizer` consumes them. `Settings.Partitioning`
  and the polygonization settings (`PolyMaxEdgeLen`,
  `PolyMaxSimplificationError`, `PolyMaxVerts`, `DetailSampleDist`,
  `DetailMaxSampleError`, `RegionMinSize`, `RegionMergeSize`,
  `GenerateEdgeClimbLinks`, `GenerateEdgeJumpLinks`, jump-link settings)
  become dead after Phase 10 and are removed then.

### Assumptions made
- A voxel is **walkable ground** iff it is empty (`VoxelMap.IsEmpty` true)
  AND the voxel directly below it (same x,z, y-1) is solid (occupied). The
  quad surface Y is the top of that solid voxel (the floor the player stands
  on). This matches how `NavmeshRasterizer` already marks solid spans and how
  the game places the player on top of solid geometry.
- `VoxelMap` leaf-voxel resolution (L3 = `Settings.NumTiles[2]` per tile
  axis, currently 8 → 2-voxel leaves) is fine enough for ground quads. If
  in-game testing shows quads too coarse, the fix is bumping `NumTiles[2]`,
  not changing the mesher.
- The quad graph is small enough (order 10^3–10^4 quads per zone) that a flat
  array + binary-heap A\* with open-addressing lookup (mirroring
  `VoxelPathfind`'s structures) is fast enough. No spatial hash index needed
  for A\*; a per-axis sorted edge list is enough for adjacency build.
- Funnel portals are the shared-edge segments between adjacent quads,
  clamped to the overlapping span. This is the standard Detour portal
  definition, which makes the Simple Stupid Funnel Algorithm directly
  applicable.

## Out of Scope
- Replacing or altering the flying voxel A\* / LoS string-pull (Phases 0+1).
- Changing the `Voxelizer` rasterization pipeline or `SceneExtractor`.
- Multi-floor / 3D-overlap ground graphs. Quads are flat (minY == maxY per
  quad); vertical transitions are adjacency edges with a climb check, same
  as Recast's walkable-climb. Overlapping floors at different Y are
  separate quads connected by climb-adjacency, not stacked polygons.
- Upstream DotRecast rebase (the aborted Phase 2). This plan deletes
  DotRecast entirely instead.
- New IPC methods. Only existing methods are preserved.

## Existing Patterns

- `vnavmesh/NavVolume/VoxelPathfind.cs` — A\* template to mirror for
  `QuadPathfind`: `Node` struct (GScore/HScore/ParentIndex/OpenHeapIndex/
  Position), flat `int[] _lookupTable` open-addressing hash, `List<int>`
  binary heap (`PercolateUp`/`PercolateDown`/`HeapLess`), `Execute` with
  `maxSteps` + cancellation every 1024 steps, `BuildPathToVisitedNode`
  backwalk. Copy this structure; swap `ulong Voxel` → `int QuadId`.
- `vnavmesh/NavVolume/VoxelMap.cs` — voxel encoding (`EncodeIndex`/
  `DecodeIndex`/`IndexToVoxel`/`VoxelToIndex`), `Tile.WorldToVoxel`/
  `VoxelToWorld`, `FindLeafVoxel`, `EnumerateLeafVoxels(bmin,bmax)`,
  `VoxelBounds(voxel,eps)`, `IsEmpty(voxel)`. The quad mesher consumes these.
- `vnavmesh/NavVolume/VoxelSearch.cs` — `FindClosestVoxelPoint`,
  `FindNearestEmptyVoxel`, `EnumerateVoxelsInLine`, `LineOfSight`. Reused by
  flying; the quad pathfinder does not need LoS (funnel replaces it).
- `vnavmesh/NavmeshQuery.cs` — entry points to rewrite:
  `PathfindMesh` (ground, → `QuadPathfind`), `PathfindVolume` (fly, keep),
  `FindNearestMeshPoly`/`FindIntersectingMeshPolys`/
  `FindNearestPointOnMeshPoly`/`FindNearestPointOnMesh`/`FindPointOnFloor`/
  `FindReachableMeshPolys`/`FindNearestVolumeVoxel`/`GetAreaId`. The
  `IntersectQuery`/`GoalRadiusHeuristic`/`TeleportAwareFilter`/
  `FloodFillAwareFilter` inner classes are DotRecast-coupled and are
  replaced by quad-graph equivalents in Phase 8.
- `vnavmesh/NavmeshManager.cs` — lifecycle: `Reload` → `BuildNavmesh` →
  `NavmeshBuilder.BuildTiles` → `Prune` (flood-fill reachable polys from
  aetheryte seeds via `FloodFill.GetAsync()`). `QueryPath` dispatches
  `flying ? PathfindVolume : PathfindMesh`. `BuildBitmap` uses
  `FindNearestMeshPoly` + `FindReachableMeshPolys` +
  `Navmesh.Mesh.GetTileAndPolyByRefUnsafe`. All these switch to the quad
  graph in Phase 8 (pathfinding) / Phase 5 (pruning) / Phase 10 (bitmap).
- `vnavmesh/Navmesh.cs` — `record class Navmesh(int CustomizationVersion,
  DtNavMesh Mesh, VoxelMap? Volume)`; `Magic=0x444D564E`, `Version=24`;
  `Serialize`/`Deserialize` write Mesh then Volume via Brotli. Phase 5 adds
  QuadGraph serialization; Phase 10 removes Mesh serialization and bumps
  `Version`.
- `vnavmesh/NavmeshBuilder.cs` — builds `DtNavMesh` tiles via the full
  Recast pipeline (heightfield → compact → regions → contours → polymesh →
  detail → jumplinks → `DtNavMeshBuilder.CreateNavMeshData`) AND builds
  `VoxelMap` from the `Voxelizer`. Phase 10 strips the Recast pipeline; the
  quad graph is built from `VoxelMap` after `BuildTiles` (added in Phase 1).
- `vnavmesh/NavmeshRasterizer.cs` — rasterizes scene triangles into
  `RcHeightfield` + `Voxelizer`. Phase 10 removes the `RcHeightfield` path
  and all Recast-specific span manipulation (`AddSpan`, `FillInterior`
  intersection set); keeps only the `Voxelizer`-feeding path (which is the
  `if (includeInVolume && _voxelizer != null)` block at the end of `AddSpan`
  and the `voxelizer`-aware branch in `RasterizeMesh`). This is the single
  hardest deletion; it is sequenced last.
- `vnavmesh/NavmeshCustomization.cs` — `LinkPoints`/`InsertPointPoly`/
  `AddOffMeshConnection` manipulate `DtNavMesh` links. Phase 4 rewrites these
  for `QuadGraph`; Phase 10 deletes the DtNavMesh versions. `CustomizeScene`
  and the `SceneExtensions.InsertAABoxCollider`/`InsertCylinderCollider`
  helpers are DotRecast-free and stay.
- `vnavmesh/Customizations/*.cs` — 34 files. Only `Z0132NewGridania.cs`
  (3 `LinkPoints` + 1 `AddOffMeshConnection`) and
  `Z1252OccultCrescentSouthHorn.cs` (12 `AddOffMeshConnection`) use the link
  APIs. `Z0363TheLostCityofAmdapor.cs` and `Z1041BrayfloxsLongstop.cs` set
  `Settings.Partitioning = RcPartition.MONOTONE` (becomes a no-op after
  Phase 10; rewritten to drop the line). The rest use only `CustomizeScene`
  (collider flags/removal) and constructor `Settings` field tweaks — all
  DotRecast-free.
- `vnavmesh/Movement/FollowPath.cs` — `Update` consumes `Waypoint`s,
  `OverrideMovement.DesiredPosition = Waypoints[0].Position`, stuck
  detection, jump-to-fly transition. `Move(waypoints, ignoreDeltaY,
  destTolerance)`. Phase 6 adds spline smoothing between `Move` and
  `Update`; Phase 7 adds velocity-aware modulation in `Update` +
  `OverrideMovement`.
- `vnavmesh/Movement/OverrideMovement.cs` — `RMIWalkDetour`/
  `RMIFlyDetour` set `sumLeft`/`sumForward`/`Forward`/`Left` to the full
  normalized direction (magnitude 1, binary on/off). `DirectionToDestination`
  returns `(h, v)` angles relative to player/legacy heading. Phase 7
  modulates the magnitude (PD ease-in/out + corner speed) instead of always
  1.0.
- `vnavmesh/IPCProvider.cs` — full IPC surface (do not break). All
  `RegisterFunc`/`RegisterAction` calls stay. Ground methods delegate to
  `NavmeshQuery`; they keep working when `NavmeshQuery` internals switch.
- `vnavmesh/Extensions.cs` — `SystemToRecast`/`RecastToSystem`/
  `Floor`/`Ceiling`. Deleted in Phase 10; `Floor`/`Ceiling` move to a
  DotRecast-free home (inline or a small `MathEx`).
- `vnavmesh/Debug/*.cs` — 16 files. `DebugCompactHeightfield`,
  `DebugContourSet`, `DebugDetourNavmesh`, `DebugPolyMesh`,
  `DebugPolyMeshDetail`, `DebugSolidHeightfield`, `DebugRecast`,
  `DebugNavmeshCustom`, `DebugExtractedCollision` reference DotRecast
  types directly. Phase 10 deletes the Recast-only debug files and rewrites
  `DebugNavmeshManager`/`DebugNavmeshCustom` to visualize `QuadGraph`
  instead of `DtNavMesh`; adds a new `DebugQuadGraph.cs`.
- `vnavmesh/vnavmesh.csproj` — 4 `<ProjectReference>` to
  `DotRecast/src/...`. Phase 10 removes all four + deinit submodule.
- `vnavmesh/NavmeshSettings.cs` — exposes Recast-specific enums
  (`RcPartition`) and polygonization settings via `ImGui` controls. Phase
  10 removes the Recast-only controls; keeps voxelization-relevant settings
  (`CellSize`, `CellHeight`, `AgentHeight`, `AgentRadius`, `AgentMaxClimb`,
  `AgentMaxSlopeDeg`, `Filtering`, `NumTiles`).
- `vnavmesh/NavmeshBitmap.cs` — `RasterizePolygon(Navmesh.Mesh, p)` walks
  `DtNavMesh` polys. Phase 10 rewrites `RasterizePolygon` to rasterize
  `Quad` axis-aligned rectangles (trivial: a quad is an XZ rectangle; fill
  the bitmap rows directly). `BuildBitmap` in `NavmeshManager` switches to
  quad-graph reachable-set.
- `vnavmesh/Config.cs` — no DotRecast refs; untouched except any new
  movement/spline tunables (Phase 7).
- `vnavmesh/FloodFill.cs` — seeds aetheryte positions; consumed by
  `NavmeshManager.Prune`. Stays; `Prune` switches to quad-graph flood in
  Phase 5.

## Architecture Decision

**Recommended approach:** Derive a flat quad graph from the existing
`VoxelMap` by greedy-meshing walkable-ground voxels into axis-aligned
rectangles, connect quads by shared edges with a climb-height check, run A\*
on the quad graph with a Euclidean heuristic and area-cost multipliers, and
string-pull the result with the Simple Stupid Funnel Algorithm over quad
portals. Ground `NavmeshQuery` methods switch to this graph; flying stays
on the voxel A\*. The quad graph is serialized per-zone alongside the
`VoxelMap`; the `DtNavMesh` and the entire Recast build pipeline are deleted
once ground pathfinding is verified in-game.

**Why:** The `VoxelMap` already encodes all geometry needed for ground
pathfinding (empty-above-solid = walkable). Recast's heightfield→contour→
polygon pipeline is redundant work whose only output (`DtNavMesh`) we are
replacing. A quad graph over the same voxels is simpler, smaller, builds
faster, and its portals map directly onto the well-understood funnel
algorithm. Keeping the `VoxelMap` as the single source means flying and
ground share one rasterization and one cache.

**Alternatives considered and rejected:**
1. *Port to upstream DotRecast.* Aborted in the old Phase 2 (220 errors, 22
   files, full API divergence). Rejected again: this plan removes the
   dependency instead of migrating it.
2. *Voxel A\* for ground too.* Rejected (SESSION_NOTES decisions log: keep
   two specialized pathfinders; do not unify). Voxel A\* is 3D and
   over-resolved for ground; quads give a 2D graph with clean portals for
   funnel string-pulling, which voxel LoS string-pulling cannot match for
   ground smoothness.
3. *Triangle mesh instead of quads.* Rejected: greedy-meshing voxels yields
   axis-aligned quads trivially (O(N) sweep); triangulating contours would
   reintroduce a Recast-like contour+triangulation pipeline. Quads are
   convex and funnel-compatible; the only loss is diagonal edges, which
   don't matter for axis-aligned FFXIV geometry.
4. *Keep building DtNavMesh forever, quad graph as overlay.* Rejected: the
   point is to delete DotRecast (~3500 lines + submodule + slow build).

## Implementation Plan

> Build command (every phase): `DALAMUD_HOME=~/.cache/dalamud-dev DOTNET_ROOT=~/.dotnet ~/.dotnet/dotnet build vnavmesh/vnavmesh.csproj -c Release -p:Platform=x64`
> Output: `vnavmesh/bin/x64/Release/vnavmesh.dll`. Clean gate: 0 errors, 0
> warnings (the 3 pre-existing DotRecast warnings are ignored only while
> DotRecast is still referenced — after Phase 10 they must be gone).
> Commit per phase. Rollback per phase = `git revert <phase-commit>`.

### Phase 1 — Ground quad mesher

Why this phase: the quad graph's data structure and its construction from
`VoxelMap`. No consumers yet; `NavmeshBuilder` produces a `QuadGraph` and
stores it on `Navmesh`, but nothing reads it. Keeps the build compiling with
both `DtNavMesh` and `QuadGraph` present.

- [ ] Task 1.1: Create `vnavmesh/GroundGraph/QuadMesher.cs` with `namespace
  Navmesh.GroundGraph`. Define `public readonly record struct Quad(float
  MinX, float MinY, float MinZ, float MaxX, float MaxY, float MaxZ,
  Navmesh.AreaId Area)` (world-space AABB of the walkable surface patch;
  for a flat ground quad `MinY == MaxY == surfaceY`; the XZ span is the
  footprint). Add `Vector3 Center` computed property, `Vector2 MinXZ`/
  `MaxXZ` computed properties, and `bool ContainsXZ(Vector3 p)`.
- [ ] Task 1.2: In `QuadMesher.cs`, add `public static QuadGraph GreedyMesh(
  VoxelMap volume, Vector3 boundsMin, Vector3 boundsMax)`. Iterate the
  `VoxelMap` leaf voxels via `volume.RootTile.EnumerateLeafVoxels(bmin,
  bmax)` over the full bounds. For each empty leaf voxel at `(x,y,z)`,
  query the voxel below at `(x,y-1,z)` via `volume.FindLeafVoxel` at the
  world point of that voxel's center; if that lower voxel is solid
  (`!empty`), mark the upper voxel as walkable ground at surface Y =
  `VoxelToWorld(x, y-1, z).Y + cellSizeY/2` (top of the solid voxel). Use a
  `HashSet<ulong>` of occupied-below voxel indices to avoid re-testing.
- [ ] Task 1.3: In `QuadMesher.cs`, implement the greedy-mesh sweep:
  maintain a `bool[,] occupied` 2D grid per Y-slice (indexed by leaf voxel
  x,z within the level). For each distinct surface-Y, sweep rows in z: for
  each unvisited walkable cell, extend maximally in x to form a row strip,
  then extend the strip maximally in z while the x-range stays identical,
  emit one `Quad` with `Area = Navmesh.AreaId.Default`, and mark all
  covered cells visited. Target ~150 lines for the whole file.
- [ ] Task 1.4: Create `vnavmesh/GroundGraph/QuadGraph.cs` with `namespace
  Navmesh.GroundGraph` and `public class QuadGraph`. Fields: `public
  List<Quad> Quads`, `public List<List<int>> Adjacency` (per-quad edge
  list, populated in Phase 2), `public Vector3 BoundsMin`, `public Vector3
  BoundsMax`, `public int Count => Quads.Count`. Constructor takes
  `(Vector3 boundsMin, Vector3 boundsMax)`. `AddQuad(Quad q) -> int` returns
  the new quad index and appends an empty adjacency list. Target ~80 lines
  (this file holds adjacency too, populated in Phase 2).
- [ ] Task 1.5: In `vnavmesh/Navmesh.cs`, add `public QuadGraph? Ground`
  field to the `Navmesh` record (`record class Navmesh(int
  CustomizationVersion, DtNavMesh Mesh, VoxelMap? Volume, QuadGraph?
  Ground)`). Update the two `new(...)` call sites (`NavmeshBuilder` ctor and
  `Navmesh.Deserialize`) to pass `null` for `Ground` explicitly. Bump
  `Navmesh.Version` from 24 to 25 here so all existing caches invalidate
  immediately and `Ground` is never loaded as null from a stale cache.
  Phase 5 wires ground serialization; until then cached zones rebuild from
  scratch and populate `Ground` via Task 1.6.
- [ ] Task 1.6: In `vnavmesh/NavmeshBuilder.cs`, after `BuildTiles` merges
  all `VoxelMap` tiles into `Navmesh.Volume` (the loop at lines 143–153),
  if `Navmesh.Volume != null` build the quad graph:
  `Navmesh.Navmesh = Navmesh.Navmesh with { Ground = QuadMesher.GreedyMesh(
  Navmesh.Volume, BoundsMin, BoundsMax) }`. The `Navmesh` record is
  immutable so reassign via `with`. Add `using Navmesh.GroundGraph;`.
- [ ] **Checkpoint: Verify Phase 1 complete.** Confirm `QuadMesher.cs`,
  `QuadGraph.cs` exist with the specified structs/methods; `Navmesh.cs`
  record has the `Ground` field; `NavmeshBuilder.cs` calls `GreedyMesh`
  after volume merge. No stubs. Build must pass.

**Verification (Phase 1):**
- Build: 0 errors, 0 warnings (DotRecast still present, 3 pre-existing
  warnings tolerated).
- Static: `rg "GreedyMesh" vnavmesh/` → 2 matches (def + call).
  `rg "public QuadGraph\? Ground" vnavmesh/Navmesh.cs` → 1.
- In-game (deferred to Phase 11; no consumer yet): none.

**Rollback:** `git revert <phase1-commit>`. Removes `GroundGraph/`, reverts
`Navmesh.cs` record and `NavmeshBuilder.cs` call. Build returns to
Phase-1-pre state.

### Phase 2 — Quad adjacency builder

Why this phase: connect quads so A\* has edges. Still no pathfinding; only
graph structure. Depends on Phase 1's `QuadGraph`.

- [ ] Task 2.1: In `vnavmesh/GroundGraph/QuadGraph.cs`, add
  `public float MaxClimb` field (set from `NavmeshSettings.AgentMaxClimb`
  by the builder). Add `public void BuildAdjacency(float maxClimb)` that
  fills `Adjacency` from `Quads`. Store `MaxClimb = maxClimb`.
- [ ] Task 2.2: Implement `BuildAdjacency` as a sweep-line over quad edges.
  Build two sort indices: quads sorted by `MinX` (for Z-aligned edges, i.e.
  edges where `MinX == MaxX`) and by `MinZ` (for X-aligned edges where
  `MinZ == MaxZ`). For each quad, emit its two candidate-edge sides: the
  `+X` side `(MaxX, MinZ..MaxZ, Y)` and the `-X` side `(MinX, MinZ..MaxZ,
  Y)`, similarly `+Z`/`-Z`. For each emitted edge, binary-search the
  opposite-side index for edges on the same plane (same X or same Z) whose
  Y-difference `<= maxClimb` and whose XZ-span overlaps by ≥ one voxel.
  Overlap test: `max(a.MinSpan, b.MinSpan) < min(a.MaxSpan, b.MaxSpan)`.
  For each match, add `b` to `Adjacency[a]` and `a` to `Adjacency[b]`
  (bidirectional). The overlap segment is the portal (stored in Phase 3).
- [ ] Task 2.3: Add `public record struct Portal(int FromQuad, int ToQuad,
  Vector2 SpanMin, Vector2 SpanMax, float YFrom, float YTo, bool IsOffMesh,
  Navmesh.AreaId Area)` to `QuadGraph.cs` and a `public List<Portal>
  Portals` field. Populate `Portals` inside `BuildAdjacency` for each
  adjacency edge (the overlap span; `IsOffMesh=false`, `Area=Default`).
  Off-mesh connections from Phase 4 append to `Portals` with
  `IsOffMesh=true`.
- [ ] Task 2.4: In `vnavmesh/NavmeshBuilder.cs`, after the `GreedyMesh` call
  added in Task 1.6, call `Navmesh.Navmesh.Ground!.BuildAdjacency(
  Settings.AgentMaxClimb)`.
- [ ] **Checkpoint: Verify Phase 2 complete.** `BuildAdjacency` and `Portal`
  exist; `NavmeshBuilder` calls it; `Adjacency` and `Portals` are
  non-empty for a built zone. No stubs.

**Verification (Phase 2):**
- Build: 0 errors, 0 warnings.
- Static: `rg "BuildAdjacency" vnavmesh/` → 2. `rg "record struct Portal"`
  vnavmesh/` → 1. `rg "Portals" vnavmesh/GroundGraph/` → matches.
- In-game (deferred): none (no pathfinding yet).

**Rollback:** `git revert <phase2-commit>`. Restores `QuadGraph.cs` to
Phase-1 shape (empty `Adjacency`), reverts the `NavmeshBuilder` call.

### Phase 3 — Quad A\* + funnel string-pull

Why this phase: the actual ground pathfinder. Mirrors `VoxelPathfind`'s
A\* structures and implements the Simple Stupid Funnel Algorithm. Still
not wired into `NavmeshQuery` (Phase 8); testable via a temporary internal
hook or a debug command. Depends on Phases 1–2.

- [ ] Task 3.1: Create `vnavmesh/GroundGraph/QuadPathfind.cs` with
  `namespace Navmesh.GroundGraph`. Define `public struct QNode { public
  float GScore; public float HScore; public int QuadId; public int
  ParentIndex; public int OpenHeapIndex; public Vector3 EnterPos; }`.
  Mirror `VoxelPathfind`'s open-addressing `int[] _lookupTable` + `int
  _lookupMask`, `List<int> _openList` binary heap, `VoxelHash`-style mix
  (reuse the Finalizer constants), `InitLookup`/`LookupGet`/`LookupSet`/
  `LookupGrow`, `PercolateUp`/`PercolateDown`/`HeapLess`/`PopMinOpen`. Key
  is `int QuadId` (not `ulong`).
- [ ] Task 3.2: In `QuadPathfind.cs`, add `private QuadGraph _graph;`
  `private List<QNode> _nodes`, `int _bestNodeIndex`, `int _goalQuad`,
  `Vector3 _goalPos`, `bool _useRaycast`, `float _raycastLimitSq`. Add
  `public QuadPathfind(QuadGraph graph)` and `public List<(int quad,
  Vector3 p)> FindPath(int fromQuad, int toQuad, Vector3 fromPos, Vector3
  toPos, bool useRaycast, CancellationToken cancel)`. `Start` seeds the
  start node (HScore = Euclidean `fromPos→toPos`), `Execute` loops
  `ExecuteStep` with `maxSteps` + cancellation every 1024 steps.
- [ ] Task 3.3: Implement `ExecuteStep`: pop min, enumerate neighbours from
  `_graph.Adjacency[curQuad]` + off-mesh portals whose `FromQuad == curQuad`
  (from `_graph.Portals`). For each neighbour quad, compute the enter
  position as the portal midpoint projected onto the neighbour quad's
  surface (clamp `EnterPos` into the neighbour quad's XZ footprint at its
  surface Y). `VisitNeighbour` mirrors `VoxelPathfind.VisitNeighbour`:
  lookup-or-create node, compute G via `CalculateGScore`, relax if better,
  update `bestNodeIndex` on HScore improvement. Add the area-cost multiplier
  for off-mesh portals: `Warp→×1, ClientPath→×3, Shortcut→×8, Default→×10`
  (matches `NavmeshQuery.TeleportAwareFilter`).
- [ ] Task 3.4: Implement `CalculateGScore(ref QNode parent, int destQuad,
  Vector3 destPos, ref int parentIndex)`: if `_useRaycast`, check grandparent
  LoS via a 2D XZ segment-quad intersection test against all quads between
  grandparent and dest (a quad is a rectangle; segment-rectangle test in
  XZ). If clear, shortcut to grandparent. Otherwise base distance =
  `(parent.EnterPos - destPos).Length()` + vertical penalty `0.2f *
  abs(dy)` (same as `VoxelPathfind`). Return `parentBaseG + baseDistance +
  verticalPenalty`.
- [ ] Task 3.5: Implement `BuildPathToVisitedNode(int nodeIndex, bool
  returnIntermediate)` backwalk mirroring `VoxelPathfind`'s; returns
  `List<(int quad, Vector3 p)>` from start to the best node.
- [ ] Task 3.6: Create `vnavmesh/GroundGraph/FunnelStringPull.cs` with
  `namespace Navmesh.GroundGraph` and `public static class FunnelStringPull`.
  Implement the Simple Stupid Funnel Algorithm: input `List<(int quad,
  Vector3 p)> path` (quad corridor from A\*) + `Vector3 fromPos` +
  `Vector3 toPos`. Build the portal list by walking the corridor: for each
  consecutive quad pair, find the `Portal` in `_graph.Portals` matching
  `(FromQuad, ToQuad)`; the portal's XZ overlap span + Y gives the two
  portal edge endpoints in 3D. Run the standard funnel: maintain `apex`
  (start), `left`/`right` portal vertices; for each portal, advance apex
  when the funnel degenerates, else tighten left/right. Output
  `List<Vector3>` waypoints starting at `fromPos`, through apex moves,
  ending at `toPos`. Target ~120 lines for this file. (Total Phase 3 ~270
  lines across the two files.)
- [ ] Task 3.7: Add a `public List<Vector3> Pathfind(Vector3 from, Vector3
  to, bool useRaycast, bool useStringPulling, float range, CancellationToken
  cancel)` method on `QuadGraph` that: finds the nearest quad to `from`
  and `to` (XZ containment, fallback nearest center within a half-extent),
  runs `QuadPathfind.FindPath`, then (if `useStringPulling`)
  `FunnelStringPull.Pull(...)`, else maps corridor nodes to quad centers.
  Handle `range > 0` as a goal-radius early-accept: in `ExecuteStep`, if
  `(EnterPos - _goalPos).Length() <= range`, treat as goal. This is the
  method `NavmeshQuery.PathfindMesh` will call in Phase 8.
- [ ] **Checkpoint: Verify Phase 3 complete.** `QuadPathfind.cs` and
  `FunnelStringPull.cs` exist with all listed methods; `QuadGraph.Pathfind`
  compiles. No stubs. The A\* structures mirror `VoxelPathfind`. Build
  passes.

**Verification (Phase 3):**
- Build: 0 errors, 0 warnings.
- Static: `rg "class QuadPathfind" vnavmesh/` → 1. `rg "FunnelStringPull"
  vnavmesh/` → 2. `rg "_lookupTable" vnavmesh/GroundGraph/` → matches
  (mirrors VoxelPathfind). `rg "Simple Stupid|funnel" vnavmesh/ -i` → no
  stray references in other files.
- In-game (deferred to Phase 11): none (not wired yet).

**Rollback:** `git revert <phase3-commit>`. Removes `QuadPathfind.cs`,
`FunnelStringPull.cs`, `QuadGraph.Pathfind`. Quads + adjacency (Phases
1–2) remain.

### Phase 4 — Area annotations on quads + territory markup port

Why this phase: give quads area flags (Warp/ClientPath/Shortcut) and port
the off-mesh-connection customizations to the quad graph. The old
DotRecast `LinkPoints`/`AddOffMeshConnection` stay (unused after this phase
by customizations) until Phase 10 deletes them. Depends on Phases 1–3
(portal list exists).

- [ ] Task 4.1: In `vnavmesh/GroundGraph/QuadGraph.cs`, add `public void
  SetArea(int quadId, Navmesh.AreaId area)` that OR-sets `Quads[quadId].Area`
  (records are immutable; replace via index assignment using a mutable
  backing or a `Quad[]` — switch `Quads` from `List<Quad>` to a `Quad[]`-
  backed list with an indexer setter, or keep `List<Quad>` and reassign the
  element). Add `public void MarkAreaBox(Vector3 min, Vector3 max,
  Navmesh.AreaId area)` that sets the area on every quad whose XZ footprint
  intersects the box and whose surface Y is within `[min.Y, max.Y]`.
- [ ] Task 4.2: In `QuadGraph.cs`, add `public int AddOffMesh(Vector3 a,
  Vector3 b, Navmesh.AreaId area, float radius=0.5f, bool bidirectional=
  false, int userId=0)` that: finds the nearest quad to `a` (quadA) and to
  `b` (quadB) via XZ containment + nearest-center fallback within `radius`+
  half-extent; appends a `Portal` with `FromQuad=quadA, ToQuad=quadB,
  SpanMin/Max = the endpoint XZ clamped to each quad's footprint,
  YFrom=a.Y, YTo=b.Y, IsOffMesh=true, Area=area`; appends the reverse portal
  if `bidirectional`; adds `quadB` to `Adjacency[quadA]` and vice-versa (if
  bidirectional). Returns `quadA` (or -1 if either endpoint has no quad).
  This replaces `CreateParamsExtensions.AddOffMeshConnection`.
- [ ] Task 4.3: In `vnavmesh/NavmeshCustomization.cs`, add a new virtual
  `public virtual void CustomizeGround(QuadGraph graph, List<uint>
  festivalLayers) { }` to `NavmeshCustomization`. This is the quad-graph
  counterpart of `CustomizeMesh`. Add a protected `static (int a, int b)
  LinkQuads(QuadGraph graph, Vector3 a, Vector3 b, Navmesh.AreaId area =
  Navmesh.AreaId.ClientPath)` that calls `graph.AddOffMesh(a, b, area |
  Navmesh.AreaId.Endpoint)` and returns the two quad ids. This replaces
  `LinkPoints`.
- [ ] Task 4.4: In `vnavmesh/NavmeshManager.cs:BuildNavmesh`, after
  `customization.CustomizeMesh(builder.Navmesh, layers)` (line 310) and
  after the cache-load path (line 285), add
  `customization.CustomizeGround(navmesh.Ground!, layers)` when
  `navmesh.Ground != null`. This calls the new quad-graph customization
  hook for both the fresh-build and cache-load branches.
- [ ] Task 4.5: Rewrite `vnavmesh/Customizations/Z0132NewGridania.cs`:
  keep `CustomizeScene` (planter height tweak — voxelization-relevant,
  unchanged). Replace `CustomizeSettings(config => config.AddOffMeshCon
  nection(...))` with `CustomizeGround(graph => graph.AddOffMesh(new(45.03f,
  -0.13f, 83.1f), new(46.78f, -8.5f, 91.75f)))`. Replace the three
  `LinkPoints(mesh, ...)` calls in `CustomizeMesh` with `LinkQuads(graph,
  ...)` moved into `CustomizeGround`. Bump `Version => 3` (cache
  invalidation). Keep `CustomizeMesh` calling `base.CustomizeMesh` (still
  needed for the DtNavMesh path until Phase 10).
- [ ] Task 4.6: Rewrite `vnavmesh/Customizations/Z1252OccultCrescentSouth
  Horn.cs`: keep `CustomizeScene` (stair vertex tweak). Move all 12
  `AddOffMeshConnection` calls from `CustomizeSettings` into a new
  `override CustomizeGround(graph => { graph.AddOffMesh(...); ... })`.
  Bump `Version => 5`. Keep `CustomizeSettings` empty (or remove it).
- [ ] Task 4.7: Update the two `RcPartition.MONOTONE` customizations
  (`Z0363TheLostCityofAmdapor.cs`, `Z1041BrayfloxsLongstop.cs`): leave the
  `Settings.Partitioning = RcPartition.MONOTONE` line in place for now
  (still consumed by the Recast build until Phase 10). Add a comment-free
  note in the commit message that these become no-ops after Phase 10. Do
  not touch the other 30 customizations (they only use `CustomizeScene` /
  ctor `Settings`, both DotRecast-free).
- [ ] **Checkpoint: Verify Phase 4 complete.** `QuadGraph.SetArea`/
  `MarkAreaBox`/`AddOffMesh` exist; `NavmeshCustomization.CustomizeGround` +
  `LinkQuads` exist; `NavmeshManager.BuildNavmesh` calls `CustomizeGround`;
  `Z0132` and `Z1252` use `CustomizeGround`/`LinkQuads`/`AddOffMesh`. The old
  `LinkPoints`/`AddOffMeshConnection` still compile (Phase 10 deletes them).
  Build passes.

**Verification (Phase 4):**
- Build: 0 errors, 0 warnings.
- Static: `rg "CustomizeGround" vnavmesh/` → ≥4 (def + manager call + 2
  customizations). `rg "LinkQuads" vnavmesh/` → ≥3. `rg "AddOffMesh\(" vnav
  mesh/GroundGraph/` → 1 (def). `rg "LinkPoints" vnavmesh/Customizations/`
  → 0 (all ported). `rg "AddOffMeshConnection" vnavmesh/Customizations/`
  → 0 (all ported).
- In-game (deferred): none (ground path not wired until Phase 8).

**Rollback:** `git revert <phase4-commit>`. Reverts the two customization
files to their DotRecast `LinkPoints`/`AddOffMeshConnection` forms, removes
`CustomizeGround`/`LinkQuads`/`AddOffMesh`/`SetArea`/`MarkAreaBox`, reverts
`BuildNavmesh`. Customizations work again via the old path.

### Phase 5 — Reachability pruning on quad graph + serialization

Why this phase: prune unreachable quads from aetheryte seeds (so pathfinding
never routes into walled-off areas) and persist the quad graph per zone.
Depends on Phases 1–4 (graph + off-mesh portals exist). Replaces the
`DtNavMesh`-based `NavmeshManager.Prune` for the ground graph; the
`DtNavMesh` prune stays until Phase 10 (harmless duplicate work).

- [ ] Task 5.1: In `vnavmesh/GroundGraph/QuadGraph.cs`, add `public const
  int FLAG_UNREACHABLE = 0x10;` (mirrors `Navmesh.FLAG_UNREACHABLE`). Add
  `public int[] Flags` (per-quad flags, sized to `Quads.Count`, default 1 =
  reachable). Add `public HashSet<int> FloodReachable(IEnumerable<int>
  seeds)` that BFS-walks `Adjacency` from the seeds (including off-mesh
  portals) and returns the reachable set. Add `public void ApplyReachable(
  HashSet<int> reachable)` that sets `FLAGS_UNREACHABLE` on every quad not
  in `reachable`.
- [ ] Task 5.2: In `vnavmesh/GroundGraph/QuadGraph.cs`, add `public int
  NearestQuad(Vector3 p, float halfExtentXZ=5, float halfExtentY=5, bool
  allowUnreachable=true)` that returns the quad id whose footprint contains
  `p.XZ` and whose surface Y is closest to `p.Y` within `halfExtentY`, or
  the nearest-by-center quad within `halfExtentXZ`; returns -1 if none. If
  `!allowUnreachable`, skip quads with `FLAG_UNREACHABLE`. This is the
  quad-graph `FindNearestMeshPoly`.
- [ ] Task 5.3: In `vnavmesh/NavmeshManager.cs`, add a `PruneGround(QuadGraph
  graph, IEnumerable<Vector3> seeds)` method that maps each seed to
  `graph.NearestQuad(seed)` (filtering -1), calls `graph.FloodReachable`,
  and `graph.ApplyReachable`. In `Reload`'s task (after the existing
  `Prune(points)` call at line 113), add `if (Navmesh?.Ground != null)
  PruneGround(Navmesh.Ground, points)`.
- [ ] Task 5.4: In `vnavmesh/Navmesh.cs`, add `SerializeGround`/
  `DeserializeGround` mirroring the `SerializeVolume`/`DeserializeVolume`
  pattern. `SerializeGround`: write `Quads.Count`, then for each quad write
  `MinX,MinY,MinZ,MaxX,MaxY,MaxZ` (6 floats) + `(byte)Area`. Write
  `Portals.Count`, then for each portal write `FromQuad,ToQuad` (2 ints),
  `SpanMin,SpanMax` (4 floats), `YFrom,YTo` (2 floats), `IsOffMesh` (1
  byte), `(byte)Area`. Write `Flags.Length` then `Flags` as int run-length
  (same RLE trick as volume: if value is 0 or 0x10, write a run count).
  `DeserializeGround` reverses. Use the existing Brotli `BinaryWriter`.
  Call `SerializeGround` after `SerializeVolume` in `Serialize`; call
  `DeserializeGround` after `DeserializeVolume` in `Deserialize` (null if
  the stream is exhausted → old cache without ground). Bump
  `Navmesh.Version` from 24 to 25 (invalidates all caches; one-time
  rebuild cost, acceptable).
- [ ] Task 5.5: In `vnavmesh/NavmeshBuilder.cs`, after `BuildAdjacency`
  (Task 2.4), size `Navmesh.Navmesh.Ground.Flags` to `Quads.Count` and
  initialize to 1 (reachable) before customization. Customization's
  `AddOffMesh` appends portals + adjacency; `FloodReachable` runs after.
- [ ] **Checkpoint: Verify Phase 5 complete.** `FloodReachable`/
  `ApplyReachable`/`NearestQuad`/`Flags` exist; `NavmeshManager.PruneGround`
  exists and is called; `SerializeGround`/`DeserializeGround` exist and are
  called; `Navmesh.Version == 25`. Build passes. First build after this
  phase regenerates all caches (version bump).

**Verification (Phase 5):**
- Build: 0 errors, 0 warnings.
- Static: `rg "FloodReachable|ApplyReachable|NearestQuad|SerializeGround"
  vnavmesh/` → ≥6. `rg "Version = 25" vnavmesh/Navmesh.cs` → 1.
  `rg "PruneGround" vnavmesh/` → 2.
- In-game (deferred): none (ground path not wired).

**Rollback:** `git revert <phase5-commit>`. Reverts `Navmesh.Version` to 24
(caches valid again), removes `SerializeGround`/`DeserializeGround`/
`PruneGround`/`FloodReachable`/`ApplyReachable`/`NearestQuad`/`Flags`.

### Phase 6 — Spline smoothing

Why this phase: smooth the string-pulled waypoint corridor into a curve so
movement doesn't snap between funnel corners. Applies to BOTH ground and
fly paths (it operates on the final `List<Waypoint>`). Orthogonal to
Phases 1–5; lands after pathfind exists but before movement controller
changes. Depends on nothing structurally.

- [ ] Task 6.1: Create `vnavmesh/Movement/SplineSmoothing.cs` with
  `namespace Navmesh.Movement` and `public static class SplineSmoothing`.
  Implement `public static List<Vector3> CatmullRom(List<Vector3> pts, int
  segmentsPerSpan=8)` that: if `pts.Count < 2` returns `pts` unchanged;
  prepends a duplicated first point and appends a duplicated last point
  (clamped endpoints); for each span `i` in `[1, pts.Count)`, emits
  `segmentsPerSpan` samples of the Catmull-Rom curve
  `0.5*((2P1)+(-P0+P2)*t+(2P0-5P1+4P2-P3)*t^2+(-P0+3P1-3P2+P3)*t^3)` for
  `t` in `(0,1]` (skip `t=0` except for the first span to avoid dupes);
  returns the sampled list including the original endpoints. Target ~80
  lines.
- [ ] Task 6.2: In `vnavmesh/Config.cs`, add `public bool SplineSmoothing =
  true;` and `public int SplineSegments = 8;` fields. Add an `ImGui`
  checkbox + `SliderInt` in `Config.Draw` (under a "Path smoothing" group).
  Wire `Modified` notification.
- [ ] Task 6.3: In `vnavmesh/Movement/FollowPath.cs:Move`, when
  `Service.Config.SplineSmoothing` and `waypoints.Count >= 2`, replace the
  incoming `waypoints` positions with `SplineSmoothing.CatmullRom(positions,
  Service.Config.SplineSegments)` before constructing `Waypoint`s. Preserve
  the `Waypoint.Type` of the nearest original waypoint for each spline
  sample (map by nearest original index) so `CheckCondition` (ClientPath/
  ClientPathEnd) still fires at the right samples. The last waypoint must
  stay exactly the requested destination (spline ends at the duplicated
  last point, so the final sample == last original point).
- [ ] **Checkpoint: Verify Phase 6 complete.** `SplineSmoothing.cs` exists
  with `CatmullRom`; `Config` has the two new fields + UI; `FollowPath.Move`
  applies the spline when enabled. Build passes.

**Verification (Phase 6):**
- Build: 0 errors, 0 warnings.
- Static: `rg "CatmullRom" vnavmesh/` → 2 (def + call).
  `rg "SplineSmoothing" vnavmesh/Config.cs` → 2 (field + UI).
- In-game (can verify now — fly paths already string-pulled): enable
  `ShowWaypoints`, fly a long path; confirm the waypoint line is a smooth
  curve, not a polyline. Toggle `SplineSmoothing` off and confirm it
  reverts to the polyline. Confirm the player still arrives at the exact
  destination.

**Rollback:** `git revert <phase6-commit>`. Removes `SplineSmoothing.cs`,
reverts `Config` + `FollowPath.Move`.

### Phase 7 — Velocity-aware movement controller

Why this phase: modulate movement magnitude (PD ease-in/out + corner speed)
instead of always driving at full speed. Depends on Phase 6 (spline-smoothed
paths have gentler corners, making turn-rate limiting meaningful). Modifies
`FollowPath.Update` + `OverrideMovement`.

- [ ] Task 7.1: In `vnavmesh/Movement/OverrideMovement.cs`, add fields
  `public float MaxTurnRateDeg = 720f;` (degrees/sec), `public float
  EaseDistance = 3f;` (distance over which to ease in/out), `private Vector3
  _prevPos;` `private float _prevForwardMag = 0f;`. Add `public float
  ComputeMagnitude(Vector3 desired, Vector3 playerPos, float dt, Vector3
  nextWaypointDir)` that returns a forward magnitude in `[0,1]`: ease-out
  near the waypoint (`clamp(dist / EaseDistance, 0, 1)`), corner-speed
  modulation (`clamp(1 - turnError, 0.25f, 1)` where `turnError` = angle
  between current heading and `nextWaypointDir`), PD term
  (`kp * dist + kd * (dist - prevDist)/dt` clamped). Document the exact
  formula inline (no comments — formula is the code).
- [ ] Task 7.2: In `OverrideMovement.cs:RMIWalkDetour`, replace the direct
  `*sumLeft = dir.X; *sumForward = dir.Y;` with a magnitude-scaled version:
  compute `mag = ComputeMagnitude(DesiredPosition, playerPos, dt,
  DesiredPosition - playerPos)`; scale `*sumLeft = dir.X * mag; *sumForward
  = dir.Y * mag;`. Pass `dt` from the framework tick (add a `public float
  LastDt` set by `FollowPath.Update` each frame, or compute from a stored
  `DateTime`). Do the same in `RMIFlyDetour` for `result->Forward`/
  `result->Left` (keep `result->Up` full magnitude for takeoff/climb).
- [ ] Task 7.3: In `OverrideMovement.cs:DirectionToDestination`, add
  turn-rate limiting: compute the desired azimuth delta from the player's
  current heading, clamp it to `±MaxTurnRateDeg * dt` per frame, and use the
  clamped azimuth for the returned `h` angle. This prevents the
  instantaneous 180-degree snap when a new waypoint is behind the player.
  Store `_lastDesiredAzimuth` and clamp the delta.
- [ ] Task 7.4: In `vnavmesh/Movement/FollowPath.cs:Update`, before setting
  `_movement.DesiredPosition`, compute `nextWaypointDir =
  Waypoints[min(1, Waypoints.Count-1)].Position - Waypoints[0].Position`
  and pass it to `OverrideMovement` (store on a new `public Vector3
  NextSegmentDir` field read by `ComputeMagnitude`). Set
  `_movement.LastDt = fwk.UpdateDelta.Milliseconds / 1000f`.
- [ ] Task 7.5: In `vnavmesh/Config.cs`, add `public float MoveMaxTurnRate =
  720f;` `public float MoveEaseDistance = 3f;` and wire them into
  `OverrideMovement` via `NavmeshManager`/`Plugin` initialization or a
  config-changed callback. Add `ImGui` sliders in `Config.Draw` under a
  "Movement" group.
- [ ] **Checkpoint: Verify Phase 7 complete.** `ComputeMagnitude` exists and
  is called in both detours; turn-rate limiting clamps the azimuth delta in
  `DirectionToDestination`; `FollowPath.Update` feeds `NextSegmentDir` +
  `LastDt`; `Config` has the two new tunables. Build passes.

**Verification (Phase 7):**
- Build: 0 errors, 0 warnings.
- Static: `rg "ComputeMagnitude" vnavmesh/Movement/` → 2.
  `rg "MaxTurnRateDeg|LastDt|NextSegmentDir" vnavmesh/Movement/` → matches.
  `rg "MoveMaxTurnRate|MoveEaseDistance" vnavmesh/Config.cs` → 2.
- In-game: walk a ground path with a sharp 90-degree corner; confirm the
  player slows into the corner rather than jerking. Walk toward a waypoint
  and stop short; confirm the player eases to a stop (magnitude → 0) rather
  than overshooting then snapping back. Fly a path; confirm vertical
  (`result->Up`) stays full-strength for climb.

**Rollback:** `git revert <phase7-commit>`. Reverts `OverrideMovement` to
full-magnitude binary movement, removes `ComputeMagnitude`/turn-rate/
`LastDt`/`NextSegmentDir`, reverts `Config` fields.

### Phase 8 — Rewrite NavmeshManager + NavmeshQuery for ground → quad graph

Why this phase: switch ground pathfinding from `DtNavMesh` to `QuadGraph`.
This is the pivot: after it, ground paths come from the quad A\*. The
`DtNavMesh` is still built (deleted in Phase 10) but unused for
`PathfindMesh`. All `NavmeshQuery` public method names/signatures stay
identical. Depends on Phases 1–7.

- [ ] Task 8.1: In `vnavmesh/NavmeshQuery.cs`, add `private QuadGraph?
  _ground;` and `private QuadPathfind? _groundPath;`. In the constructor,
  if `navmesh.Ground != null`, set `_ground = navmesh.Ground` and
  `_groundPath = new(navmesh.Ground)`. Keep `MeshQuery` (DtNavMesh) for
  now — it is still used by `BuildBitmap` and the Debug viz until Phase 10.
- [ ] Task 8.2: Rewrite `NavmeshQuery.PathfindMesh` (keep signature):
  `List<Waypoint> PathfindMesh(Vector3 from, Vector3 to, bool useRaycast,
  bool useStringPulling, float range, CancellationToken cancel)`. If
  `_ground == null`, fall back to the old DtNavMesh path (so non-flyable
  zones with no volume/ground still work until Phase 10 guarantees a
  ground graph). Otherwise: call `_ground.Pathfind(from, to, useRaycast,
  useStringPulling, range, cancel)` returning `List<Vector3>`; map each
  position to `new Waypoint(p, GetAreaIdForPos(p))` (nearest quad's Area, or
  Default); append `new Waypoint(to)`. The `GoalRadiusHeuristic` (range)
  behavior moves into `QuadGraph.Pathfind` (already handled in Task 3.7).
- [ ] Task 8.3: Rewrite `NavmeshQuery.FindNearestMeshPoly` (keep signature
  `long`): if `_ground != null`, return `(long)_ground.NearestQuad(p,
  halfExtentXZ, halfExtentY, allowUnreachable)` (or 0 if -1). Else fall
  back to the DtNavMesh `MeshQuery.FindNearestPoly`. The return type stays
  `long`; quad ids cast to `long`.
- [ ] Task 8.4: Rewrite `FindIntersectingMeshPolys` (keep signature
  `List<long>`): if `_ground != null`, return all quad ids whose footprint
  intersects the `halfExtent` XZ box and whose Y is within `halfExtent.Y` of
  `p.Y`, as `List<long>` (cast). Else DtNavMesh fallback.
- [ ] Task 8.5: Rewrite `FindNearestPointOnMeshPoly` (keep signature
  `Vector3?`): if `_ground != null` and `poly` (now a quad id) is in range,
  return the point on that quad's surface (clamp `p.XZ` into the quad
  footprint, set Y = quad surface Y). Else DtNavMesh fallback.
- [ ] Task 8.6: Rewrite `FindNearestPointOnMesh` (keep signature): delegate
  to `FindNearestMeshPoly` then `FindNearestPointOnMeshPoly` (both now
  quad-aware). Signature unchanged.
- [ ] Task 8.7: Rewrite `FindPointOnFloor` (keep signature `Vector3?`): if
  `_ground != null`, iterate quads whose XZ footprint contains `p.XZ`
  within `halfExtentXZ` and whose surface Y ≤ `p.Y`; return the highest
  such surface point. Else DtNavMesh fallback.
- [ ] Task 8.8: Rewrite `FindReachableMeshPolys` (keep signature
  `HashSet<long>`): if `_ground != null`, return
  `_ground.FloodReachable(starting.Select(s => (int)s).Where(s => s>=0))`
  cast to `long`. Else DtNavMesh fallback (the old `polyLinks` walk).
- [ ] Task 8.9: Rewrite `GetAreaId` (keep signature `Navmesh.AreaId`): if
  the `refs` is a quad id in `_ground` range, return
  `_ground.Quads[(int)refs].Area`. Else DtNavMesh
  `GetPolyArea` fallback.
- [ ] Task 8.10: Remove the `IntersectQuery`/`GoalRadiusHeuristic`/
  `TeleportAwareFilter`/`FloodFillAwareFilter` inner classes from
  `NavmeshQuery.cs` only if the DtNavMesh fallback paths no longer reference
  them. Keep them if any fallback still uses them (Phase 10 removes them
  when the fallback is deleted). Do not delete yet unless unused.
- [ ] Task 8.11: In `vnavmesh/NavmeshManager.cs:QueryPath`, the dispatch
  `flying ? PathfindVolume : PathfindMesh` stays unchanged — `PathfindMesh`
  now routes to the quad graph internally. No change to `QueryPath`
  signature or the IPC. `Prune` (DtNavMesh) stays alongside the new
  `PruneGround` from Phase 5 until Phase 10.
- [ ] **Checkpoint: Verify Phase 8 complete.** Every `NavmeshQuery` public
  method listed in Tasks 8.2–8.9 is quad-aware with a DtNavMesh fallback.
  Signatures unchanged. `PathfindMesh` calls `_ground.Pathfind` when a
  ground graph exists. Build passes.

**Verification (Phase 8):**
- Build: 0 errors, 0 warnings.
- Static: `rg "_ground\.Pathfind|_ground\.NearestQuad|_ground\.Flood
  Reachable" vnavmesh/NavmeshQuery.cs` → ≥3. `rg "public List<Waypoint>
  PathfindMesh" vnavmesh/NavmeshQuery.cs` → 1 (signature intact).
  `rg "public long FindNearestMeshPoly" vnavmesh/NavmeshQuery.cs` → 1.
  Confirm no IPC method signature changed: `rg "RegisterFunc|RegisterAction"
  vnavmesh/IPCProvider.cs` output unchanged vs pre-Phase-8.
- In-game (now meaningful): in a zone with a built ground graph (New
  Gridania), call `Nav.Pathfind` with `fly=false` for a known ground route;
  confirm a path is returned with fewer/smoother waypoints than the
  DotRecast path (funnel string-pull). Confirm `Query.Mesh.NearestPoint`,
  `Query.Mesh.PointOnFloor` return sensible positions. See Phase 11 for
  the full checklist.

**Rollback:** `git revert <phase8-commit>`. `PathfindMesh` and friends
revert to pure DtNavMesh. Quad graph is built but unused (Phases 1–7
output still compiles).

### Phase 9 — Wire ground to quad graph + IPC verification

Why this phase: explicit verification that every IPC method still works end
to end with the quad graph as the ground data source. No new code unless a
gap is found; this phase is verification + fixes. Depends on Phase 8.

- [ ] Task 9.1: Run the full build. Confirm 0 errors, 0 warnings.
- [ ] Task 9.2: Static sweep — confirm the IPC surface is byte-identical.
  Diff `IPCProvider.cs` against the Phase-0 baseline: `git diff <phase0-
  commit> -- vnavmesh/IPCProvider.cs` must show NO changes. If any IPC
  signature drifted, fix `NavmeshQuery` (the IPC delegates to it), never
  `IPCProvider`.
- [ ] Task 9.3: Enumerate every IPC method and verify its delegate target
  compiles + runs: `Nav.IsReady`, `Nav.BuildProgress`, `Nav.Reload`,
  `Nav.Rebuild`, `Nav.Pathfind`, `Nav.PathfindWithTolerance`,
  `Nav.PathfindCancelable`, `Nav.PathfindCancelAll`, `Nav.PathfindInProgress`,
  `Nav.PathfindNumQueued`, `Query.Mesh.NearestPoint`,
  `Query.Mesh.NearestPointReachable`, `Query.Mesh.PointOnFloor`,
  `Query.Mesh.FlagToPoint`, `Path.MoveTo`, `Path.Stop`, `Path.IsRunning`,
  `Path.NumWaypoints`, `Path.ListWaypoints`, `Path.GetMovementAllowed`,
  `Path.SetMovementAllowed`, `Path.GetAlignCamera`, `Path.SetAlignCamera`,
  `Path.GetTolerance`, `Path.SetTolerance`,
  `SimpleMove.PathfindAndMoveTo`, `SimpleMove.PathfindAndMoveCloseTo`,
  `SimpleMove.PathfindInProgress`, `Window.IsOpen`, `Window.SetOpen`,
  `DTR.IsShown`, `DTR.SetShown`. For each, confirm the delegate lambda in
  `IPCProvider.cs` still type-checks against the (unchanged) `NavmeshQuery`
  / `FollowPath` / `NavmeshManager` methods.
- [ ] Task 9.4: In-game smoke test (record results in `SESSION_NOTES.md`):
  zone-load a flyable zone (e.g. New Gridania, 132 — has a customization
  with off-mesh links ported in Phase 4). For each IPC method above, invoke
  it via a consumer or the Dalamud console and confirm a sane result. Pay
  attention to: `Nav.Pathfind` fly=false (ground quad path),
  `Query.Mesh.NearestPoint`, `Query.Mesh.PointOnFloor`,
  `Query.Mesh.FlagToPoint`, `SimpleMove.PathfindAndMoveTo` fly=false (full
  move-to loop), `Path.MoveTo`/`Path.Stop`/`Path.IsRunning`/
  `Path.NumWaypoints`/`Path.ListWaypoints`.
- [ ] Task 9.5: Fix any IPC regression found in 9.3/9.4 by adjusting
  `NavmeshQuery` internals only. If a method cannot be made to work on the
  quad graph without a signature change, STOP and surface it as an Open
  Question — do not change the IPC signature.
- [ ] **Checkpoint: Verify Phase 9 complete.** `IPCProvider.cs` is
  byte-identical to Phase 0. Every IPC method invoked in-game returns a
  sane result. No signature drift. Build passes.

**Verification (Phase 9):**
- Build: 0 errors, 0 warnings.
- Static: `git diff <phase0-commit> -- vnavmesh/IPCProvider.cs` → empty.
- In-game: all 31 IPC methods listed in 9.3 return sane values in a flyable
  zone with a built ground graph. Results recorded in `SESSION_NOTES.md`.

**Rollback:** `git revert <phase9-commit>` only if a regression was fixed;
otherwise this phase may be a no-op commit (or folded into Phase 8). If a
regression is unfixable without an IPC change, revert Phase 8 too and
surface the Open Question.

### Phase 10 — Delete DotRecast + all dependent code

Why this phase last: only after ground pathfinding on the quad graph is
verified in-game (Phases 8–9). Deletes ~3500 lines, the submodule, all
csproj refs, and every DotRecast-coupled file/branch. Depends on Phases
1–9. After this phase the build no longer references DotRecast at all and
the 3 pre-existing warnings are gone.

- [ ] Task 10.1: In `vnavmesh/vnavmesh.csproj`, remove the four
  `<ProjectReference Include="..\DotRecast\..." />` lines (lines 14–17).
  Remove `using DotRecast...` from every file as they become unused
  (handled per-file below).
- [ ] Task 10.2: Rewrite `vnavmesh/NavmeshBuilder.cs` to drop the entire
  Recast pipeline. Remove `RcHeightfield`/`RcCompactHeightfield`/
  `RcContourSet`/`RcPolyMesh`/`RcPolyMeshDetail`/`RcContext`/`RcRegions`/
  `RcContours`/`RcMeshs`/`RcMeshDetails`/`RcFilters`/`RcAreas`/
  `RcCompacts`/`JumpLinkBuilder`/`JumpLink`/`JumpLinkBuilderConfig`/
  `JumpLinkType`/`DtNavMeshBuilder`/`DtNavMeshCreateParams`/`DtNavMeshParams`/
  `RcBuilderResult`/`RcPartition` usage. The new `BuildTiles` only
  rasterizes into the `Voxelizer` (keep the `NavmeshRasterizer` voxelizer-
  feeding path from Task 10.6), builds `VoxelMap` per tile, merges, then
  builds `QuadGraph` via `QuadMesher.GreedyMesh` + `BuildAdjacency`. Remove
  `NumTilesX`/`NumTilesZ`/`_walkableClimbVoxels`/`_walkableHeightVoxels`/
  `_walkableRadiusVoxels`/`_walkableNormalThreshold`/`_borderSizeVoxels`/
  `_borderSizeWorld`/`_tileSizeXVoxels`/`_tileSizeZVoxels` (Recast-specific)
  but keep `_voxelizerNumX/Y/Z` and the `Settings.NumTiles`/`CellSize`/
  `CellHeight`/`AgentMaxClimb`/`AgentMaxSlopeDeg`/`Filtering` values that the
  `Voxelizer` + `QuadMesher` consume. Remove the `Intermediates` record and
  the `List<RcBuilderResult>` return. `BuildTiles` returns `void` (or the
  task list); the `onTileFinished` progress callback stays.
- [ ] Task 10.3: Rewrite `vnavmesh/NavmeshRasterizer.cs` to keep only the
  `Voxelizer`-feeding path. Remove `RcHeightfield _heightfield`,
  `IntersectionSet _iset`, all `RcHeightfield`/`RcSpan`/span-pool direct
  access, `FillInterior`, `RasterizeOld`, the `SplitConvexPoly`/
  `AxisMinMax`/`TransformVertices` helpers that were only for the
  heightfield. Keep the triangle-bbox clipping that feeds the voxelizer:
  the cell-iteration loop (z→x→y) computing `y0,y1` per triangle cell, then
  `_voxelizer.AddSpan(x, z, y0-shift, y1-shift)` when `includeInVolume &&
  _voxelizer != null`. The `realSolid`/`unwalkable`/`areaId` computation
  stays only insofar as it decides whether to mark the voxelizer span; the
  heightfield span bookkeeping (`cellHead`, `spanPool`, `prevSpanIndex`) is
  gone. This is the single hardest deletion; expect the most edits here.
  Add `public NavmeshRasterizer(Voxelizer voxelizer, float cellSize, float
  cellHeight, float walkableNormalThreshold, int walkableMaxClimb, int
  minGap, bool fillInteriors)` (drop the `RcHeightfield` + `RcContext`
  params). `Rasterize` signature stays `(SceneExtractor geom,
  SceneExtractor.MeshType types, bool perMeshInteriors, bool
  solidBelowNonManifold)`.
- [ ] Task 10.4: Rewrite `vnavmesh/Navmesh.cs`: change the record to
  `record class Navmesh(int CustomizationVersion, QuadGraph? Ground,
  VoxelMap? Volume)` (drop `DtNavMesh Mesh`). Remove
  `SerializeMesh`/`DeserializeMesh`/`SerializeMeshParams`/
  `DeserializeMeshParams`/`SerializeMeshTile`/`DeserializeMeshTile` and all
  `DtNavMesh`/`DtMeshData`/`DtMeshTile`/`DtPoly`/`DtPolyDetail`/`DtBVNode`/
  `DtOffMeshConnection`/`DtNavMeshParams` usage. Keep
  `SerializeVolume`/`DeserializeVolume` + the new
  `SerializeGround`/`DeserializeGround` (Phase 5). `Serialize` writes
  Magic/Version/CustomizationVersion, then Brotli: Ground, then Volume.
  `Deserialize` reverses. Bump `Version` from 25 to 26 (final; the DtNavMesh
  block is gone so the format changes again). Remove the `Links` field
  (was DtNavMesh-visualization only). Keep `FLAG_UNREACHABLE` const (move
  the canonical definition here; `QuadGraph.FLAG_UNREACHABLE` references it).
  Keep `AreaId` enum. Update all `new Navmesh(...)` call sites (in
  `NavmeshBuilder`, `Navmesh.Deserialize`) and `Navmesh.Mesh`/`.Volume`
  references (`NavmeshManager.BuildBitmap`, `NavmeshManager.Prune`,
  `DebugNavmeshManager`, `DebugNavmeshCustom`).
- [ ] Task 10.5: Rewrite `vnavmesh/NavmeshCustomization.cs`: remove
  `LinkPoints`/`InsertPointPoly` (DtNavMesh link manipulation). Remove
  `CustomizeSettings(DtNavMeshCreateParams)` virtual. Remove
  `CustomizeMesh(Navmesh, List<uint>)` virtual (the DtNavMesh post-build
  hook is gone; its only purpose was `LinkPoints`). Keep `CustomizeScene`
  and `CustomizeGround` (Phase 4). Keep `SceneExtensions` (DotRecast-free).
  Remove `CreateParamsExtensions.AddOffMeshConnection` (Phase 4's
  `QuadGraph.AddOffMesh` replaces it). Update the two `RcPartition.MONOTONE`
  customizations to drop the `Settings.Partitioning` line (the field is
  removed in Task 10.8).
- [ ] Task 10.6: Rewrite `vnavmesh/NavmeshManager.cs:BuildBitmap` to use
  the quad graph: `startPoly = Query.FindNearestMeshPoly(startingPos)` (now
  a quad id), `reachablePolys = Query.FindReachableMeshPolys(startPoly)`
  (now quad ids). Replace `Navmesh.Mesh.GetTileAndPolyByRefUnsafe(p, ...)` +
  `poly.vertCount`/`poly.verts` + `NavmeshBitmap.GetVertex` with reading
  `_ground.Quads[(int)p]`'s `MinX/MinZ/MaxX/MaxZ`. Rewrite
  `NavmeshBitmap.RasterizePolygon` to take a `Quad` and fill the bitmap
  rows `[MinZ..MaxZ]` × `[MinX..MaxX]` (a quad is an axis-aligned rectangle —
  trivial fill, no polygon rasterization needed). `NavmeshManager.Prune`
  (DtNavMesh-based) is removed; `PruneGround` (Phase 5) is the only prune.
- [ ] Task 10.7: Rewrite `vnavmesh/NavmeshQuery.cs`: remove all DtNavMesh
  fallbacks (Tasks 8.2–8.9 fallbacks). Remove `MeshQuery`, `IntersectQuery`,
  `GoalRadiusHeuristic`, `TeleportAwareFilter`, `FloodFillAwareFilter`,
  `_filter`, `_pathFilter`, `_reachableFilter`, `_lastPath`,
  `LastPath`. `PathfindMesh` is pure quad-graph. All `FindNearestMeshPoly`
  etc. are pure quad-graph (no DtNavMesh). Remove `using DotRecast...`.
  Keep `VolumeQuery` + `PathfindVolume` (voxel) untouched.
- [ ] Task 10.8: Rewrite `vnavmesh/NavmeshSettings.cs`: remove
  `RcPartition Partitioning` field + `DrawConfigPartitioningCombo`/
  `DrawConfigPartitioningEnum`. Remove `PolyMaxEdgeLen`,
  `PolyMaxSimplificationError`, `PolyMaxVerts`, `DetailSampleDist`,
  `DetailMaxSampleError`, `RegionMinSize`, `RegionMergeSize`,
  `GenerateEdgeClimbLinks`, `GenerateEdgeJumpLinks`, `GroundTolerance`,
  `ClimbDown*`, `EdgeJump*` (all Recast/jump-link-only). Keep `CellSize`,
  `CellHeight`, `AgentHeight`, `AgentRadius`, `AgentMaxClimb`,
  `AgentMaxSlopeDeg`, `Filtering`, `NumTiles`. Update `NavmeshBuilder`'s
  derived-param computation to drop the removed fields.
- [ ] Task 10.9: Rewrite `vnavmesh/Extensions.cs`: remove
  `SystemToRecast`/`RecastToSystem` (no more `RcVec3f`). Keep
  `Floor`/`Ceiling` (used by `VoxelSearch`). Remove `using DotRecast...`.
- [ ] Task 10.10: Delete the Recast-only `Debug/` files:
  `DebugCompactHeightfield.cs`, `DebugContourSet.cs`,
  `DebugDetourNavmesh.cs`, `DebugPolyMesh.cs`, `DebugPolyMeshDetail.cs`,
  `DebugSolidHeightfield.cs`, `DebugRecast.cs`, `DebugExtractedCollision.cs`
  (uses `RcHeightfield`), `DebugNavmeshCustom.cs` (uses `DtNavMesh` +
  `RcBuilderResult` — delete entirely; its customization-preview function is
  replaced by `DebugQuadGraph`'s portal/link visualization; remove its tab
  from `MainWindow.cs`), `DebugLinks.cs` (consumes `Navmesh.Links` which is
  removed in Task 10.4). Create `vnavmesh/Debug/DebugQuadGraph.cs` that draws
  the `QuadGraph` quads (XZ rectangles at surface Y) + portals + reachable/
  unreachable coloring, mirroring `DebugDetourNavmesh`'s draw structure but
  over `Quad` data. Rewrite `DebugNavmeshManager.cs` to use
  `DebugQuadGraph` instead of `DebugDetourNavmesh`, drop all
  `Navmesh.Mesh`/`Query.MeshQuery`/`Query.LastPath` references, keep the
  voxel-map + floor-point debug. Update `MainWindow.cs` field/init/dispose
  for the renamed/added debug classes. Explicitly kept (DotRecast-free,
  unaffected): `DebugDrawer.cs`, `DebugExportObj.cs`,
  `DebugGameCollision.cs`, `DebugLayout.cs`, `DebugVoxelMap.cs`.
- [ ] Task 10.11: Deinit the DotRecast submodule: `git submodule deinit -f
  DotRecast`, `git rm DotRecast`, remove the `[submodule "DotRecast"]`
  block from `.gitmodules`, `git rm --cached DotRecast`. Commit the
  submodule removal in this phase's commit (or a separate immediately-
  following commit; user preference — ask before committing the submodule
  removal specifically).
- [ ] Task 10.12: Remove `using DotRecast...` from `Z0132NewGridania.cs`,
  `Z1252OccultCrescentSouthHorn.cs`, `NavmeshBitmap.cs`,
  `NavmeshRasterizer.cs`, `NavmeshBuilder.cs`, `Navmesh.cs`,
  `NavmeshCustomization.cs`, `NavmeshQuery.cs`, `NavmeshSettings.cs`,
  `Extensions.cs`. Confirm `rg "DotRecast" vnavmesh/` → 0.
- [ ] **Checkpoint: Verify Phase 10 complete.** `rg "DotRecast" vnavmesh/`
  → 0. `rg "DtNavMesh|RcHeightfield|RcContext|RcPoly|DtPoly|DtMesh|
  JumpLink|RcConstants|RcSpan|RcBuilder|RcVec|RcPartition|RcAreas|RcRegions|
  RcContours|RcMeshs|RcMeshDetails|RcFilters|RcCompacts|DtOffMeshConnection|
  DtBVNode|DtPolyDetail|DtNavMeshQuery|DtQueryDefaultFilter|DtQueryHeuristic|
  DtFindPath|DtStraightPath|SystemToRecast|RecastToSystem" vnavmesh/` → 0.
  The `DotRecast/` directory is gone. `vnavmesh.csproj` has no
  `<ProjectReference Include="..\DotRecast...`. Build: 0 errors, 0 warnings
  (the 3 pre-existing DotRecast warnings are gone). No stubs; every removed
  API has a quad-graph replacement wired.

**Verification (Phase 10):**
- Build: 0 errors, 0 warnings (first time with zero warnings — DotRecast is
  gone).
- Static: the two `rg` commands above return 0.
  `rg "using DotRecast" vnavmesh/` → 0. `ls DotRecast` → no such directory.
  `git submodule status` → no DotRecast entry.
- In-game (deferred to Phase 11 for the full pass; quick smoke here): load
  a zone, confirm the ground graph builds (DTR / log), call `Nav.Pathfind`
  fly=false, confirm a path returns.

**Rollback:** `git revert <phase10-commit>` (this is a large revert; it
restores DotRecast + all deleted files + the csproj refs + the submodule).
If the submodule was removed in a separate commit, revert that commit too.
Confirm `git submodule update --init --recursive` rehydrates DotRecast after
revert.

### Phase 11 — In-game verification + tuning + final audit

Why this phase: the codebase compiles DotRecast-free and the quad pathfinder
is wired, but most behavior can only be verified in-game. This phase is the
final pass + tuning. Depends on Phase 10.

- [ ] Task 11.1: Full IPC in-game verification (extend the Phase 9 list to
  post-Phase-10 state): confirm all 31 IPC methods work in (a) a flyable
  outdoor zone (New Gridania 132), (b) a non-flyable dungeon with a
  customization (Lost City of Amdapor 363), (c) a large zone with many
  off-mesh links (Occult Crescent South Horn 1252). Record pass/fail per
  method per zone in `SESSION_NOTES.md`.
- [ ] Task 11.2: Ground path quality: in each of the three zones, pathfind
  between distant aetherytes/landmarks; confirm the quad path reaches the
  destination, the funnel string-pull produces few waypoints (compare
  count vs the old DotRecast path — expect similar or fewer), and the
  player follows it without clipping through walls or getting stuck. Note
  any quad-graph artifacts (over-large quads bridging a gap, missing climb
  adjacency at a step) and tune `AgentMaxClimb` / `NumTiles[2]` if needed.
- [ ] Task 11.3: Flying path regression: re-run the Phase 0+1 flying
  verification (long fly path, identical-results determinism, string-pull
  waypoint reduction). Confirm no regression from Phases 6–7 (spline +
  velocity-aware movement apply to fly too).
- [ ] Task 11.4: Movement quality: walk a path with sharp corners;
  confirm corner-speed modulation (Phase 7) slows into turns. Walk to a
  nearby waypoint; confirm ease-to-stop. Fly a path with a climb; confirm
  vertical stays full-strength. Tune `MoveMaxTurnRate` / `MoveEaseDistance`
  / `SplineSegments` if movement feels off.
- [ ] Task 11.5: Build performance: time a full zone build (cold cache) in
  a large zone; confirm it is materially faster than the Phase-0 baseline
  (the Recast pipeline is gone; only voxelizer + quad mesher remain).
  Record before/after in `SESSION_NOTES.md`.
- [ ] Task 11.6: Cache format: confirm a cache built in this phase loads
  on a fresh restart (serialize → deserialize round-trip). Confirm
  `Navmesh.Version == 26`.
- [ ] Task 11.7: Debug visualization: confirm `DebugQuadGraph` renders the
  quads + portals + reachability coloring in-game. Confirm the old Recast
  debug tabs are gone from the window.
- [ ] Task 11.8: Bump `vnavmesh/vnavmesh.csproj` `<Version>` (per
  AGENTS.md publish rule) and `vnavmesh.json` description noting the
  DotRecast removal + quad-graph ground pathfinder.
- [ ] **Checkpoint: Verify Phase 11 complete.** All 31 IPC methods pass in
  all three test zones. Ground + flying paths reach destinations without
  clipping/stuck. Build is faster than baseline. Cache round-trips. Debug
  viz works. Version bumped.
- [ ] **Final Audit.** Re-read this entire plan. For each task in Phases
  1–11, verify the implementation exists in the codebase (file created /
  symbol present / DotRecast gone). List any gaps. All gaps must be
  resolved before reporting completion. Confirm: `rg "DotRecast" vnavmesh/`
  → 0; `ls DotRecast` → gone; build 0/0; all checkpoint tasks signed off.

**Verification (Phase 11):** the in-game checklist above is the gate.
Record results in `SESSION_NOTES.md` under a "Phase 2 (new) — Quad-graph
ground pathfinder + DotRecast removal" section.

**Rollback:** per-phase reverts are available for every earlier phase; this
phase is verification + tuning, so its commit (if any) is small (version
bump + config tunables). `git revert <phase11-commit>` if a tunable change
regresses.

## Edge Cases & Risks

- **Quad mesher misses diagonal walkable connections.** Greedy-meshing axis-
  aligned voxels cannot produce diagonal quads. If two walkable patches meet
  only at a diagonal corner, the mesher makes two quads with no shared edge
  and A\* cannot cross. Mitigation: addressed by Task 2.2's adjacency which
  connects quads by overlapping span; a corner-touch is an overlap of zero
  length, so it is NOT connected — correct (a player cannot walk through a
  diagonal corner gap). If in-game testing shows a needed diagonal is
  blocked, the fix is voxel resolution (`NumTiles[2]`), not the mesher.
  Open Question if it persists.
- **Overlapping floors (multi-level).** Quads at different Y over the same
  XZ are separate; adjacency connects them only if the climb ≤
  `AgentMaxClimb`. A second floor directly above a first is unreachable by
  climb (correct — you take stairs/aetheryte). If a staircase's step rise
  exceeds `AgentMaxClimb`, the quad graph drops the connection. Mitigation:
  `AgentMaxClimb` default 0.5 yalms covers FFXIV steps; if a specific
  staircase (e.g. the Occult Crescent stair tweaked in `Z1252`) is too
  steep, the existing `CustomizeScene` vertex tweak + an off-mesh
  `AddOffMesh` covers it.
- **Off-mesh link endpoint outside any quad.** `QuadGraph.AddOffMesh` (Task
  4.2) returns -1 if `a` or `b` has no nearest quad. The customization then
  silently drops the link. Mitigation: `AddOffMesh` logs a warning via
  `Service.Log.Warning` with the coords so the customization author can
  adjust. Addressed by Task 4.2.
- **Funnel portal at a Y step.** A portal between two quads at different Y
  has two endpoints at different heights. The standard funnel assumes
  coplanar portals; a Y step makes the portal a 3D segment. The Simple
  Stupid Funnel Algorithm operates on 2D portals; we project to XZ for the
  funnel and keep the Y from the `toPos`-side quad. Mitigation: Task 3.6
  projects portals to XZ; the Y of each output waypoint is the surface Y of
  the quad the apex is on. Acceptable (steps are ≤ climb, small Y deltas).
  Open Question if large Y steps produce visible jumps.
- **Cache invalidation storm.** Phases 5 and 10 each bump `Navmesh.Version`
  (24→25→26), invalidating all caches twice. Every installed user rebuilds
  every zone twice across the two phase-upgrades. Mitigation: these are
  pre-release dev phases; the version bumps are one-time. Addressed by
  Tasks 5.4 and 10.4.
- **`NavmeshRasterizer` rewrite (Task 10.3) is the riskiest deletion.** It
  directly walks `RcHeightfield` span linked lists and the
  `IntersectionSet`. Mis-porting the voxelizer-feeding path would corrupt
  the `VoxelMap` and break BOTH ground and flying. Mitigation: keep the
  voxelizer-feeding block (`AddSpan`'s `if (includeInVolume && _voxelizer
  != null)` tail) byte-for-byte; only delete the heightfield bookkeeping
  around it. Run the Phase 11 flying regression (Task 11.3) immediately
  after this task to catch a voxelizer regression before any other
  Phase-10 task.
- **IPC signature drift.** Any change to `NavmeshQuery` public method
  signatures breaks drop-in compatibility. Mitigation: Phase 9 static
  diff of `IPCProvider.cs`; Phase 8 keeps signatures; Phase 10 removes
  fallbacks but not signatures. Risk is low because the `long` handle
  type absorbs the quad-id change.
- **`DebugNavmeshManager` / `DebugNavmeshCustom` heavy DtNavMesh use.**
  These break the build in Phase 10 if not rewritten. Mitigation: Task
  10.10 rewrites them + adds `DebugQuadGraph`. If a debug feature cannot be
  ported in time, stub it to a no-op draw rather than leaving a
  DotRecast reference (but prefer full port).
- **Submodule removal leaves a stale `.git/modules` entry.** Mitigation:
  Task 10.11 runs `git submodule deinit -f DotRecast` which cleans
  `.git/modules/DotRecast`. Verify with `git submodule status` after.
- **`NavmeshBuilder.BuildTiles` return type change (Task 10.2).**
  `BuildTiles` currently returns `List<RcBuilderResult>` consumed by nobody
  outside the builder (the caller `NavmeshManager.BuildNavmesh` ignores the
  return). Changing to `void` is safe; confirmed by `rg
  "BuildTiles" vnavmesh/`.

## Acceptance Criteria

- `DALAMUD_HOME=~/.cache/dalamud-dev DOTNET_ROOT=~/.dotnet ~/.dotnet/dotnet
  build vnavmesh/vnavmesh.csproj -c Release -p:Platform=x64` → 0 errors, 0
  warnings.
- `rg "DotRecast" vnavmesh/` → 0 matches.
- `ls DotRecast` → no such directory; `git submodule status` → empty.
- `rg "using DotRecast" vnavmesh/` → 0.
- `git diff <phase0-commit> -- vnavmesh/IPCProvider.cs` → empty (IPC surface
  byte-identical).
- `Nav.Pathfind` fly=false returns a valid `List<Vector3>` in New Gridania,
  Lost City of Amdapor, and Occult Crescent South Horn (in-game, Phase 11).
- `Nav.Pathfind` fly=true still works (Phase 0+1 regression check, Phase 11).
- All 31 IPC methods return sane values in-game (Phase 11 Task 11.1).
- `Navmesh.Version == 26`; a built cache round-trips through serialize →
  deserialize (Phase 11 Task 11.6).
- Full zone build is materially faster than the Phase-0 baseline (Phase 11
  Task 11.5).
- `DebugQuadGraph` renders quads + portals + reachability in-game (Phase 11
  Task 11.7).

## Open Questions

- **Diagonal-corner walkable gaps:** does any FFXIV zone require walking
  through a diagonal-corner gap that axis-aligned quads cannot represent?
  If Phase 11 testing finds one, the options are (a) raise `NumTiles[2]`
  resolution, (b) add a corner-touch adjacency (risky — lets the player
  clip corners), (c) add an off-mesh `AddOffMesh` in the zone's
  customization. Decide based on the specific gap found.
- **Funnel Y-step visibility:** do large Y steps (≥ AgentMaxClimb, via
  off-mesh) produce visible waypoint jumps in the funnel output? If so, the
  fix is to insert an intermediate waypoint at the portal midpoint with the
  lower quad's Y, then the upper quad's Y. Decide after Phase 11 ground
  path visual inspection.
- **`Nav.BuildBitmap` consumers:** `Nav.BuildBitmap` /
  `Nav.BuildBitmapBounded` are IPC methods that produce a walkable-area
  bitmap. Are any consumers (SealHunter, etc.) relying on the bitmap being
  polygon-rasterized vs quad-rasterized? A quad is an axis-aligned
  rectangle so the bitmap is identical for walkable area; the only
  difference is unreachable quads are now flagged on the quad graph not
  the DtNavMesh. Confirm with a consumer smoke test in Phase 11; if a
  consumer breaks, the fix is in `BuildBitmap`'s reachable-set source
  (already switched to `FindReachableMeshPolys` = quad flood in Phase 10),
  not the IPC signature.
- **Submodule removal commit boundary:** should the `git rm DotRecast` +
  `.gitmodules` edit be its own commit (cleaner revert) or folded into the
  Phase 10 commit? Ask the user before executing Task 10.11.
