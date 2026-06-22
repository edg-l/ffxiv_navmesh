# vnavmesh improvement ideas

Analysis of the current codebase (`awgil/ffxiv_navmesh` @ `HEAD`) and its
DotRecast submodule (`xanunderscore/DotRecast`, a fork of `ikpil/DotRecast`
that is 304 commits behind upstream).

## Strategic recommendation

**Rebase to upstream `ikpil/DotRecast` for the 2D ground mesh. Do NOT build a
custom navmesh lib from scratch.**

- DotRecast/Detour handles the ground mesh correctly; it is the industry
  standard, and the 300+ missed upstream commits include real fixes
  (crash on large-scale generation recastnavigation#796, span validation)
  and perf work (multi-thread build, allocation rework). Free value left on
  the table.
- The actual quality problems live in vnavmesh's own **volume/flying
  layer** (`VoxelPathfind`, `VoxelMap`, `VoxelSearch`, `VoxelStraighten`),
  which is already custom code and not part of DotRecast.
- FFXIV geometry is not special enough to justify a from-scratch mesh lib:
  it's walkable surfaces, walls, and height layers — exactly what Recast was
  designed for. Territory-specific knowledge (aetheryte warps, client paths,
  shortcuts) is already injected via `NavmeshCustomization` without touching
  the core algorithm.
- Keep two specialized pathfinders (Detour for ground, custom voxel A* for
  flight). Do not unify them into one general 3D pathfinder — the TODO flirts
  with this; it is a trap.

## Fork rebase (DotRecast submodule)

The `xanunderscore/DotRecast` fork has only 13 fork-specific commits, mostly
WIP/hacks:

- `f60750c` "reduce allocations on hot path" — 1-line change.
- `91880dd` "pool RcSpans (breaking)" — span pooling during rasterization.
- `abff290` "add early exit support to pathfind" — adds `goto break_outer`
  when heuristic goes negative (used by `GoalRadiusHeuristic`). Has a
  **redundant nested `if (heuristicCost < 0)` check** — copy-paste leftover.
- The rest are WIP experiments ("WIP long distance portals", "wip edge
  jump generation", "Experiment: always use edge intersection point").

Plan:
1. Rebase submodule to `ikpil/DotRecast` main.
2. Re-apply the 2 useful fork patches (span pooling, early exit) cleanly.
   The early-exit patch needs the double `if` removed. The span-pooling
   patch is marked "breaking" — verify it still helps against current
   upstream before keeping it.
3. Drop the WIP commits — they are unfinished experiments.

## Volume pathfinding (the flying path — highest leverage)

`VoxelPathfind.cs` is hand-written A* over a 3-level hierarchical voxel
octree. The hierarchical design is good (skips empty regions), but:

### 1. Implement string-pulling for volume paths (HIGHEST PRIORITY)

`VoxelStraighten.cs` is **entirely commented out**. `NavmeshQuery.PathfindVolume`
has a `// TODO: string-pulling support`. Flying paths come out as
voxel-center staircases with visible 90-degree kinks at every voxel face
crossing. A funnel algorithm over axis-aligned voxel faces is tractable
and would cut waypoint count by 5-10x and make movement visually smooth.

The skeleton in `VoxelStraighten.cs` already has the right structure
(face/edge/vertex transition cases). It just needs finishing.

### 2. Replace `_nodeLookup` Dictionary with a flat indexed array

`Dictionary<ulong, int>` lookup on every node visit. For long flying paths
through dense voxel grids this is the dominant cost. The voxel encoding
is already a packed hierarchical index `(L2<<32)|(L1<<16)|L0` — a perfect
dense key for the reachable voxel space. Even a 2x-4x speedup on long
flying paths.

### 3. Remove `Random` from `CalculateGScore`

```csharp
float randomFactor = (float)_rng.NextDouble() * Service.Config.RandomnessMultiplier;
```

Non-determinism in A* cost breaks optimality and makes paths jitter between
runs. If tie-breaking is needed, use the standard `HScore`-as-tiebreaker or
a deterministic small perturbation based on voxel index. This is a
correctness smell, not a feature.

### 4. Cache `FindClosestVoxelPoint` or inline it

`CalculateGScore` calls `VoxelSearch.FindClosestVoxelPoint` for every
neighbor — each does a tree walk down to the leaf and a `Vector3.Clamp`.
This happens for every candidate neighbor of every popped node.

### 5. Add budget-aware early exit

`ExecuteStep` runs up to 1,000,000 times with only a 1024-step cancellation
check. No early termination on goal — only on `_openList.Count == 0` or
`nodeSpan[_bestNodeIndex].HScore <= 0`. A path that can't reach the goal
will burn all 1M steps. Add a configurable step budget and fall back to the
best node (`_bestNodeIndex`) when exceeded.

### 6. Revisit `_allowReopen = false`

Hardcoded off with a comment that it is "extremely expensive and doesn't
seem to actually improve the result." Usually fine with an admissible
heuristic, but combined with the random cost factor above, non-optimal
paths can result that reopening would have fixed. Revisit after #3 is done.

## Mesh pathfinding (`NavmeshQuery.PathfindMesh`)

Uses standard Detour A* + optional string-pull. Fine for ground, but:

### 7. `FindNearestMeshPoly` uses the wrong filter

`FindNearestMeshPoly` uses `_filter` (default `DtQueryDefaultFilter`) even
when `PathfindMesh` uses `_pathFilter` (teleport-aware). Nearest-poly lookup
ignores the teleport/area-cost model, so the start/end poly can be
inconsistent with the path filter — mismatch can pick a start poly the
path filter would refuse to traverse. Use the same filter.

### 8. Batch `FindPointOnFloor`

```csharp
IEnumerable<long> polys = FindIntersectingMeshPolys(p, new(halfExtentXZ, 2048, halfExtentXZ), allowUnreachable);
return polys.Select(poly => FindNearestPointOnMeshPoly(p, poly))
            .Where(pt => pt != null && pt.Value.Y <= p.Y)
            .MaxBy(pt => pt!.Value.Y);
```

Per-poly `ClosestPointOnPoly` calls are N separate Detour calls instead of
a batched query. On large maps this is a hot path that pays full Y-column
cost every call.

### 9. Fix commented-out end-clamp for partial paths

`NavmeshQuery.PathfindMesh` has commented-out end-clamping
(`//if (polysPath.Last() != endRef)`). Partial paths feed straight into
string-pulling with the raw `to` as endpoint, which can produce a final
waypoint floating off-mesh.

## FollowPath / movement

### 10. Add local re-pathing on stuck instead of full abort

`FollowPath.Update` does not re-query the navmesh when the player drifts
off the planned line beyond `Tolerance` — it just drops the current
waypoint and continues. For long paths a single stuck event forces a full
replan from the caller (via `OnStuck`). A local re-path from current
position to the next-keeping waypoint would be cheaper and smoother.

### 11. Jump-to-takeoff cadence

`ExecuteJump` spams `ActionManager.UseAction(GeneralAction, 2)` every 100ms
while waiting for flight. Works, but relies on a fixed 100ms cadence
rather than detecting the mounted-and-flying transition. Detect
`ConditionFlag.InFlight` / `ConditionFlag.Mounted` state change and stop
spamming once airborne.

### 12. Stuck-detection unit semantics

`distance = Vector3.Distance(...) / delta` compares to `StuckTolerance`
in yalms/sec — fine, but the threshold semantics are easy to misconfigure.
Document the unit or rename the config field to make it explicit.

## Priority order

1. Volume string-pulling (#1) — highest quality impact, skeleton exists.
2. Flat-array node lookup (#2) — biggest perf win on flying paths.
3. Remove random cost factor (#3) — correctness fix.
4. Rebase DotRecast submodule to upstream — pick up 300+ commits of fixes.
5. Local re-pathing on stuck (#10) — resilience win for long paths.
6. Batch `FindPointOnFloor` (#8) — hot-path perf on ground.
7. Filter consistency (#7) and partial-path end-clamp (#9) — correctness.
8. Everything else as time permits.