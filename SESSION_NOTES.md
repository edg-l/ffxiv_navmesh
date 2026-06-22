# vnavmesh fork — Session Notes

Incremental log of caveats, in-game verification needs, and decisions across
sessions. Updated per phase. Read this before resuming work.

## Environment

- Build command: `DALAMUD_HOME=~/.cache/dalamud-dev DOTNET_ROOT=~/.dotnet ~/.dotnet/dotnet build vnavmesh/vnavmesh.csproj -c Release -p:Platform=x64`
- CRITICAL: must invoke `~/.dotnet/dotnet` (SDK 10). System `dotnet` at `/usr/bin/dotnet` is SDK 9 and fails with `NETSDK1045: The current .NET SDK does not support targeting .NET 10.0`. Setting `DOTNET_ROOT=~/.dotnet` alone is NOT enough if the resolved `dotnet` on PATH is the system one — call the binary explicitly or ensure PATH order.
- Baseline build (clean main): 0 errors, 3 warnings — all in the DotRecast submodule (RcThrowHelper CS0219, DtNavMesh CS0162, RcLayers CS0162). Not our code; safe to ignore for our phases.
- Version lives in `vnavmesh/vnavmesh.csproj` `<Version>1.2.3.7</Version>` (SDK injects it into the packaged manifest). The sidecar `vnavmesh.json` does NOT carry the version. Bump in the csproj before each publish.

## Remotes (already configured)

Parent repo `ffxiv_navmesh`:
- `origin` = `git@github.com:edg-l/ffxiv_navmesh.git` (this fork)
- `upstream` = `https://github.com/awgil/ffxiv_navmesh.git`

Submodule `DotRecast/`:
- `origin` = `git@github.com:edg-l/DotRecast.git` (our fork)
- `upstream` = `https://github.com/ikpil/DotRecast.git`
- `upstream-xan` = `https://github.com/xanunderscore/DotRecast` (old base)

Init after checkout: `git submodule update --init --recursive`.

## Phase 0 — Quick volume-layer wins (DONE, committed)

What changed:
- `vnavmesh/NavVolume/VoxelPathfind.cs`:
  - Removed `private Random _rng = new();` and `randomFactor` from `CalculateGScore`. Return is now `parentBaseG + baseDistance + verticalPenalty`. A* tie-break via `HeapLess` (`nodeL.GScore > nodeR.GScore`) was already correct and is now the only tie-break.
  - Replaced `Dictionary<ulong, int> _nodeLookup` with an open-addressing flat `int[] _lookupTable` + `_lookupMask`, power-of-two sized, load factor 0.5, linear probing. Helpers: `VoxelHash` (Finalizer mix: `v ^= v>>33; v *= 0xff51afd7ed558ccdUL; v ^= v>>33; v *= 0xc4ceb9fe1a85ec53UL; v ^= v>>33`), `InitLookup`, `LookupGet`, `LookupSet`, `LookupGrow`. `Start` calls `InitLookup(4096)`; growth is O(N) and rare. `Node.Voxel` preserved as the key for re-insert on grow.
- `vnavmesh/Config.cs`:
  - Removed `RandomnessMultiplier` field and its `ImGui.SliderFloat` slider block. Existing config files with a stale `RandomnessMultiplier` JSON key load without error (reflection-based `Load` silently ignores unknown keys).

Build: 0 errors, 0 warnings (the 3 pre-existing DotRecast warnings are unchanged and unrelated).

Static checks (all pass):
- `rg "RandomnessMultiplier|_rng|Random" vnavmesh/` → no matches.
- `rg "_nodeLookup" vnavmesh/` → no matches.
- `rg "Dictionary<ulong, int>" vnavmesh/` → no matches.

IPC safety: no IPC method referenced `RandomnessMultiplier`; `Path.ListWaypoints` returns `List<Vector3>` of positions — string-pulling later will change count/positions but not the type or signature. Drop-in compatibility preserved.

### Needs in-game verification (Phase 0)
- Fly a long path in a zone with a built volume (Ishgard, Sharlayan) via IPC `Nav.Pathfind` with `fly=true`. Confirm:
  1. Path completes (no `PathfindLoopException`, no "failed to find path" log).
  2. Repeated identical pathfind calls (same from/to) produce byte-identical waypoint lists (verifies random-factor removal). Compare `Path.ListWaypoints` IPC twice.
  3. Long flying path (~200+ voxels) completes in materially less wall-time than pre-change (check the `Pathfind took {t}s` debug log line; enable Debug log level if needed).
- Ground path (`Nav.Pathfind` with `fly=false`) must be unchanged — confirms no regression to the mesh pathfinder.

### Notes for future sessions
- The flat-array lookup uses a Finalizer-style hash (the same constant family recommended by the plan reviewer: `v ^= v>>33; v *= 0xff51afd7ed558ccdUL; ...`). Do NOT switch to a naive `v ^ (v>>21) ^ (v>>42)` — low bits of L0 are the Y coordinate and cluster without proper mixing.
- `InitLookup(4096)` is a starting default. If a flying path ever exceeds 2048 visited voxels, `LookupGrow` doubles the table. The table sticks at the high-water mark across pathfinds (reused via `Array.Fill(-1)` in `Start`) — memory is tiny (16KB+ per size doubling), don't add a shrink path.
- If you ever need to revert `_allowReopen` to `true` (currently `false`), the flat-array lookup supports it correctly — `LookupGet` finds closed nodes and the reopen path in `VisitNeighbour` will update them. No code change needed beyond flipping the bool.

## Phase 1 — Volume string-pulling (DONE, committed)

What changed:
- `vnavmesh/NavVolume/VoxelStraighten.cs`: rewritten as a live class (was entirely commented out). `Simplify(List<(ulong voxel, Vector3 p)> path, Vector3 toPos) -> List<Vector3>` uses an iterative LoS-based string-pull: anchor + probe loop, commits `path[probe-1].p` when LoS from `result.Last()` to `path[probe].p` breaks, appends `path[^1].p` and `toPos` at the end. `HasLineOfSight` wraps `VoxelSearch.LineOfSight` in try/catch (`PathfindLoopException` -> no LoS) so a float-error re-entry at a voxel boundary doesn't abort the string-pull.
- `vnavmesh/NavmeshQuery.cs:PathfindVolume`: replaced the `// TODO: string-pulling support` block with an `if (useStringPulling)` branch that calls `VoxelStraighten.Simplify(voxelPath, to)` and maps to `Waypoint`s. Mirrors `PathfindMesh`'s `if (useStringPulling) { FindStraightPath... }` structure. The non-string-pulling branch keeps the old behavior (raw voxel points + `to`).

Edge cases handled (per plan review):
- `path.Count == 0`: returns `[toPos]`.
- `path.Count <= 2`: short-circuits to `[path[0].p, (path[1].p if != toPos), toPos]` without running the LoS loop.
- `FindClosestVoxelPoint` clamps with `eps=0.1f` so committed waypoints are strictly inside their source voxel's AABB — `path[anchor].voxel` stays valid for the next LoS call (no re-voxelization needed).
- `EnumerateVoxelsInLine` may throw `PathfindLoopException` on float-error re-entry at a voxel boundary; caught and treated as "no LoS".

Build: 0 errors, 0 warnings.

Static checks (all pass):
- `rg "TODO: string-pulling" vnavmesh/` -> no matches.
- `rg "^\s*//" vnavmesh/NavVolume/VoxelStraighten.cs` -> 0 (no commented-out remnants).

IPC safety: `Path.ListWaypoints` still returns `List<Vector3>` of positions — type and signature unchanged. Waypoint count and positions change when `useStringPulling=true` (default), but that is a documented behavior change, not an API break.

### Needs in-game verification (Phase 1)
- IPC `Nav.Pathfind` with `fly=true` on a path with a long straight segment (e.g. across a large open field in Ishgard). Compare `Path.ListWaypoints` count before (Phase 0 build) and after (Phase 1 build) for the same from/to. Expect 5-10x fewer waypoints.
- Enable `ShowWaypoints` config and visually confirm the waypoint line is smooth (no 90-degree kinks at every voxel face).
- For each segment, confirm the player does not clip through geometry when following with `Path.MoveTo`. Test in a zone with obstacles (pillars in a city). A clip means `LineOfSight` has a bug, not that string-pulling is wrong — debug via the `PathfindLoopException` catch.
- Confirm the final waypoint equals the requested `to` (within float epsilon) via `Path.ListWaypoints`.
- IPC `Nav.Pathfind` with `fly=false` unchanged (string-pulling only touches the volume path; mesh pathfinder already had its own `FindStraightPath`).

### Notes for future sessions
- The LoS-based string-pull stops at the last voxel before LoS breaks, not at the optimal face-crossing point. Produces slightly more waypoints than an ideal projection-funnel but hits the 5-10x reduction target. A tighter funnel can be a follow-up if in-game testing shows it's needed.
- `Simplify` calls `VoxelSearch.LineOfSight` which calls `EnumerateVoxelsInLine(...).All(v => v.empty)` — each LoS probe walks the voxel grid along the line. For very long paths with many probes, this is O(probes * voxels_per_line). If profiling shows this is hot, consider caching LoS results per (anchor, probe) pair or switching to the projection-funnel. Not a concern until measured.

## Phase 2 — DotRecast submodule rebase (ABORTED, upstream divergence too large)

Status: ABORTED. Submodule restored to original pointer `8002b30`. Plugin
reverted to pre-Phase-2 state. Phases 0 and 1 remain committed and intact.

### What was attempted

User chose option B (port the fork's FindPath to upstream). I created a
`rebase-to-upstream` branch off `upstream/main` and added adapter overloads
to `DtNavMeshQuery.cs`:
- `FindPath(..., ref List<long>, DtFindPathOption)` — wraps a new internal
  `FindPath(..., IDtQueryHeuristic, int options, float raycastLimit, Span<long>, ...)`
  that uses `heuristic.GetCost()` instead of the hardcoded
  `RcVec3f.Distance(...) * H_SCALE`, with the early-exit on negative heuristic
  (single `if`, no double-if bug).
- `FindStraightPath(..., List<long>, ref List<DtStraightPath>, int, int)` —
  wraps the upstream `Span`-based version.
- `QueryPolygons(..., Action<DtMeshTile, DtPoly, long>)` — wraps the upstream
  `long[]`-based version for the plugin's `IntersectQuery` callback.

These compiled cleanly in `DotRecast.Detour.csproj` (0 errors, 10 warnings —
all pre-existing CA2265 span-null comparisons in upstream code).

### Why it was aborted

Building the full plugin revealed **220 errors** across 22 plugin files. The
fork's divergence from upstream spans the ENTIRE Recast/Detour API surface,
not just `FindPath`. The plugin depends on fork-specific:

Detour APIs (missing or renamed in upstream):
- `DtMeshTile.polyLinks` (fork) vs `DtPoly.firstLink` (upstream) — 10 call sites
- `DtNavMesh.AllocLink`, `DT_NULL_LINK`, `DT_POLY_BITS`, `DT_NAVMESH_MAGIC`,
  `DT_NAVMESH_VERSION`, `DT_OFFMESH_CON_BIDIR`, `DecodePolyIdSalt`,
  `EncodePolyId` — all removed/renamed in upstream
- `DtNavMesh(params)` 2-arg constructor — upstream has different ctor signature
- `DtNavMesh.AddTile(...)` without `out long result` — upstream requires it
- `DtNavMesh.GetTileCount()` — removed in upstream
- `RcBuilderResult.tileX`/`tileZ` — renamed in upstream
- `RcVec3i[int]` setter — upstream's indexer is read-only
- `JumpLinkBuilder`, `JumpLink`, `JumpLinkBuilderConfig`, `JumpLinkType` —
  entire jump-link subsystem removed from upstream Recast

Recast APIs (missing or renamed in upstream):
- `RcConstants` — removed; constants inlined or moved
- `RcCommons` — removed
- `RcVecUtils` — removed
- `RcSpan` (fork: class with implicit int conversion for pooling) vs upstream
  `RcSpan` (different shape after upstream's own pooling rework)
- `RcHeightfield.Span`, `RcHeightfield.spanPool` — fork's span-pooling API;
  upstream uses `RcSpanPool` + `AllocSpan`/`FreeSpan`

The 220 errors break down as:
- ~80 in `Debug/` files (debug visualization, uses RcConstants/RcCommons/RcSpan heavily)
- ~30 in `NavmeshBuilder.cs` (jump links, RcVecUtils, RcBuilderResult)
- ~25 in `NavmeshRasterizer.cs` (RcSpan/RcHeightfield.Span/spanPool — the
  fork's span pooling that the plan said to drop, but the plugin's
  RASTERIZER directly accesses the internal span data structure)
- ~20 in `NavmeshCustomization.cs` (polyLinks, AllocLink, DT_NULL_LINK,
  DecodePolyIdSalt, EncodePolyId, DT_OFFMESH_CON_BIDIR)
- ~15 in `Navmesh.cs` (DtNavMesh ctor, AddTile, GetTileCount, DT_NAVMESH_MAGIC/
  VERSION, RcVec3i readonly)
- ~10 in `NavmeshQuery.cs` (polyLinks, DT_NULL_LINK)

### Root cause

The `xanunderscore/DotRecast` fork was not a thin patch on upstream — it was
a significant API refactoring: it renamed/moved constants (`RcConstants`,
`RcCommons`, `RcVecUtils`), restructured `RcSpan`/`RcHeightfield` for pooling
(the plugin's `NavmeshRasterizer.cs` accesses the pooled span internals
directly), changed `DtMeshTile` link storage (`polyLinks` vs `firstLink`),
reworked `DtNavMesh` construction and serialization, and removed the
`JumpLinkBuilder` subsystem. The plugin was written against the fork's API
shapes, not upstream's.

### What a real migration would require

This is a full plugin port from the fork's Recast/Detour API to upstream's,
across 22 files and 220 call sites. Key work items:
1. Rewrite `NavmeshRasterizer.cs` to use upstream's `RcSpanPool`/`AllocSpan`/
   `FreeSpan` instead of the fork's `RcHeightfield.Span`/`spanPool` direct
   access. This is the hardest part — the rasterizer directly walks the span
   linked list.
2. Port `NavmeshBuilder.cs` jump-link logic or remove it (upstream removed
   `JumpLinkBuilder` entirely; the fork's jump links were WIP anyway per the
   TODO file).
3. Replace all `polyLinks[poly.index]` with `poly.firstLink` (10 sites).
4. Replace all `RcConstants`/`RcCommons`/`RcVecUtils` with upstream equivalents
   (~30 sites). Need to find where the constants moved to.
5. Adapt `Navmesh.cs` serialization to upstream's `DtNavMesh` ctor and
   `AddTile` signature, plus find where `DT_NAVMESH_MAGIC`/`VERSION` moved.
6. Rewrite `NavmeshCustomization.cs` link manipulation to use upstream's
   `DtNavMesh` link API (no `AllocLink`, no `DecodePolyIdSalt`/`EncodePolyId`).
7. Rewrite all `Debug/*.cs` files for upstream's Recast struct shapes.

This is estimated at 2-4 hours of focused porting work, not a cherry-pick or
adapter-layer job. The adapter-overload approach I started (adding `FindPath`
+ `FindStraightPath` + `QueryPolygons` overloads) addressed the query-side
API but not the build-side or rasterizer-side APIs, which need source-level
porting.

### Decision

Phase 2 is abandoned. The fork submodule stays at `8002b30` (the original
`xanunderscore/DotRecast` fork). Phases 0 and 1 (the volume-layer improvements)
are complete and valuable on their own; they don't depend on the DotRecast
version because they only touch the plugin's own `NavVolume/` code.

The DotRecast fork is stale (304 commits behind upstream) but functional.
A future migration to upstream is possible but requires a dedicated porting
effort, not a rebase. Document this as a known limitation in the fork's
README and SESSION_NOTES.

### Fork submodule state (unchanged, clean)

- Pointer: `8002b30b9f196bcd9eb0a898e51abcea4177fb04` (original)
- HEAD: detached at `8002b30b`
- No branches created (the temporary `rebase-to-upstream` was deleted)
- No cherry-picks applied, no stashes, no pushes
- Plugin working tree: clean (reverted to Phase 1 state)

### If a future session wants to retry Phase 2

Read this section first. The adapter-overload approach for the query side
(`FindPath`/`FindStraightPath`/`QueryPolygons`) is viable and was prototyped
(see the stashed diff if still in reflog, or re-derive from the description
above). But the build-side and rasterizer-side APIs (`RcSpan`, `RcHeightfield`,
`RcConstants`, `polyLinks`, `JumpLinkBuilder`, `DtNavMesh` ctor) require
source-level porting of `NavmeshRasterizer.cs`, `NavmeshBuilder.cs`,
`NavmeshCustomization.cs`, `Navmesh.cs`, and all `Debug/*.cs` files. Budget
2-4 hours minimum, and expect to make tradeoff decisions about jump links
(upstream removed them; the fork's were WIP).

## Decisions log

- Keep two specialized pathfinders (Detour for ground, custom voxel A* for flight). Do NOT unify into a general 3D pathfinder — the TODO file flirts with this; it is a trap.
- LoS-based string-pull (Phase 1) is preferred over the half-written projection-funnel skeleton. Simpler, obviously correct (every segment is LoS-verified by construction), reuses tested `VoxelSearch.LineOfSight`. Produces slightly more waypoints than an ideal funnel (stops at last voxel before LoS breaks, not at the optimal face-crossing point) but still hits the 5-10x reduction target. A tighter funnel can be a follow-up if needed.
- Fork manifest description explicitly notes "fork" and "drop-in IPC compatible" so users installing alongside upstream `vnavmesh` (same `InternalName`) get a clear choice. Same `InternalName` means they cannot coexist — by design.

## PLAN2 — Custom pathfinding lib (in progress, 7/11 phases done)

Replaces the aborted DotRecast rebase. The user chose to build a custom
FFXIV-specialized pathfinding lib. Architecture: **quad graph for ground**
(greedy-meshed from walkable voxels, A* on quads, funnel string-pull) +
**voxel A* for fly** (already done in Phases 0+1). No DotRecast dependency.

Plan file: `PLAN2.md` (1170 lines, 11 phases). Do NOT commit it.

### Completed phases (PLAN2)

- Phase 1 (committed `c712b45`): Ground quad mesher. `GroundGraph/QuadMesher.cs` + `QuadGraph.cs`. `Navmesh.cs` gains `QuadGraph? Ground`, Version 24->25.
- Phase 2 (committed `468da79`): Quad adjacency builder. `BuildAdjacency` wired into `NavmeshBuilder`.
- Phase 3 (committed `b89be49`): Quad A* + funnel string-pull. `QuadPathfind.cs` + `FunnelStringPull.cs`. `QuadGraph.Pathfind` entry point.
- Phase 4 (committed `bb08fd1`): Area annotations + territory markup port. `CustomizeGround`/`LinkQuads`/`AddOffMesh`. Z0132 + Z1252 ported. 6 other customizations (Z0155, Z0613, Z1237, Z1291, Z1310, Z1319) still use `LinkPoints` via `CustomizeMesh`; need porting before Phase 10.
- Phase 5 (committed `9983a4d`): Reachability pruning on quad graph + serialization (save/load VoxelMap + QuadGraph per zone).
- Phase 6 (committed `6f5b227`): Spline smoothing (Catmull-Rom through waypoints). `Movement/SplineSmoothing.cs`. `FollowPath.Move` applies it when `Config.SplineSmoothing` and `waypoints.Count >= 2`. Config gains `SplineSmoothing` + `SplineSegments` slider.
- Phase 7 (committed `af71874`, revised `3e65379`/uncommitted revision): Velocity-aware movement controller — **turn-rate limiting only**. `OverrideMovement` gains `MaxTurnRateDeg`/`LastDt` + azimuth-delta clamp in `DirectionToDestination` (prevents 180° snap when next waypoint is behind player). `FollowPath.Update` feeds `LastDt` + `Config.MoveMaxTurnRate`. `Config` gains `MoveMaxTurnRate` + slider. **Magnitude scaling reverted**: FFXIV movement is binary (run-speed or stop), so `ComputeMagnitude`/`easeOut`/`cornerMod`/`EaseDistance`/`NextSegmentDir` were removed — they caused overshoot-then-spin loops at stop and stuck the volume pathfinder at every spline corner. Corner slowdown (Test 1) still works because turn-rate limiting indirectly slows cornering without modulating speed. In-game verified: Test 1 (corner) pass, Test 3 (no 180° snap) pass. Test 2 (ease-to-stop) handled by `FollowPath.Tolerance` + game's natural deceleration, not magnitude.

### Remaining phases

- Phase 8: Rewrite NavmeshManager + NavmeshQuery (PathfindMesh -> QuadGraph.Pathfind, signatures preserved).
- Phase 9: Wire ground to quad graph + IPC verification.
- Phase 10: Delete DotRecast + dependent code (submodule deinit, ~3500 lines, 9+ Debug files). Port remaining 6 customizations.
- Phase 11: In-game verification + tuning + final audit.

All committed phases build clean: 0 errors, 0 warnings. Phase 7 uncommitted
but also builds clean; needs commit + in-game verification (sharp corner
slowdown, ease-to-stop, fly vertical stays full-strength).