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

## PLAN2 — Custom pathfinding lib (DONE except in-game verification, 10/11 phases done)

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
- Phase 7 (reverted, committed `608aec1`): **ABANDONED.** Turn-rate limiting caused stuck pathfinding at tight corners. FFXIV binary movement + the game's own instant direction snap (original vnavmesh behavior) is what works. All Phase 7 code removed (MaxTurnRateDeg, LastDt, azimuth clamp, ComputeMagnitude, config fields). `OverrideMovement` is back to original binary direction. Spline smoothing (Phase 6) remains as the only movement-quality layer.

- Phase 9 (static verification only; in-game deferred to Phase 11): IPC surface byte-identical to Phase 0 (`git diff 8c8e484 -- vnavmesh/IPCProvider.cs` empty). All 31 IPC method delegate targets compile. No signature drift.
- Phase 10 (committed `5748e22` + `7783fda` + `b6ab123`): Delete DotRecast + all dependent code. Ported 6 customizations (Z0155/Z0613/Z1237/Z1291/Z1310/Z1319) from LinkPoints to LinkQuads (10.0). Deleted entire Recast/Detour pipeline from NavmeshBuilder, NavmeshRasterizer, Navmesh.cs (Mesh field dropped, record now `(CustomizationVersion, QuadGraph? Ground, VoxelMap? Volume)`, Version 25->26), NavmeshCustomization (LinkPoints/CustomizeSettings/CustomizeMesh removed), NavmeshSettings (Recast/jump-link fields removed), NavmeshBitmap (RasterizePolygon takes Quad), NavmeshManager (BuildBitmap uses quad graph, Prune dropped), NavmeshQuery (pure quad-graph, no DtNavMesh fallback), Extensions (SystemToRecast/RecastToSystem removed). Deleted 10 Recast-only Debug files, created DebugQuadGraph. Removed DotRecast submodule + .gitmodules. Net -3407 lines. Build: 0 errors, 0 warnings (first time with zero warnings — DotRecast pre-existing warnings gone). `rg DotRecast vnavmesh/` -> 0. `rg DtNavMesh|RcHeightfield|...|SystemToRecast` -> 0.

### Remaining phases

- Phase 11: In-game verification + tuning + final audit. This is the only remaining phase. All code is done; needs in-game testing of ground pathfinding on the quad graph, fly pathfinding on voxel A*, all 31 IPC methods, IPC consumers (SealHunter, Hunty, etc.) work unchanged.

All committed phases build clean: 0 errors, 0 warnings. DotRecast submodule
is gone. The plugin is now a self-contained FFXIV pathfinding lib with no
external navmesh dependencies.

## PLAN3 — Any-angle ground pathfinding (CDT/LCT + Polyanya)

Plan file: `PLAN3.md` (do NOT commit). Replaces the Recast-clone ground stack
(greedy quads → A*-on-centers + funnel) with a modern any-angle navmesh,
staged and reversible, gated by an offline test harness (`vnavmesh.Tests/`,
`net10.0-windows`, NOT in the sln). 6 phases (0–6). No in-game verification
available to the implementer; synthetic scenes are the offline proxy.

### Phases 0–2 (DONE, committed `49e964d`, deployed v1.2.3.33)

Single commit "feat: any-angle ground pathfinding via Polyanya on triangulated
quads" covers Phase 0 (harness) + Phase 1 (predicates + Polyanya core) + Phase
2 (wire behind IPC seam, delete old stack):

- `vnavmesh/GroundGraph/Geometry/Predicates.cs` — Shewchuk adaptive
  Orient2D/InCircle (public domain, managed double-based adaptive expansion).
- `vnavmesh/GroundGraph/Polyanya/PolyMesh.cs` — triangulate every quad along a
  FIXED MinX-MaxZ→MaxX-MinZ diagonal, map shared portal spans to interior
  edges, non-portal sides = obstacle edges, every `Portal.IsOffMesh` →
  `OffMeshLink`. `static PolyMesh FromQuadGraph(QuadGraph)`.
- `vnavmesh/GroundGraph/Polyanya/PolyanyaSearch.cs` — interval-node search
  (Cui/Harabor/Grastien IJCAI 2017): observable + non-observable successors,
  cul-de-sac + intermediate pruning, binary-heap open list. Off-mesh links
  handled in the outer A* loop (discrete successor), NOT as zero-width
  intervals. `range` terminates within range of goal; `range==0` = exact goal.
- `vnavmesh/GroundGraph/QuadGraph.cs` — `Pathfind` builds/caches
  `PolyMesh.FromQuadGraph(this)` (invalidated on `BuildAdjacency`/`InitFlags`)
  and runs `PolyanyaSearch` instead of `QuadPathfind`+`FunnelStringPull`.
  Signature byte-identical. `useStringPulling`/`useRaycast` are now NO-OPS
  (any-angle supersedes both; accepted for IPC compat). Reachability gates
  earlier: `!fromReachable || !toReachable` → empty.
- DELETED `vnavmesh/GroundGraph/QuadPathfind.cs` + `FunnelStringPull.cs`
  (deletion compile-guaranteed by the build gate).
- `vnavmesh/NavVolume/VoxelMap.cs` — added `FillBox(min,max)` (marks every leaf
  voxel whose center is in `[min,max]` solid, subdividing tiles) for synthetic
  scene rasterization in the harness.
- `vnavmesh/Navmesh.cs` — `[assembly: InternalsVisibleTo("vnavmesh.Tests")]`;
  `SerializeGround` made `internal` for golden-snapshot tests.
- `vnavmesh.Tests/` — Predicates/Polyanya/Range/funnel-comparison tests,
  GeometryOracle/ClearanceOracle, golden snapshots, FunnelFixtureCapture
  (captured funnel reference into `Fixtures/funnel_reference.json` BEFORE the
  funnel was deleted). **51 tests / 0 skipped** (FlatPlane any-angle unskipped
  per task 2.5). Goldens in `vnavmesh.Tests/Goldens/`.

Build: 0 errors, 0 warnings. Test gate:
`~/.dotnet/dotnet test vnavmesh.Tests/vnavmesh.Tests.csproj` → 51/51 pass.

Version note: csproj `<Version>` 1.2.3.31 → .32 (committed) → **.33** (deployed
for in-game test). PLAN3 says "Phase 2 no version bump" — that refers to the
SERIALIZATION `Navmesh.Version` (unchanged; Phase 3 bumps it 27→28), NOT the
plugin package `<Version>`, which is bumped per-publish as usual.

### Needs in-game verification (PLAN3 Phases 0–2)
- Ground `Nav.Pathfind` (`fly=false`) reaches destinations on real zone meshes;
  the offline harness only exercises 6 synthetic scenes.
- Paths are visibly taut/any-angle (no staircase kinks across open quads) and
  have materially fewer waypoints than the old A*+funnel output.
- `range`-based queries (`Nav.PathfindWithTolerance`,
  `SimpleMove.PathfindAndMoveCloseTo`) still stop within tolerance of the goal.
- IPC consumers (SealHunter, Hunty, Lifestream, AutoDuty, Questionable, …)
  resolve all ground delegates unchanged; `useStringPulling`-as-no-op does not
  break any caller (they get a taut path with a different waypoint count).
- Reachability pruning still excludes islands (`FLAG_UNREACHABLE` honored by
  Polyanya); player ends up standing on the floor at the final waypoint.

### Phase 3 (DONE, committed `d44067f`, deployed v1.2.3.36)

Root cause found via the `RealNavmeshDiag` harness on the real Limsa navmesh:
ground pathfind failed because the **2y voxel resolution fragmented the quad
graph** — 1210 connected components, 837 isolated singletons, ramps split
because at 2y a walkable ramp steps 4y between cells (> climb=3). The fine
0.25y rasterization already existed but was downsampled to 2y before the ground
mesher saw it. Phase 3 extracts the ground mesh from the fine data instead.

- `NavmeshRasterizer`: accumulates raw spans `(y0,y1,area)`; `PopulateChf()`
  flushes the fine `CompactHeightfield` before rasterizer state is discarded.
- `GroundGraph/Extraction/CompactHeightfield.cs`: per-tile fine 0.25y walkable
  grid; per-column floors at solid-span TOP (GAP2 fix) + clearance
  (bottom-of-ceiling → floor). `FromVoxelMap` is a test-only helper.
- `GroundGraph/Extraction/LayerPartition.cs`: 4-connected within-climb layering;
  contour extraction with collinear merge (loop assembly deferred to Phase 4
  CDT); inter-layer links (within-climb seams → off-mesh links, over-climb →
  walls).
- `QuadMesher.GreedyMesh` rewritten to mesh per layer from the merged CHF.
  DELETED `ScanMixedTileForSurfaces`/`MarkColumnRange`/2y `IsClearBetween`
  (`rg` → 0). Ground extraction never reads the 2y VoxelMap; the octree now
  serves flight only.
- `NavmeshBuilder`: per-tile CHF, `StitchTileCHFs` (drop border cells) +
  `HealTileSeams` (snap/average flanking floor-Y within CellHeight; warns when
  the gap exceeds CellHeight).
- `PolyMesh.FromQuadGraph`: off-mesh link endpoints resolve to the triangle
  containing them (not face[0]), so inter-layer ramp links route correctly.
- `Navmesh.Version` 27→28. `LayerTests` + `Phase3GeometryTests` (73 tests
  total). Connectivity validated at the EXACT production climb (0.5), not a
  margin: Overpass → 2 disjoint layers; BridgeOnramp/Staircase within-climb →
  1 component; over-climb → disconnected. Goldens regenerated.

Build 0/0, tests 73/73. Code-reviewed; 6 findings fixed (test-climb fidelity,
seam-gap warning, collinear merge, clearance y0, CHF null-volume guard, off-mesh
face selection).

### Needs in-game verification (Phase 3)
- Rebuild a real multi-level zone (`/vnav rebuild` in Limsa `s1t2`) on v1.2.3.36+
  and re-upload the `.navmesh`; the `RealNavmeshDiag` harness (now v28) should
  show the quad graph collapsing from ~1210 components to a few, and the
  ramp/corner from→to landing in ONE component.
- Walk down the ramp + corner that previously had no path; confirm `ground OK`
  with `wps>0` in the devlog and the player follows it.
- Confirm ledges/walls still block (over-climb seams stayed walls, not links).
- Synthetic fuzz is a proxy; real-zone CDT robustness is Phase 4's risk.

### Remaining PLAN3 phases
- Phase 4: CDT mesh from contours, gated by `Config.UseCdtMesh` (code-default
  false). Bumps `Navmesh.Version` 28→29.
- Phase 5: LCT clearance (5.1 ship gate = per-edge approximate clearance,
  reject crossing when `Clearance < 2*agentRadius`). Bumps 29→30.
- Phase 6: additive `Nav.PathfindWithRadius` IPC + debug viz; consolidation.