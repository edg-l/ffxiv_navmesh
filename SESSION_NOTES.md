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

## Phase 2 — DotRecast submodule rebase (BLOCKED, needs user decision)

Status: STOPPED. Submodule restored to original pointer `8002b30`. No changes
committed for Phase 2. Phases 0 and 1 are committed and intact.

### What blocked

The plan assumed the fork's only meaningful Detour change was `abff290` (early
exit), and that the rest of the fork's Detour work was droppable WIP. Wrong.
The plugin's call site at `NavmeshQuery.cs:96`:

```csharp
MeshQuery.FindPath(startRef, endRef, from.SystemToRecast(), to.SystemToRecast(),
    _pathFilter, ref _lastPath, opt);
```

uses a fork-specific overload `FindPath(..., ref List<long> path, DtFindPathOption fpo)`
that does NOT exist in upstream `ikpil/DotRecast`. Upstream's signature is:

```csharp
FindPath(long startRef, long endRef, RcVec3f startPos, RcVec3f endPos,
    IDtQueryFilter filter, Span<long> path, out int pathCount, int maxPath)
```

The fork's overload was built up across MULTIPLE fork commits, not just
`abff290`:
- `de3af0fc` "Experiment: always use edge intersection point for pathfinding"
- `abff2907` "add early exit support to pathfind"
- `60843e71` "WIP long distance portals"
- `4b20f884` "fix raycast shortcut endpoint bug (again?)"

All four touch `DtNavMeshQuery.cs` and together define the
`FindPath(..., DtFindPathOption)` overload + the `IDtQueryHeuristic`
plumbing + `GetEdgeIntersectionPoint` usage + the `GoalRadiusHeuristic`
support that the plugin's `NavmeshQuery.GoalRadiusHeuristic` depends on.

Upstream DOES have `DtFindPathOption`, `IDtQueryHeuristic`,
`DtDefaultQueryHeuristic` structs/classes — but not the `FindPath` overload
that takes them. Upstream's `FindPath` uses `Span<long>` + `out int pathCount`
+ `int maxPath`, not `ref List<long>` + `DtFindPathOption`.

Additionally, the plan's other keep-patch `91880dd` (span pooling) is
REDUNDANT: upstream added `RcSpanPool` + `AllocSpan/FreeSpan` API in April 2024
(commits `6b2bd27b` and `f49f9eb5`), two months after the fork's March 2024
attempt. Drop `91880dd` entirely — upstream's version is canonical.

### What a clean rebase actually requires

This is NOT a 2-patch cherry-pick. It requires:
1. Port the fork's `FindPath(..., ref List<long>, DtFindPathOption)` overload
   onto upstream's current `DtNavMeshQuery.cs`, reconciling with upstream's
   `Span<long>`-based API. This means writing a new overload that wraps or
   replaces upstream's signature, and porting the `IDtQueryHeuristic` call
   site (the `heuristic.GetCost(neighbourPos, endPos)` line) into the
   upstream loop structure.
2. Port the `GetEdgeIntersectionPoint` usage from `de3af0fc` (the fork uses
   it unconditionally; upstream uses `GetEdgeMidPoint`).
3. Port the early-exit `goto break_outer` from `abff290`, fixing the
   double-`if` bug, into the upstream loop structure.
4. Decide whether to keep `60843e71` (WIP long distance portals) and
   `4b20f884` (raycast shortcut endpoint fix) — these are WIP but the
   plugin may depend on their behavior. Need to check if removing them
   breaks the plugin's `useRaycast` path.
5. Re-verify the plugin's `GoalRadiusHeuristic` (in `NavmeshQuery.cs`)
   still works — it relies on the `IDtQueryHeuristic.GetCost` returning -1
   for the early-exit trigger, which is fork-specific behavior.

This is a substantial porting effort, not a cherry-pick. The plan's
threshold (">10 call sites need rewriting -> STOP and surface") is exceeded
in spirit: it's not 10 call sites, but it's one core API rewrite that
cascades into behavioral questions about 4 fork commits.

### Decision needed from user

Options:
A. **Abort Phase 2 entirely.** Keep the stale fork submodule as-is. Phases
   0 and 1 are done and valuable on their own. The DotRecast fork works;
   it's just stale. Document this as a known limitation.
B. **Port the fork's FindPath overload to upstream.** Substantial work:
   write a new overload on upstream's `DtNavMeshQuery.cs` that takes
   `DtFindPathOption` + `ref List<long>`, port the `IDtQueryHeuristic`
   call site, port the early-exit (fixing the double-if), decide on the
   WIP commits. Then bump the submodule pointer. This is a real
   feature-port, not a rebase.
C. **Adapt the plugin to upstream's API.** Change `NavmeshQuery.cs:96` to
   call upstream's `FindPath(..., Span<long>, out int, int)` instead of
   the fork's overload. Lose the `DtFindPathOption` / `GoalRadiusHeuristic`
   / early-exit behavior. This means the plugin's `range > 0` pathfinding
   (used by `Nav.PathfindWithTolerance`) degrades to no-early-exit.
   Simpler but a behavior regression.

Recommend A (abort) or B (port) over C (regress). A is the lowest risk;
B is the highest value. C loses a feature the plugin actively uses.

### Fork submodule state (unchanged)

- Pointer: `8002b30b9f196bcd9eb0a898e51abcea4177fb04` (original)
- Branch: detached HEAD at the fork's tip
- No branches created, no cherry-picks applied, no pushes.
- To resume Phase 2 from a clean state, the submodule is ready as-is.

## Decisions log

- Keep two specialized pathfinders (Detour for ground, custom voxel A* for flight). Do NOT unify into a general 3D pathfinder — the TODO file flirts with this; it is a trap.
- LoS-based string-pull (Phase 1) is preferred over the half-written projection-funnel skeleton. Simpler, obviously correct (every segment is LoS-verified by construction), reuses tested `VoxelSearch.LineOfSight`. Produces slightly more waypoints than an ideal funnel (stops at last voxel before LoS breaks, not at the optimal face-crossing point) but still hits the 5-10x reduction target. A tighter funnel can be a follow-up if needed.
- Fork manifest description explicitly notes "fork" and "drop-in IPC compatible" so users installing alongside upstream `vnavmesh` (same `InternalName`) get a clear choice. Same `InternalName` means they cannot coexist — by design.