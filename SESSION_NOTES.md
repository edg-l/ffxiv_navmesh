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

## Phase 1 — Volume string-pulling (next)

Plan: rewrite `VoxelStraighten.cs` (currently entirely commented out) as a live class with `Simplify(List<(ulong voxel, Vector3 p)> path, Vector3 toPos)` using an iterative LoS-based string-pull (anchor/probe loop reusing `VoxelSearch.LineOfSight`). Wire into `NavmeshQuery.PathfindVolume` behind `useStringPulling`, replacing the `// TODO: string-pulling support` block.

Key invariants (verified during plan review):
- `FindClosestVoxelPoint` clamps with `eps=0.1f`, so a committed waypoint is always strictly inside its source voxel's AABB — `path[anchor].voxel` remains valid for the next LoS call.
- `EnumerateVoxelsInLine` throws `PathfindLoopException` on float-error re-entry at a voxel boundary; `Simplify` must wrap the LoS call in try/catch and treat it as "no LoS" (commit previous, continue).
- Do NOT conflate with `BuildPathToVisitedNode`'s `returnIntermediatePoints=true` (that subdivides, producing more points; string-pulling is a separate post-processing pass on the raw A* path).

## Phase 2 — DotRecast submodule rebase (last)

Plan: fetch upstream, branch `rebase-to-upstream` off `upstream/main`, cherry-pick `91880dd` (span pooling, breaking, verify build) and `abff290` (early exit, fix the double `if (heuristicCost < 0)` copy-paste bug while cherry-picking), push to `origin` (confirm branch name with user first), bump the submodule pointer in the parent repo (stage but do NOT commit — user decides).

Risks flagged:
- Span pooling may be redundant or conflicting if upstream already reworked RcSpan in its 304 commits. Pre-cherry-pick recon: `git log --oneline 4b8cd8e..upstream/main -- src/DotRecast.Recast/RcSpan.cs src/DotRecast.Recast/RcHeightfield.cs` to check.
- If upstream API churn breaks >10 plugin call sites in `vnavmesh/`, STOP and surface to the user with a fix list for approval.
- The `goto break_outer` label in `abff290` must land in a loop structure that still supports it. Current upstream `DtNavMeshQuery.cs` has the label at line ~1001; verify after cherry-pick.

## Decisions log

- Keep two specialized pathfinders (Detour for ground, custom voxel A* for flight). Do NOT unify into a general 3D pathfinder — the TODO file flirts with this; it is a trap.
- LoS-based string-pull (Phase 1) is preferred over the half-written projection-funnel skeleton. Simpler, obviously correct (every segment is LoS-verified by construction), reuses tested `VoxelSearch.LineOfSight`. Produces slightly more waypoints than an ideal funnel (stops at last voxel before LoS breaks, not at the optimal face-crossing point) but still hits the 5-10x reduction target. A tighter funnel can be a follow-up if needed.
- Fork manifest description explicitly notes "fork" and "drop-in IPC compatible" so users installing alongside upstream `vnavmesh` (same `InternalName`) get a clear choice. Same `InternalName` means they cannot coexist — by design.