# vnavmesh Improvement Plan

Three sequential phases. Do NOT start a phase until the previous phase is
verified complete. Do NOT commit this PLAN.md. Do NOT commit anything to git
unless explicitly asked by the user.

Build command (use at every checkpoint):

```
DALAMUD_HOME=~/.cache/dalamud-dev DOTNET_ROOT=~/.dotnet dotnet build vnavmesh/vnavmesh.csproj -c Release -p:Platform=x64
```

Conventions:
- This is a Dalamud FFXIV plugin. Most behavior can only be verified in-game.
  Each phase has an explicit "Needs in-game verification" list. Do not claim a
  phase is done based on build success alone for items that require in-game
  checks.
- No code comments unless explicitly requested.
- Minimal changes; do not refactor unrelated code.
- File paths are relative to `/home/edgar/dev/ffxiv_navmesh/`.

---

## Phase 0 — Quick volume-layer wins

Low-risk, independent of DotRecast. Two sub-tasks: remove the random cost
factor (#3), then replace the Dictionary node lookup with a flat array (#2).
Do #3 first because it changes `CalculateGScore`'s signature/body, which #2's
profiling will want to measure against a clean baseline.

### 0.1 Remove the `Random` cost factor from `CalculateGScore`

File: `vnavmesh/NavVolume/VoxelPathfind.cs`

- Delete the field at line 355: `private Random _rng = new();`
- In `CalculateGScore` (lines 356-396):
  - Delete line 358: `float randomFactor = (float)_rng.NextDouble() * Service.Config.RandomnessMultiplier;`
  - In the return statement at line 395, remove `+ randomFactor` so it reads:
    `return parentBaseG + baseDistance + verticalPenalty;`
- Tie-breaking: the existing `HeapLess` (lines 483-493) already tie-breaks
  toward larger G (`nodeL.GScore > nodeR.GScore`). This is the standard A*
  tie-break and is sufficient. Do NOT add a deterministic perturbation; the
  explicit G tie-break is cleaner and already present. Leave `HeapLess` as-is.
- Remove the `using System;` at line 1 ONLY if no other code in the file uses
  `System` types. Check: `MathF`, `float.MaxValue`, `Action` all live in
  `System` and are still used. Keep `using System;`.

### 0.2 Remove `RandomnessMultiplier` config field and UI

File: `vnavmesh/Config.cs`

- Delete the field at line 26: `public float RandomnessMultiplier = 1f;`
- Delete lines 83-85 (the `ImGui.SliderFloat("Randomness Multiplier", ...)`
  block and its `NotifyModified()` call). Also delete the `ImGui.SetNextItemWidth(200);`
  on line 83 that precedes it, since it has no other consumer in that spot
  (verify by reading the surrounding block; the next widget, if any, sets its
  own width).
- The `Load` method (lines 105-134) deserializes fields by reflection and
  silently ignores unknown JSON keys, so existing config files with a stale
  `RandomnessMultiplier` entry will load without error. Do NOT add migration
  code in `ConvertConfig` (line 136) — reflection-based load already handles
  the missing field. Leave `ConvertConfig` returning `payload` unchanged.

File: `vnavmesh/IPCProvider.cs`

- Search for `RandomnessMultiplier` (none found in the grep). Confirm no IPC
  method references it. If none, no change. (Verified: no references.)

File: `vnavmesh/MainWindow.cs`

- `MainWindow.Draw` delegates to `Service.Config.Draw()`. No direct reference
  to `RandomnessMultiplier`. No change. (Verified.)

File: `vnavmesh/NavmeshSettings.cs`

- This is a separate config class for mesh build settings; `RandomnessMultiplier`
  is not here. No change. (Verified.)

### 0.3 Replace `_nodeLookup` Dictionary with a flat indexed array

File: `vnavmesh/NavVolume/VoxelPathfind.cs`

Background (from `VoxelMap.cs`):
- A voxel index is packed as `(L2<<32)|(L1<<16)|L0`, where each level index is
  a `ushort` (16-bit). `IndexLevelMask = 0xffff`, `IndexLevelShift = 16`.
- The full 64-bit key space is sparse: most combinations of (L0,L1,L2) do not
  correspond to a real tile subdivision path. A naive `ulong[]` of size
  `2^48` (L0+L1 fill the low 32 bits; L2 fills the top 16) is infeasible.
- The reachable voxel space at query time is bounded by the number of nodes
  the A* actually opens — typically thousands to low tens of thousands for
  long flying paths. The Dictionary is currently re-hashing and bucket-walking
  on every one of those insertions/lookups.

Design decision — use a flat `int[]` indexed by a *dense node-sequence key*
assigned at first visit, with the Dictionary replaced by a small
open-addressing hash or, simpler and faster for our sizes, keep the
voxel→node-index mapping but back it with a flat array keyed by a *compact
hash* of the packed voxel into a power-of-two-sized table with linear
probing. This removes the `Dictionary` bucket-object overhead and the
`GetValueOrDefault` interface dispatch.

Concretely:
- Replace `private Dictionary<ulong, int> _nodeLookup = new();` (line 23) with:
  - `private int[] _lookupTable = Array.Empty<int>();` (stores node indices;
    `-1` = empty slot)
  - `private int _lookupMask;` (table size - 1, power-of-two)
- Add a helper `private static int VoxelHash(ulong v)` that mixes the bits of
  the packed voxel to spread the 48 used bits across the table (e.g.
  `v ^ (v >> 21) ^ (v >> 42)`, folded to 32 bits via `(int)(v ^ (v >> 32))`).
  This is necessary because low bits of L0 are the Y coordinate and would
  cluster without mixing.
- Add `private void InitLookup(int capacity)`:
  - size = next power of two >= `capacity * 2` (load factor 0.5). If the
    caller does not know capacity, use a default of 4096 and grow on demand.
  - `_lookupTable = new int[size]; Array.Fill(_lookupTable, -1); _lookupMask = size - 1;`
- Add `private int LookupGet(ulong voxel)`:
  - `int i = VoxelHash(voxel) & _lookupMask;`
  - `while (_lookupTable[i] >= 0) { if (_nodes[_lookupTable[i]].Voxel == voxel) return _lookupTable[i]; i = (i + 1) & _lookupMask; }`
  - `return -1;`
- Add `private void LookupSet(ulong voxel, int nodeIndex)`:
  - `int i = VoxelHash(voxel) & _lookupMask;`
  - `while (_lookupTable[i] >= 0) { i = (i + 1) & _lookupMask; }`
  - `_lookupTable[i] = nodeIndex;`
- Add `private void LookupGrow()`:
  - snapshot old table, double size, re-insert all entries by walking
    `_nodes` (each node carries its `Voxel`) — this is O(N) and rare.

Allocation strategy:
- Allocate per-pathfind in `Start` (line 48). The table is small (16KB for a
  4096-entry `int[]`). Reuse across pathfinds by re-`Array.Fill`-ing with -1
  in `Start` instead of reallocating, and only grow when load factor exceeded.
  This avoids per-query allocation after the first few queries warm up the
  size. Tradeoff: the table size sticks at the high-water mark of the longest
  recent path, which is fine (memory is tiny).

Call site changes:
- `Start` (lines 48-67):
  - Replace `_nodeLookup.Clear();` (line 51) with `InitLookup(4096);` (or the
    reuse path: `Array.Fill(_lookupTable, -1); _lookupMask = _lookupTable.Length - 1;`
    only re-init if table is empty).
  - Replace `_nodeLookup[fromVoxel] = 0;` (line 64) with `LookupSet(fromVoxel, 0);`
- `VisitNeighbour` (lines 319-353):
  - Replace `var nodeIndex = _nodeLookup.GetValueOrDefault(nodeVoxel, -1);`
    (line 321) with `var nodeIndex = LookupGet(nodeVoxel);`
  - Replace `_nodeLookup[nodeVoxel] = nodeIndex;` (line 327) with
    `LookupSet(nodeVoxel, nodeIndex);`
  - After `LookupSet`, check load factor: if `_nodeCount > _lookupTable.Length / 2`,
    call `LookupGrow()`. Track `_nodeCount` as `_nodes.Count` (already
    maintained). Insert the check at the end of the "first time visiting"
    branch (after line 327's replacement).

Do NOT change `Node.Voxel` (line 15) — it is the key and is read by
`LookupGet`/`LookupGet` re-insert. Keep the `Node` struct unchanged.

### 0.4 Verification (Phase 0)

Build:
```
DALAMUD_HOME=~/.cache/dalamud-dev DOTNET_ROOT=~/.dotnet dotnet build vnavmesh/vnavmesh.csproj -c Release -p:Platform=x64
```
Must succeed with zero warnings related to the changed files (unused `using`
warnings for `System.Collections.Generic` are fine only if Dictionary is no
longer used in the file — it is still used by `_openList` and `_nodes`, so the
using stays).

Static checks:
- `rg "RandomnessMultiplier|_rng|Random" vnavmesh/` must return no matches in
  `Config.cs` or `VoxelPathfind.cs`. (DotRecast matches are unrelated and
  expected.)
- `rg "_nodeLookup" vnavmesh/` must return no matches.
- `rg "Dictionary<ulong, int>" vnavmesh/` must return no matches.

Needs in-game verification (Phase 0):
- Fly a long path in a zone with a built volume (e.g. Ishgard, Sharlayan) via
  IPC `Nav.Pathfind` with `fly=true`. Confirm:
  1. Path completes (no `PathfindLoopException`, no "failed to find path" log).
  2. Repeated identical pathfind calls (same from/to) produce byte-identical
     waypoint lists — this verifies the random-factor removal. Compare via
     `Path.ListWaypoints` IPC twice.
  3. Long flying path (~200+ voxels) completes in materially less wall-time
     than pre-change (qualitative; check the `Pathfind took {t}s` debug log
     line in `[pathfind]` — enable Debug log level if needed). The flat-array
     lookup should show a visible drop on long paths.
- Ground path (IPC `Nav.Pathfind` with `fly=false`) must be unchanged —
  confirms no regression to the mesh pathfinder path.

### 0.5 Checkpoint: Verify Phase 0 complete

Review all tasks 0.1-0.4. Confirm every item is implemented, not stubbed, not
deferred. List each task and its status. Do not proceed to Phase 1 until the
build passes and the in-game verification list is confirmed (or explicitly
waived by the user with a note in the session handoff).

---

## Phase 1 — Volume string-pulling

The big quality win. Implement a funnel/string-pulling pass over the voxel
A* path so flying paths are not voxel-center staircases.

Do NOT conflate this with `BuildPathToVisitedNode`'s
`returnIntermediatePoints=true` mode (lines 107-140 in VoxelPathfind.cs).
That mode subdivides each A* hop by walking `EnumerateVoxelsInLine` and
emitting intermediate points — it produces *more* points, not fewer, and does
not straighten. String-pulling is a separate post-processing pass that takes
the *raw* A* voxel path (with `returnIntermediatePoints=false`) and collapses
collinear/LoS-clear runs into straight segments.

### 1.1 Implement `VoxelStraighten` (funnel over axis-aligned voxel faces)

File: `vnavmesh/NavVolume/VoxelStraighten.cs` (currently entirely commented out)

Uncomment and rewrite the class. The commented skeleton has the right idea
(face/edge/vertex transition cases) but is incomplete and references API that
doesn't exist (`_volume.IndexToVoxel`, `_volume.CellSize`, `_prevPlaneNormal`,
`_prevPlaneCenter`). Use the real `VoxelMap` API: `VoxelBounds(voxel, eps)`,
`Tile.VoxelToWorld`, `Level.CellSize`.

Recommended algorithm — simple and robust, preferred over the half-finished
projection-based funnel in the skeleton:

**Iterative line-of-sight string-pull** (simpler than a true funnel, robust on
axis-aligned voxel grids, and the existing `VoxelSearch.LineOfSight` already
exists and is correct):

1. Input: the raw A* path `List<(ulong voxel, Vector3 p)>` from
   `VoxelPathfind.FindPath(..., returnIntermediatePoints: false, ...)`,
   plus the true `toPos` endpoint.
2. Output: `List<Vector3>` simplified waypoints, starting at `fromPos` and
   ending at `toPos`.
3. Algorithm:
   - `result = [path[0].p]`
   - `anchor = 0` (index into input path)
   - `probe = anchor + 1`
   - While `probe < path.Count`:
     - If `VoxelSearch.LineOfSight(volume, path[anchor].voxel, path[probe].voxel, result.Last(), path[probe].p)`:
       - `probe++` (can skip this waypoint)
     - Else:
       - Commit `path[probe-1].p` to `result`.
       - `anchor = probe - 1`
       - `probe = anchor + 1`
   - Commit `path[^1].p` to `result`.
   - Append `toPos` as final waypoint.
4. Edge cases (each must be handled explicitly, not "etc."):
   - `path.Count <= 2`: return `[path[0].p, toPos]` (or `[path[0].p, path[^1].p, toPos]` if `path[^1].p != toPos`). No LoS loop needed.
   - `anchor == path.Count - 1`: loop exits, commit last.
   - LoS from `result.Last()` (not `path[anchor].p`) — important because the
     committed waypoint may be clamped to a voxel boundary and differ from
     the A* node position. Use `result.Last()` as the LoS `fromPos` and
     `path[anchor].voxel` as `fromVoxel` (re-find the voxel for
     `result.Last()` if it has drifted; in practice the committed point is
     inside `path[anchor]`'s voxel, so the voxel is still valid).
   - `EnumerateVoxelsInLine` can throw `PathfindLoopException` if the line
     re-enters the same voxel due to floating-point error at a boundary.
     `LineOfSight` calls it; wrap the LoS call in a try/catch that treats a
     `PathfindLoopException` as "no LoS" (commit the previous waypoint and
     continue). This prevents a single numerically-unstable edge from
     aborting the whole string-pull.
   - `toPos` may be inside the goal voxel but not at its center; the final
     appended `toPos` is the real destination and must be the last element.

Class shape:
```
public class VoxelStraighten
{
    private readonly VoxelMap _volume;
    public VoxelStraighten(VoxelMap volume) { _volume = volume; }
    public List<Vector3> Simplify(List<(ulong voxel, Vector3 p)> path, Vector3 toPos) { ... }
}
```
Use `System.Numerics.Vector3` and `System.Collections.Generic.List<>`. Add
`using` statements at the top of the file (the commented version had none
beyond the namespace). Do not add comments.

Prefer this LoS-based approach over the projection-funnel in the skeleton
because: (a) it reuses the tested `VoxelSearch.LineOfSight`, (b) it is
obviously correct (every emitted segment is LoS-verified by construction),
(c) the skeleton's funnel is half-written and would need a from-scratch
rewrite of the projection math anyway. The LoS approach produces slightly
more waypoints than an ideal funnel (it stops at the last voxel before LoS
breaks, not at the optimal face-crossing point), but the 5-10x reduction
target is still met because the dominant cost is removing every inter-voxel
hop along straight runs. If a later phase wants tighter waypoints, the
funnel can be added then; do not build it now.

### 1.2 Wire string-pulling into `PathfindVolume`

File: `vnavmesh/NavmeshQuery.cs`, method `PathfindVolume` (lines 136-166).

- Change the `FindPath` call at line 154 to pass `returnIntermediatePoints: false`
  (it already does — confirm the literal `false` is there). The raw A* path
  is the input to string-pulling.
- After the `voxelPath.Count == 0` check (lines 155-159), replace the
  `// TODO: string-pulling support` block (lines 162-165) with:
  ```
  if (useStringPulling)
  {
      var straighten = new VoxelStraighten(VolumeQuery.Volume);
      var simplified = straighten.Simplify(voxelPath, to);
      var res = simplified.Select(p => new Waypoint(p)).ToList();
      return res;
  }
  else
  {
      var res = voxelPath.Select(r => new Waypoint(r.p)).ToList();
      res.Add(new(to));
      return res;
  }
  ```
- This mirrors `PathfindMesh`'s `if (useStringPulling) { FindStraightPath... }`
  structure (lines 112-127).
- `useStringPulling` is already plumbed through from `NavmeshManager` (line
  150: `Query.PathfindVolume(from, to, UseRaycasts, UseStringPulling, ...)`),
  and `NavmeshManager.UseStringPulling` defaults to `true` (line 20). No new
  wiring needed.
- Keep the `res.Add(new(to))` only in the non-string-pulling branch; the
  string-pulling branch already appends `toPos` inside `Simplify`.

### 1.3 Verification (Phase 1)

Build: same command as Phase 0.

Static checks:
- `rg "TODO: string-pulling" vnavmesh/` must return no matches (the TODO at
  NavmeshQuery.cs:162 is removed).
- `VoxelStraighten.cs` has no commented-out code remaining (the whole class
  is now live).

Unit-style verification (in-game, via IPC):
- Call `Nav.Pathfind` with `fly=true` on a path with a long straight segment
  (e.g. across a large open field in Ishgard).
- Compare `Path.ListWaypoints` count before (Phase 0 build) and after
  (Phase 1 build) for the same from/to. Expect 5-10x fewer waypoints.
- Enable `ShowWaypoints` config (or `Service.Config.ShowWaypoints = true`
  via the Config tab) and visually confirm the waypoint line is smooth
  (no 90-degree kinks at every voxel face).

Correctness verification (in-game):
- For each segment of the simplified path, verify LoS holds: this is true by
  construction in `Simplify`, but confirm in-game that the player does not
  clip through geometry when following the path with `Path.MoveTo`. Test in
  a zone with obstacles (e.g. pillars in a city) where a naive straight line
  would clip.
- Confirm the final waypoint equals the requested `to` (within float
  epsilon). Use `Path.ListWaypoints` and compare last element to the input.

Needs in-game verification (Phase 1):
- IPC `Nav.Pathfind` with `fly=true`: waypoint count reduction, visual
  smoothness, no geometry clipping on the simplified path.
- IPC `Nav.Pathfind` with `fly=false`: unchanged (string-pulling only
  touches the volume path; mesh pathfinder already had its own
  `FindStraightPath`).
- `Path.MoveTo` with a fly path: movement completes to destination without
  getting stuck on geometry that the old staircase path avoided by detour
  but the new straight path cuts through. If this happens, the LoS check in
  `Simplify` is the safeguard — a clip means `LineOfSight` has a bug, not
  that string-pulling is wrong. Debug via the `PathfindLoopException`
  catch (log when it fires).

### 1.4 Checkpoint: Verify Phase 1 complete

Review tasks 1.1-1.3. Confirm `VoxelStraighten` is fully implemented (no
commented-out remnants), wired into `PathfindVolume`, and the in-game
verification list is confirmed. Do not proceed to Phase 2 until the build
passes and waypoint-count reduction is observed in-game.

---

## Phase 2 — DotRecast submodule rebase

Rebase the DotRecast submodule from the stale `xanunderscore/DotRecast` fork
(13 fork-specific commits, 304 commits behind upstream) to upstream
`ikpil/DotRecast` main. Re-apply the 2 useful fork patches cleanly. Drop the
WIP/experiment commits.

Submodule path: `/home/edgar/dev/ffxiv_navmesh/DotRecast/`
Parent submodule pointer (as of HEAD): `8002b30b9f196bcd9eb0a898e51abcea4177fb04`
Remotes (already configured in submodule):
- `origin` = `git@github.com:edg-l/DotRecast.git` (user's fork, push here)
- `upstream` = `https://github.com/ikpil/DotRecast.git`
- `upstream-xan` = `https://github.com/xanunderscore/DotRecast` (old fork base)

Fork base (merge-base of upstream-xan/main and upstream/main):
`4b8cd8e31b1574ee1567a49e413b4f979c7def88` — this is on upstream/main, so
cherry-picking the fork commits onto current upstream/main is a clean
rebase.

Fork-specific commits to evaluate (from `git log 4b8cd8e..upstream-xan/main`):
- `91880dd` "Optimization (breaking): pool RcSpans." — KEEP (re-apply, marked
  breaking so it needs a build+verify).
- `abff290` "add early exit support to pathfind" — KEEP, but fix the double
  `if (heuristicCost < 0)` copy-paste bug (inner `if` is redundant; collapse
  to a single `if`).
- `f60750c` "Optimization: reduce number of allocations on hot path." — 1-line
  change; evaluate during rebase. The IDEAS.md plan says keep the 2 useful
  patches (span pooling, early exit). This one is not in the keep list. DROP
  unless the build fails without it (it's a minor alloc reduction and
  upstream has 304 commits of its own allocation work).
- `60996470`, `de3af0fc`, `992c5e27`, `875a84e1`, `60843e71`, `c42d12f2`,
  `4b20f884`, `523adbd5`, `e1015060`, `8002b30b` — all WIP/experiments/fixes
  for the WIP work. DROP.

### 2.1 Fetch upstream and create the rebase branch

Workdir: `DotRecast/` (the submodule).

- `git fetch upstream`
- `git checkout -b rebase-to-upstream upstream/main`
- Confirm `git log --oneline -1` shows the upstream/main tip (currently
  `b18da312` or newer after fetch).

### 2.2 Cherry-pick span pooling (91880dd)

- `git cherry-pick 91880dd`
- This commit touches `RcCompacts.cs`, `RcFilters.cs`, `RcHeightfield.cs`,
  `RcRasterizations.cs`, `RcSpan.cs` (5 files, +105/-46). It is marked
  "breaking" — expect possible conflicts because upstream has 304 commits of
  its own changes to these files.
- If conflict: resolve by preferring the *upstream* version of the
  surrounding code and re-applying only the pooling semantics (span free-list
  in `RcHeightfield`, `RcSpan` pool acquire/release in `RcRasterizations`).
  Do not blindly take either side; the pooling must sit on top of current
  upstream APIs. If the pooling API is incompatible with current upstream
  (e.g. upstream already pooled spans differently), STOP and surface to the
  user — do not force a broken cherry-pick.
- Build the submodule's Recast project to confirm the cherry-pick compiles:
  `dotnet build DotRecast/src/DotRecast.Recast/DotRecast.Recast.csproj -c Release`
  from the submodule dir.

### 2.3 Cherry-pick early exit (abff290) and fix the double-if bug

- `git cherry-pick abff290`
- This touches only `DtNavMeshQuery.cs` (+17/-6). Lower conflict risk.
- After cherry-pick, open `src/DotRecast.Detour/DtNavMeshQuery.cs` and find
  the added block. It currently reads:
  ```
  if (heuristicCost < 0)
  {
      // if cost is negative, ...
      if (heuristicCost < 0)
      {
          lastBestNode = bestNode;
          goto break_outer;
      }
  }
  ```
  Collapse the redundant nested `if` to:
  ```
  if (heuristicCost < 0)
  {
      lastBestNode = bestNode;
      goto break_outer;
  }
  ```
  Keep the `break_outer:` label that the original commit added at the end of
  the outer loop. Do not add a comment.
- Build: `dotnet build DotRecast/src/DotRecast.Detour/DotRecast.Detour.csproj -c Release`

### 2.4 Push the rebased branch to the user's fork

- Confirm branch name with the user before pushing (per AGENTS.md: never
  assume branch naming conventions; recommend `rebase-to-upstream`).
- `git push -u origin rebase-to-upstream`
- Do NOT force-push. Do NOT push to `main` on the fork. Do NOT delete any
  existing branches without asking.

### 2.5 Bump the submodule pointer in the parent repo

Workdir: `/home/edgar/dev/ffxiv_navmesh/` (parent).

- `cd /home/edgar/dev/ffxiv_navmesh` (use workdir param, not `cd` chain)
- In the submodule: `git checkout rebase-to-upstream` (so the submodule
  working tree matches the pointer we're about to set).
- In the parent: `git add DotRecast` — this stages the new submodule SHA.
- Do NOT commit the parent repo. The user's AGENTS.md says never commit
  unless asked. Leave the staged change in the index for the user to review
  and commit themselves. Note in the session handoff that the submodule
  pointer is staged and ready to commit on user confirmation.

### 2.6 Build the full plugin with the rebased submodule

```
DALAMUD_HOME=~/.cache/dalamud-dev DOTNET_ROOT=~/.dotnet dotnet build vnavmesh/vnavmesh.csproj -c Release -p:Platform=x64
```
This is the critical verification that span pooling (breaking) and the early-
exit patch do not break the plugin's use of DotRecast. The plugin references
`DotRecast.Detour`, `DotRecast.Core.Numerics`, `DotRecast.Recast` (see
`NavmeshQuery.cs` imports and `NavmeshBuilder.cs`). Any API breakage from
upstream's 304 commits will surface here.

If the build fails due to upstream API changes (renamed methods, moved
types, changed signatures):
- Fix the plugin's call sites in `vnavmesh/` to match the new upstream API.
  Common expected churn: `RcVec3f` API, `DtFindPathOption` constructor,
  `DtStatus` helpers. Read the upstream changelog
  (`DotRecast/CHANGELOG.md`) for the version jumped to.
- Each fix is its own edit; do not batch blindly. Prefer minimal call-site
  updates over restructuring plugin code.
- This is the riskiest task in the whole plan. If the breakage is large
  (>10 call sites need rewriting), STOP and surface to the user with a list
  of required plugin-side changes for approval before continuing.

### 2.7 Verification (Phase 2)

Build: step 2.6 above must pass.

Submodule state checks:
- `cd DotRecast && git log --oneline -5` shows the upstream/main tip plus
  exactly 2 cherry-picked commits (span pooling, early exit) on top. No WIP
  commits, no `8002b30` "whoops" commit.
- `cd DotRecast && git diff upstream/main HEAD --stat` shows only the files
  touched by the 2 kept patches (5 Recast files + 1 Detour file).
- Parent repo `git diff --cached DotRecast` shows the new submodule SHA.

Needs in-game verification (Phase 2):
- Load the plugin in-game. Confirm the plugin loads without exception
  (DTR bar appears if `EnableDTR` is on; `/vnavmesh` window opens).
- Enter a zone with a built ground navmesh (e.g. a city). Call
  `Nav.Pathfind` with `fly=false`. Confirm the ground path is produced and
  `Path.MoveTo` follows it to completion. This exercises Detour's
  `FindPath` + `FindStraightPath` with the early-exit patch applied.
- Enter a zone with a built volume (e.g. Ishgard). Call `Nav.Pathfind` with
  `fly=true`. Confirm the volume path still works (the early-exit patch
  only affects Detour/mesh, but confirm no regression).
- Test a path that triggers `GoalRadiusHeuristic` (IPC
  `Nav.PathfindWithTolerance` with a non-zero `range`) with `fly=false`.
  This is the path that exercises the early-exit `goto break_outer` —
  confirm it returns a partial path ending within `range` of the goal,
  marked `DT_PARTIAL_RESULT`, and the plugin treats it as a valid path
  (does not log "failed to find path").
- Span pooling (breaking): rebuild a navmesh from scratch in a zone (delete
  the cached mesh file, zone in, let it auto-build with
  `AutoLoadNavmesh=true` or trigger `Nav.Rebuild`). Confirm the build
  completes without crash or OOM. Span pooling changes the rasterization
  hot path; a crash here means the cherry-pick conflicted silently.

### 2.8 Checkpoint: Verify Phase 2 complete

Review tasks 2.1-2.7. Confirm the submodule is on the rebased branch with
exactly 2 cherry-picks, the full plugin builds, and the in-game verification
list (ground path, fly path, GoalRadiusHeuristic path, fresh mesh build) is
confirmed. The submodule pointer is staged in the parent repo but NOT
committed — flag this to the user.

### 2.9 Final Audit

Re-read the entire plan. For each task (0.1 through 2.7), verify the
implementation exists in the codebase:
- `VoxelPathfind.cs`: no `_rng`, no `randomFactor`, no `_nodeLookup`, has
  `_lookupTable`/`LookupGet`/`LookupSet`/`LookupGrow`/`VoxelHash`.
- `Config.cs`: no `RandomnessMultiplier` field, no slider in `Draw`.
- `VoxelStraighten.cs`: live `VoxelStraighten` class with `Simplify` method.
- `NavmeshQuery.cs:PathfindVolume`: calls `VoxelStraighten.Simplify` when
  `useStringPulling`, no `// TODO: string-pulling support` line.
- `DotRecast/`: on `rebase-to-upstream` branch, 2 cherry-picks on
  upstream/main, `DtNavMeshQuery.cs` has the single-`if` early exit (not
  double), no WIP commits in `git log --oneline 4b8cd8e..HEAD`.
- Parent repo: `DotRecast` submodule pointer staged but uncommitted.

List any gaps. All gaps must be resolved before reporting completion. Flag
to the user: (a) the staged submodule pointer needs their commit, (b) the
`rebase-to-upstream` branch needs their decision on whether to merge to
`main` on their fork.

---

## Open Questions

- Phase 0.3: is a default lookup capacity of 4096 acceptable, or should it
  derive from the volume's leaf voxel count? (Current plan: start at 4096,
  grow on demand. This is fine; the growth path handles the long-path case.)
- Phase 1.1: the LoS-based string-pull stops at the last voxel before LoS
  breaks, not at the optimal face-crossing point. This produces slightly
  more waypoints than a true funnel. Acceptable for the 5-10x reduction
  target, but if the user wants tighter, a projection-funnel can be a
  follow-up. Confirm the LoS approach is acceptable before implementing.
- Phase 2.3: the early-exit patch's `goto break_outer` uses a C# label.
  Confirm the upstream `DtNavMeshQuery.cs` loop structure still supports
  this label placement (the cherry-pick may land in a refactored loop). If
  the loop was refactored, the `goto` target may need adjustment.
- Phase 2.6: if upstream API churn breaks >10 plugin call sites, the user
  must approve the plugin-side fix scope before continuing. Threshold of
  10 is a guess; confirm acceptable.
