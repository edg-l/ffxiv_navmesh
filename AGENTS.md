# vnavmesh fork — Agent Guide

This is a fork of [awgil/ffxiv_navmesh](https://github.com/awgil/ffxiv_navmesh)
that targets pathfinding algorithm and flight-path quality improvements. The
goal is to keep the IPC API **drop-in compatible** with upstream vnavmesh so
existing consumers (SealHunter, Hunty, Lifestream, AutoDuty, Questionable,
Saucy, Henchman, etc.) work unchanged.

See `IDEAS.md` for the analysis and improvement ideas, and `PLAN.md` for the
phased implementation plan (Phase 0: quick volume wins, Phase 1: volume
string-pulling, Phase 2: DotRecast submodule rebase).

## Remotes

- `origin` → `git@github.com:edg-l/ffxiv_navmesh.git` (this fork, SSH)
- `upstream` → `https://github.com/awgil/ffxiv_navmesh.git` (original)

The DotRecast submodule under `DotRecast/` has its own remotes:
- `origin` → `git@github.com:edg-l/DotRecast.git` (our fork, SSH)
- `upstream` → `https://github.com/ikpil/DotRecast.git` (actively maintained source)
- `upstream-xan` → `https://github.com/xanunderscore/DotRecast` (old fork base, reference only)

After checking out this repo, init the submodule:
```
git submodule update --init --recursive
```

## Build

```bash
DALAMUD_HOME=~/.cache/dalamud-dev DOTNET_ROOT=~/.dotnet \
  dotnet build vnavmesh/vnavmesh.csproj -c Release -p:Platform=x64
```

Target the `.csproj` directly. SDK-15 `dotnet new sln` produces a `.slnx`
that may lack a `Release|x64` mapping, so `dotnet build` at repo root can fail
with MSB4126. Output: `vnavmesh/bin/x64/Release/vnavmesh.dll` and
`…/vnavmesh/latest.zip` (+ packaged manifest).

There is no unit-test project. The clean build is the gate. Keep 0 warnings
and 0 errors. Most behavior can only be verified in-game.

## Deploy

Use the `publish.sh` wrapper, which delegates to the global
`~/.local/bin/publish-plugin` script:
```bash
./publish.sh
```
This builds Release, stages `latest.zip` / dll / manifest under
`~/share/zhyra/vnavmesh/`, and merges the entry into the combined
`~/share/zhyra/pluginmaster.json` (the one shared repo for all of Edgar's
plugins). Do not write a per-plugin overwrite-style publish script — it would
clobber sibling plugins in the combined master.

Always bump `<Version>` in `vnavmesh/vnavmesh.json` before publishing.
Dalamud uses `<Version>` to detect updates; republishing the same version
means the in-game client won't pick up the new DLL.

## Drop-in IPC compatibility

Upstream vnavmesh exposes IPC under the `vnavmesh.*` namespace
(see `vnavmesh/IPCProvider.cs`). All registered methods must keep their
names, argument types, and return types stable across this fork. New IPC
methods can be added; existing ones must not be renamed, repurposed, or have
their signatures changed. Consumers depend on:
- `Nav.IsReady`, `Nav.BuildProgress`, `Nav.Reload`, `Nav.Rebuild`
- `Nav.Pathfind`, `Nav.PathfindWithTolerance`, `Nav.PathfindCancelable`,
  `Nav.PathfindCancelAll`, `Nav.PathfindInProgress`, `Nav.PathfindNumQueued`
- `Query.Mesh.NearestPoint`, `Query.Mesh.NearestPointReachable`,
  `Query.Mesh.PointOnFloor`, `Query.Mesh.FlagToPoint`
- `Path.MoveTo`, `Path.Stop`, `Path.IsRunning`, `Path.NumWaypoints`,
  `Path.ListWaypoints`, `Path.GetMovementAllowed`, `Path.SetMovementAllowed`,
  `Path.GetAlignCamera`, `Path.SetAlignCamera`, `Path.GetTolerance`,
  `Path.SetTolerance`
- `SimpleMove.PathfindAndMoveTo`, `SimpleMove.PathfindAndMoveCloseTo`,
  `SimpleMove.PathfindInProgress`
- `Window.IsOpen`, `Window.SetOpen`, `DTR.IsShown`, `DTR.SetShown`

If a behavior change is unavoidable (e.g. a return value now carries fewer
waypoints after string-pulling), keep the type and namespace identical and
document the semantic change in the commit message.

## Architecture

Two pathfinding modes:

- **2D ground mesh** — `NavmeshQuery.PathfindMesh` delegates to DotRecast
  Detour A* + optional string-pulling via `FindStraightPath`. The Detour code
  lives in the `DotRecast/` submodule.
- **3D voxel volume** — `NavmeshQuery.PathfindVolume` uses the hand-written
  `VoxelPathfind` A* over a 3-level hierarchical voxel octree
  (`VoxelMap` / `Voxelizer` / `VoxelSearch`). This is where the biggest
  improvement opportunities are (see `IDEAS.md`). String-pulling for volume
  paths is unimplemented (`VoxelStraighten.cs` is a commented-out skeleton,
  `NavmeshQuery.PathfindVolume` has `// TODO: string-pulling support`).

Movement consumption: `FollowPath.Update` runs every framework tick, advancing
through waypoints via `OverrideMovement` (hooks `RMIWalk`/`RMIFly`) and
`OverrideCamera`. Stuck detection compares per-frame delta to a tolerance and
fires `OnStuck` to the caller on timeout.

`NavmeshManager` owns navmesh lifecycle: loads/builds per-territory meshes,
caches them under `_cacheDir`, runs pathfinding as a single serialized task
chain (`_lastLoadQueryTask`), and prunes unreachable polys via a flood fill
from known aetheryte positions.

## Key files

- `vnavmesh/IPCProvider.cs` — the full IPC surface (do not break)
- `vnavmesh/NavmeshQuery.cs` — entry points `PathfindMesh` / `PathfindVolume`
- `vnavmesh/NavmeshManager.cs` — lifecycle, caching, async task chain
- `vnavmesh/NavVolume/VoxelPathfind.cs` — the A* to improve (Phase 0 + 1)
- `vnavmesh/NavVolume/VoxelMap.cs` — packed hierarchical voxel encoding
  (`EncodeIndex` / `DecodeIndex` / `IndexLevelShift` / `IndexLevelMask`)
- `vnavmesh/NavVolume/VoxelStraighten.cs` — string-pulling skeleton (Phase 1)
- `vnavmesh/NavVolume/VoxelSearch.cs` — `FindClosestVoxelPoint`,
  `EnumerateVoxelsInLine`, `LineOfSight`
- `vnavmesh/Movement/FollowPath.cs` — waypoint consumption + stuck detection
- `vnavmesh/Movement/OverrideMovement.cs` — input hooks
- `vnavmesh/Config.cs` — plugin config (some fields may be removed by the
  plan, e.g. `RandomnessMultiplier`)
- `TODO` — the original author's TODO file (cross-check against `IDEAS.md`)

## In-game verification

Most changes can only be verified in-game. Keep an explicit
"needs in-game verification" list per change:
- IPC method names still resolve from consumer plugins
- struct offsets / signature scans still valid after game patches
- coordinate math correct (map coords vs world coords)
- ground pathfinding still reaches destinations
- flying pathfinding doesn't clip through geometry
- string-pulled paths have visibly fewer waypoints and no staircase kinks

## Conventions

- C# / Dalamud SDK 15 (`Dalamud.NET.Sdk/15.0.0`), targets `net10.0`, `x64`.
- ImGui namespace is `Dalamud.Bindings.ImGui` (not `ImGuiNET`).
- Bundled data files use `<EmbeddedResource>`, not `<None CopyToOutputDirectory>`.
- FFXIVClientStructs: `unsafe`, singleton-accessed, namespaces differ per
  manager (`FFXIV.Client.Game` vs `…Game.UI` vs `…Client.UI.Agent`).
- Map coords are not world coords; use `MapLinkPayload` then floor-snap Y.
- This plugin automates gameplay against the ToS; ship a clear disclaimer.