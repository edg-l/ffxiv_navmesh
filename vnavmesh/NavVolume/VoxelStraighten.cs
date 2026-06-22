using System;
using System.Collections.Generic;
using System.Numerics;

namespace Navmesh.NavVolume;

public class VoxelStraighten
{
    private readonly VoxelMap _volume;

    public VoxelStraighten(VoxelMap volume)
    {
        _volume = volume;
    }

    public List<Vector3> Simplify(List<(ulong voxel, Vector3 p)> path, Vector3 toPos)
    {
        var result = new List<Vector3>();
        if (path.Count == 0)
        {
            result.Add(toPos);
            return result;
        }

        result.Add(path[0].p);

        if (path.Count <= 2)
        {
            if (path.Count == 2 && (path[1].p - toPos).LengthSquared() > 1e-6f)
                result.Add(path[1].p);
            result.Add(toPos);
            return result;
        }

        int anchor = 0;
        int probe = anchor + 1;
        while (probe < path.Count)
        {
            var fromPos = result[^1];
            if (HasLineOfSight(path[anchor].voxel, path[probe].voxel, fromPos, path[probe].p))
            {
                probe++;
            }
            else
            {
                result.Add(path[probe - 1].p);
                anchor = probe - 1;
                probe = anchor + 1;
            }
        }

        result.Add(path[^1].p);
        result.Add(toPos);
        return result;
    }

    private bool HasLineOfSight(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        try
        {
            return VoxelSearch.LineOfSight(_volume, fromVoxel, toVoxel, fromPos, toPos);
        }
        catch (PathfindLoopException)
        {
            return false;
        }
    }
}