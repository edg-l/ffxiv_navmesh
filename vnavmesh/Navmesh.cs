using Navmesh.GroundGraph;
using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Numerics;
using System.Runtime.CompilerServices;

[assembly: InternalsVisibleTo("vnavmesh.Tests")]

namespace Navmesh;

// full set of data needed for navigation in the zone
public record class Navmesh(int CustomizationVersion, QuadGraph? Ground, VoxelMap? Volume)
{
	public static readonly uint Magic = 0x444D564E; // 'NVMD'
	public static readonly uint Version = 27;
	public const int FLAG_UNREACHABLE = 0x10;

	[Flags]
	public enum AreaId
	{
		None = 0,
		Warp = 0x01, // direct teleportation, i.e. aetheryte (not implemented)
		ClientPath = 0x02, // predefined path activated by walking into a triggerbox (e.g. cosmoliner, some transitions in dungeons, etc)
		Shortcut = 0x04, // regular shortcut followed at normal movement speed, faster due to a shorter overall path (e.g. dropping down from a ledge or walking through a gap that recast thinks is too narrow)

		Endpoint = 0x10, // these need to be marked for FollowPath logic and heuristic purposes
		ClientPathEnd = ClientPath | Endpoint,

		Default = 0x3F
	}

	// throws an exception on failure
	public static Navmesh Deserialize(BinaryReader reader, int expectedCustomizationVersion)
	{
		var magic = reader.ReadUInt32();
		var version = reader.ReadUInt32();
		if (magic != Magic || version != Version)
			throw new Exception("Incorrect header");
		var customizationVersion = reader.ReadInt32();
		if (customizationVersion != expectedCustomizationVersion)
			throw new Exception("Outdated customization version");

		using var compressedReader = new BinaryReader(new BrotliStream(reader.BaseStream, CompressionMode.Decompress, true));
		var ground = DeserializeGround(compressedReader);
		var volume = DeserializeVolume(compressedReader);
		return new(customizationVersion, ground, volume);
	}

	public void Serialize(BinaryWriter writer)
	{
		writer.Write(Magic);
		writer.Write(Version);
		writer.Write(CustomizationVersion);

		using var compressedWriter = new BinaryWriter(new BrotliStream(writer.BaseStream, CompressionLevel.Optimal, true));
		SerializeGround(compressedWriter, Ground);
		SerializeVolume(compressedWriter, Volume);
	}

	private static VoxelMap? DeserializeVolume(BinaryReader reader)
	{
		var numLevels = reader.ReadInt32();
		if (numLevels == 0)
			return null;

		var tilesPerLevel = new int[numLevels];
		foreach (ref var l in tilesPerLevel.AsSpan())
			l = reader.ReadInt32();
		var (min, max) = DeserializeBounds(reader);
		var volume = new VoxelMap(min, max, tilesPerLevel);
		DeserializeVolumeTile(reader, volume.RootTile);
		return volume;
	}

	private static void SerializeVolume(BinaryWriter writer, VoxelMap? volume)
	{
		if (volume == null)
		{
			writer.Write(0); // 0 levels;
			return;
		}

		writer.Write(volume.Levels.Length);
		foreach (ref var l in volume.Levels.AsSpan())
			writer.Write(l.NumCellsX); // note: current assumption is that all dimensions are identical

		SerializeBounds(writer, volume.RootTile.BoundsMin, volume.RootTile.BoundsMax);
		SerializeVolumeTile(writer, volume.RootTile);
	}

	private static QuadGraph? DeserializeGround(BinaryReader reader)
	{
		try
		{
			var hasGround = reader.ReadBoolean();
			if (!hasGround)
				return null;

			var boundsMin = DeserializeVector3(reader);
			var boundsMax = DeserializeVector3(reader);
			var graph = new QuadGraph(boundsMin, boundsMax);
			graph.MaxClimb = reader.ReadSingle();

			var quadCount = reader.ReadInt32();
			for (int i = 0; i < quadCount; ++i)
			{
				var q = new Quad(
					reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(),
					reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(),
					(Navmesh.AreaId)reader.ReadByte());
				graph.AddQuad(q);
			}

			var portalCount = reader.ReadInt32();
			for (int i = 0; i < portalCount; ++i)
			{
				var portal = new Portal(
					reader.ReadInt32(), reader.ReadInt32(),
					new Vector2(reader.ReadSingle(), reader.ReadSingle()),
					new Vector2(reader.ReadSingle(), reader.ReadSingle()),
					reader.ReadSingle(), reader.ReadSingle(),
					reader.ReadBoolean(), (Navmesh.AreaId)reader.ReadByte());
				graph.Portals.Add(portal);
			}

			var flagsCount = reader.ReadInt32();
			graph.Flags = new int[flagsCount];
			for (int i = 0; i < flagsCount; ++i)
				graph.Flags[i] = reader.ReadInt32();

			return graph;
		}
		catch (EndOfStreamException)
		{
			return null;
		}
	}

	internal static void SerializeGround(BinaryWriter writer, QuadGraph? graph)
	{
		if (graph == null)
		{
			writer.Write(false);
			return;
		}
		writer.Write(true);
		SerializeVector3(writer, graph.BoundsMin);
		SerializeVector3(writer, graph.BoundsMax);
		writer.Write(graph.MaxClimb);

		writer.Write(graph.Quads.Count);
		foreach (var q in graph.Quads)
		{
			writer.Write(q.MinX); writer.Write(q.MinY); writer.Write(q.MinZ);
			writer.Write(q.MaxX); writer.Write(q.MaxY); writer.Write(q.MaxZ);
			writer.Write((byte)q.Area);
		}

		writer.Write(graph.Portals.Count);
		foreach (var p in graph.Portals)
		{
			writer.Write(p.FromQuad); writer.Write(p.ToQuad);
			writer.Write(p.SpanMin.X); writer.Write(p.SpanMin.Y);
			writer.Write(p.SpanMax.X); writer.Write(p.SpanMax.Y);
			writer.Write(p.YFrom); writer.Write(p.YTo);
			writer.Write(p.IsOffMesh); writer.Write((byte)p.Area);
		}

		writer.Write(graph.Flags.Length);
		foreach (var f in graph.Flags)
			writer.Write(f);
	}

	private static unsafe void DeserializeVolumeTile(BinaryReader reader, VoxelMap.Tile tile)
	{
		for (int i = 0; i < tile.Contents.Length; ++i)
		{
			var v = tile.Contents[i] = reader.ReadUInt16();
			if (v == 0 || v == ushort.MaxValue)
			{
				var run = reader.ReadUInt16();
				while (run-- != 0)
					tile.Contents[++i] = v;
			}
		}

		var numSubtiles = reader.ReadInt32();
		for (int i = 0; i < numSubtiles; ++i)
		{
			var subBounds = DeserializeBounds(reader);
			var subTile = new VoxelMap.Tile(tile.Owner, subBounds.min, subBounds.max, tile.Level + 1);
			DeserializeVolumeTile(reader, subTile);
			tile.Subdivision.Add(subTile);
		}
	}

	private static unsafe void SerializeVolumeTile(BinaryWriter writer, VoxelMap.Tile tile)
	{
		// use simple run-length encoding for fully empty / fully solid tiles
		for (int i = 0; i < tile.Contents.Length; ++i)
		{
			var v = tile.Contents[i];
			writer.Write(v);
			if (v == 0 || v == ushort.MaxValue)
			{
				ushort run = 0;
				while (i + 1 < tile.Contents.Length && tile.Contents[i + 1] == v)
				{
					++run;
					++i;
				}
				writer.Write(run);
			}
		}

		writer.Write(tile.Subdivision.Count);
		foreach (var sub in tile.Subdivision)
		{
			SerializeBounds(writer, sub.BoundsMin, sub.BoundsMax);
			SerializeVolumeTile(writer, sub);
		}
	}

	private static (Vector3 min, Vector3 max) DeserializeBounds(BinaryReader reader) => (DeserializeVector3(reader), DeserializeVector3(reader));
	private static void SerializeBounds(BinaryWriter writer, Vector3 min, Vector3 max)
	{
		SerializeVector3(writer, min);
		SerializeVector3(writer, max);
	}

	private static Vector3 DeserializeVector3(BinaryReader reader) => new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
	private static void SerializeVector3(BinaryWriter writer, Vector3 v)
	{
		writer.Write(v.X);
		writer.Write(v.Y);
		writer.Write(v.Z);
	}
}