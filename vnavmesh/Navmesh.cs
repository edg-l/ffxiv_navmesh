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
	public static readonly uint Version = 29;
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

			// Phase 4: optional prebuilt CDT triangle PolyMesh (Config.UseCdtMesh).
			var hasCdtMesh = reader.ReadBoolean();
			if (hasCdtMesh)
			{
				var mesh = DeserializeCdtMesh(reader);
				// Rebuild the face-AABB wrapper (quads/adjacency) deterministically
				// from the mesh; the persisted Flags above stay authoritative.
				var savedFlags = graph.Flags;
				graph.SetCdtMesh(mesh);
				graph.Flags = savedFlags;
			}

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

		// Phase 4: optional prebuilt CDT triangle PolyMesh (Config.UseCdtMesh).
		if (graph.PrebuiltMesh == null)
		{
			writer.Write(false);
		}
		else
		{
			writer.Write(true);
			SerializeCdtMesh(writer, graph.PrebuiltMesh);
		}
	}

	private static void SerializeCdtMesh(BinaryWriter writer, GroundGraph.Polyanya.PolyMesh mesh)
	{
		writer.Write(mesh.Vertices.Count);
		foreach (var v in mesh.Vertices)
			SerializeVector3(writer, v);

		writer.Write(mesh.Faces.Count);
		foreach (var f in mesh.Faces)
		{
			writer.Write(f.V0); writer.Write(f.V1); writer.Write(f.V2);
			writer.Write(f.Y); writer.Write(f.Layer);
		}
		// SourceQuad per face.
		foreach (var sq in mesh.SourceQuad)
			writer.Write(sq);

		// Edges: 3 per face, in face*3+edge order.
		writer.Write(mesh.Edges.Count);
		foreach (var e in mesh.Edges)
		{
			writer.Write(e.FaceLeft); writer.Write(e.FaceRight); writer.Write(e.IsObstacleEdge);
		}

		writer.Write(mesh.OffMeshLinks.Count);
		foreach (var l in mesh.OffMeshLinks)
		{
			writer.Write(l.FromFace); writer.Write(l.ToFace);
			SerializeVector3(writer, l.FromPos);
			SerializeVector3(writer, l.ToPos);
			writer.Write((byte)l.Area);
		}
	}

	private static GroundGraph.Polyanya.PolyMesh DeserializeCdtMesh(BinaryReader reader)
	{
		var mesh = new GroundGraph.Polyanya.PolyMesh();

		var vertCount = reader.ReadInt32();
		for (int i = 0; i < vertCount; ++i)
			mesh.Vertices.Add(DeserializeVector3(reader));

		var faceCount = reader.ReadInt32();
		for (int i = 0; i < faceCount; ++i)
		{
			int v0 = reader.ReadInt32(), v1 = reader.ReadInt32(), v2 = reader.ReadInt32();
			float y = reader.ReadSingle();
			int layer = reader.ReadInt32();
			mesh.Faces.Add(new GroundGraph.Polyanya.TriFace(v0, v1, v2, y, layer));
			mesh.SourceQuad.Add(-1);
		}
		for (int i = 0; i < faceCount; ++i)
			mesh.SourceQuad[i] = reader.ReadInt32();

		var edgeCount = reader.ReadInt32();
		for (int i = 0; i < edgeCount; ++i)
		{
			int fl = reader.ReadInt32(), fr = reader.ReadInt32();
			bool obs = reader.ReadBoolean();
			mesh.Edges.Add(new GroundGraph.Polyanya.TriEdge(fl, fr, obs));
		}

		var linkCount = reader.ReadInt32();
		for (int i = 0; i < linkCount; ++i)
		{
			int from = reader.ReadInt32(), to = reader.ReadInt32();
			var fromPos = DeserializeVector3(reader);
			var toPos = DeserializeVector3(reader);
			var area = (AreaId)reader.ReadByte();
			mesh.OffMeshLinks.Add(new GroundGraph.Polyanya.OffMeshLink(from, to, fromPos, toPos, area));
		}

		// Rebuild the face-id FacesByQuad index (identity) so PolyanyaSearch can
		// resolve start/goal faces; SetCdtMesh on the graph rebuilds quads/adjacency.
		var facesByQuad = new System.Collections.Generic.List<int>[mesh.Faces.Count];
		for (int f = 0; f < mesh.Faces.Count; ++f)
			facesByQuad[f] = new System.Collections.Generic.List<int> { f };
		mesh.FacesByQuad = facesByQuad;

		return mesh;
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