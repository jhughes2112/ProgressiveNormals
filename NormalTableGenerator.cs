using System;
using System.Collections.Generic;
using System.IO;
using Vec3 = System.Numerics.Vector3;
using ProgressiveNormals;

// This is just tools-time code to generate a table of 32k normals.  Probably don't ever need it again, but checked in just in case.
public class NormalTableGenerator
{
	// Produce a huge ordered set of normals so that the farther down the table you are able to index, the more normals fill in the gaps left by the previous ones.
	public void GenerateNormalTable(int total, Action<string> log)
	{
		// This is the set of normals we want to consider, but want to order them so they are progressive.
		List<Vec3> samples = new List<Vec3>();
		float increment = MathF.PI * (3 - MathF.Sqrt(5));
		int kSamples = 32768;  // 15 bits worth of index is more than enough
		for (int i = 0; i < kSamples; i++)
		{
			float y = 1 - (i / (float)(kSamples - 1)) * 2;
			float radius = MathF.Sqrt(1 - y * y);
			float phi = increment * i;
			float x = MathF.Cos(phi) * radius;
			float z = MathF.Sin(phi) * radius;
			samples.Add(new Vec3(x, y, z));
		}

		// Phase 1: pump all samples into the tree to build the centroid skeleton, then Rebuild once.
		// Rebuild freezes the structure: it computes centroids/radii, builds the grid, then clears all
		// leaf points.  The result is a spatial index with no data — ready to efficiently route queries.
		int tableCount = 0;
		Vec3[] normalTable = new Vec3[total];
		SpatialLookup chosenTree = new SpatialLookup();
		for (int i = 0; i < samples.Count; i++)
			chosenTree.AddToSet(samples[i], i);
		chosenTree.Rebuild(freeze: true);

		// Phase 2: seed with the cardinal directions.  AddToSet on a frozen tree routes each point
		// to the nearest leaf without touching any centroids.
		AddToTableAndTree(normalTable, chosenTree, new Vec3(1,0,0), tableCount++);
		AddToTableAndTree(normalTable, chosenTree, new Vec3(0,1,0), tableCount++);
		AddToTableAndTree(normalTable, chosenTree, new Vec3(0,0,1), tableCount++);
		AddToTableAndTree(normalTable, chosenTree, new Vec3(-1,0,0), tableCount++);
		AddToTableAndTree(normalTable, chosenTree, new Vec3(0,-1,0), tableCount++);
		AddToTableAndTree(normalTable, chosenTree, new Vec3(0,0,-1), tableCount++);

		// Phase 3: initialize minDistSq for every candidate using the grid fast-path.
		float[] minDistSq = new float[samples.Count];
		for (int j = 0; j < samples.Count; j++)
		{
			Vec3 sample = samples[j];
			float d = float.MaxValue;
			int dummy = -1;
			chosenTree.FindClosestVec3(ref sample, ref d, ref dummy);
			minDistSq[j] = d;
		}

		// Progressively pick the candidate farthest from all already-chosen normals.
		// When a new normal is chosen, update minDistSq in O(n) using direct distance checks —
		// no further tree queries or rebuilds needed.
		while (tableCount != total)
		{
			float mostDistantPointSqr = -1.0f;
			int   mostDistantPointIdx = 0;
			for (int j = 0; j < samples.Count; j++)
			{
				if (minDistSq[j] > mostDistantPointSqr)
				{
					mostDistantPointSqr = minDistSq[j];
					mostDistantPointIdx = j;
				}
			}

			Vec3 chosen = samples[mostDistantPointIdx];
			normalTable[tableCount++] = chosen;

			// Remove the chosen sample (swap/pop) and update minDistSq for all remaining candidates.
			int last = samples.Count - 1;
			samples[mostDistantPointIdx] = samples[last];
			minDistSq[mostDistantPointIdx] = minDistSq[last];
			samples.RemoveAt(last);
			minDistSq[last] = 0f;  // logically removed

			for (int j = 0; j < samples.Count; j++)
			{
				float d = Vec3.DistanceSquared(samples[j], chosen);
				if (d < minDistSq[j])
					minDistSq[j] = d;
			}

			log($"Normal {tableCount} farthest normal is {mostDistantPointSqr}");
		}

		// Build the two lookup trees and extract their baked leaf data for serialization.
		SpatialLookup tree128 = new SpatialLookup();
		for (int i = 0; i < 128; i++)
			tree128.AddToSet(normalTable[i], i);
		tree128.Rebuild();
		var (centroids128, radii128, ranges128, indices128) = tree128.GetBakedData();

		SpatialLookup tree32768 = new SpatialLookup();
		for (int i = 0; i < normalTable.Length; i++)
			tree32768.AddToSet(normalTable[i], i);
		tree32768.Rebuild();
		var (centroids32768, radii32768, ranges32768, indices32768) = tree32768.GetBakedData();

		WriteNormalsToFile(Path.Combine(Directory.GetCurrentDirectory(), "NormalTableData.cs"), normalTable,
			centroids128, radii128, ranges128, indices128,
			centroids32768, radii32768, ranges32768, indices32768);
	}

	private void AddToTableAndTree(Vec3[] normalTable, SpatialLookup normalTree, Vec3 p, int idx)
	{
		normalTable[idx] = p;
		normalTree.AddToSet(p, idx);
	}

	private void WriteNormalsToFile(string filePath, Vec3[] vectors,
		float[] centroids128, float[] radii128, int[] ranges128, int[] indices128,
		float[] centroids32768, float[] radii32768, int[] ranges32768, int[] indices32768)
	{
		using (StreamWriter w = new StreamWriter(filePath))
		{
			w.WriteLine("using System;");
			w.WriteLine("using Vec3 = System.Numerics.Vector3;");
			w.WriteLine("using ProgressiveNormals;");
			w.WriteLine();
			w.WriteLine("public static class NormalTableData");
			w.WriteLine("{");
			w.WriteLine("    static public Vec3 GetNormalByIndex(int index)");
			w.WriteLine("    {");
			w.WriteLine("        return new Vec3(_normalTable[index * 3], _normalTable[index * 3 + 1], _normalTable[index * 3 + 2]);");
			w.WriteLine("    }");
			w.WriteLine();
			w.WriteLine("    // Reconstructs the two SpatialLookup trees directly from baked data — no tree building at startup.");
			w.WriteLine("    public static SpatialLookup BuildTree128()");
			w.WriteLine("        => SpatialLookup.FromBakedData(_centroids128, _radii128, _ranges128, _indices128, GetNormalByIndex);");
			w.WriteLine("    public static SpatialLookup BuildTree32768()");
			w.WriteLine("        => SpatialLookup.FromBakedData(_centroids32768, _radii32768, _ranges32768, _indices32768, GetNormalByIndex);");
			w.WriteLine();
			w.WriteLine($"    // A progressive refinement table of {vectors.Length} normals.");
			w.WriteLine("    // If you use 7 bits, you can get low quality normals cheaply.");
			w.WriteLine("    // If you use 15 bits, you get pretty high quality normals.");
			w.WriteLine("    static private float[] _normalTable =");
			w.WriteLine("    {");
			foreach (Vec3 v in vectors)
				w.WriteLine($"        {v.X}f, {v.Y}f, {v.Z}f,");
			w.WriteLine("    };");
			w.WriteLine();
			WriteFloatArray(w, "_centroids128", centroids128);
			WriteFloatArray(w, "_radii128", radii128);
			WriteIntArray(w, "_ranges128", ranges128);
			WriteIntArray(w, "_indices128", indices128);
			w.WriteLine();
			WriteFloatArray(w, "_centroids32768", centroids32768);
			WriteFloatArray(w, "_radii32768", radii32768);
			WriteIntArray(w, "_ranges32768", ranges32768);
			WriteIntArray(w, "_indices32768", indices32768);
			w.WriteLine("}");
		}
		Console.WriteLine("Floats written to file successfully.");
	}

	private static void WriteFloatArray(StreamWriter w, string name, float[] data)
	{
		w.WriteLine($"    static private float[] {name} =");
		w.WriteLine("    {");
		for (int i = 0; i < data.Length; i += 8)
		{
			w.Write("        ");
			int end = Math.Min(i + 8, data.Length);
			for (int j = i; j < end; j++)
				w.Write($"{data[j]}f, ");
			w.WriteLine();
		}
		w.WriteLine("    };");
	}

	private static void WriteIntArray(StreamWriter w, string name, int[] data)
	{
		w.WriteLine($"    static private int[] {name} =");
		w.WriteLine("    {");
		for (int i = 0; i < data.Length; i += 16)
		{
			w.Write("        ");
			int end = Math.Min(i + 16, data.Length);
			for (int j = i; j < end; j++)
				w.Write($"{data[j]}, ");
			w.WriteLine();
		}
		w.WriteLine("    };");
	}
}
