using System;
using System.Collections.Generic;
using System.IO;
using Vec3 = System.Numerics.Vector3;

// This is just tools-time code to generate a table of 32k normals.  Probably don't ever need it again, but checked in just in case.
public class NormalTableGenerator
{
	// Produce a huge ordered set of normals so that the farther down the table you are able to index, the more normals fill in the gaps left by the previous ones.
	// This takes about 3 hours because I didn't want to pollute the octree implementation with an early termination if the current point is closer than the farthest known.
	// That would make this super fast though.
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

		// Seed the table with exact normal values we want present.
		int tableCount = 0;
		Vec3[] normalTable = new Vec3[total];
		NormalOctree normalTree = new NormalOctree(2.1f, Vec3.Zero);
		AddToTableAndTree(normalTable, normalTree, new Vec3(0,0,0), tableCount++);
		AddToTableAndTree(normalTable, normalTree, new Vec3(1,0,0), tableCount++);
		AddToTableAndTree(normalTable, normalTree, new Vec3(0,1,0), tableCount++);
		AddToTableAndTree(normalTable, normalTree, new Vec3(0,0,1), tableCount++);
		AddToTableAndTree(normalTable, normalTree, new Vec3(-1,0,0), tableCount++);
		AddToTableAndTree(normalTable, normalTree, new Vec3(0,-1,0), tableCount++);
		AddToTableAndTree(normalTable, normalTree, new Vec3(0,0,-1), tableCount++);

		// Progressively add the next farthest normal until we have reached total.
		while (tableCount!=total)
		{
			float mostDistantPointSqr = -1.0f;
			int   mostDistantPointIdx = 0;
			for (int j=0; j<samples.Count; j++)  // run through each possible sample point and see if it's farthest away from all points already in the set
			{
				Vec3 sample = samples[j];
				float distSqrToClosest = float.MaxValue;
				int idx = -1;
				normalTree.FindClosestVec3(ref sample, ref distSqrToClosest, ref idx);  // this finds the normal in the table that is closest to sample
				if (distSqrToClosest > mostDistantPointSqr)  // this is the best sample we've seen so far, remember it
				{
					mostDistantPointSqr = distSqrToClosest;
					mostDistantPointIdx = j;
				}
			}
			AddToTableAndTree(normalTable, normalTree, samples[mostDistantPointIdx], tableCount++);  // put this into the set, remove it from the samples array
			samples[mostDistantPointIdx] = samples[samples.Count-1];  // swap/pop trick
			samples.RemoveAt(samples.Count-1);
			log($"Normal {tableCount} farthest normal is {mostDistantPointSqr}");
		}

		WriteNormalsToFile("D:/normals.csv", normalTable);
	}

	private void AddToTableAndTree(Vec3[] normalTable, NormalOctree normalTree, Vec3 p, int idx)
	{
		normalTable[idx] = p;
		normalTree.AddToSet(p, idx);
	}

	private void WriteNormalsToFile(string filePath, Vec3[] vectors)
	{
		using (StreamWriter writer = new StreamWriter(filePath))
		{
			foreach (Vec3 v in vectors)
			{
				writer.WriteLine($"{v.X}, {v.Y}, {v.Z}");
			}
		}
		Console.WriteLine("Floats written to file successfully.");
	}
}
