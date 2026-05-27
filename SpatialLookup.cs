using System;
using System.Collections.Generic;
using Vec3 = System.Numerics.Vector3;

namespace ProgressiveNormals
{
	public class SpatialLookup
	{
		private const int kMaxPoints = 32;
		private const int kBranchFactor = 8;

		private Vec3 _centroid;
		private int _pointCount;
		private float _radius;
		private bool _frozen;   // set by Rebuild(); once frozen, AddToSet routes but never moves centroids
		private List<(Vec3 point, int index)>? _points;
		private SpatialLookup[]? _children;
		private SpatialLookup[]? _leaves;    // populated by Rebuild; flat list of all leaf nodes for query pruning

		public SpatialLookup()
		{
			_centroid = Vec3.Zero;
			_pointCount = 0;
			_radius = 0f;
			_points = new List<(Vec3 point, int index)>();
		}

		// The seed IS a point, so _pointCount starts at 1.  When the seed is re-binned via AddToSetInternal
		// the centroid math (seed*1 + seed)/2 = seed keeps it stable until other points arrive.
		private SpatialLookup(Vec3 seedCentroid)
		{
			_centroid = seedCentroid;
			_pointCount = 1;
			_radius = 0f;
			_points = new List<(Vec3 point, int index)>();
		}

		public void Rebuild(bool freeze = false)
		{
			// Single iterative traversal: collect all leaf points, clear leaf lists, reset all radii,
			// and record visit order so the same list can drive bottom-up radius propagation.
			List<(Vec3, int)> allPoints = new List<(Vec3, int)>(_pointCount);
			List<SpatialLookup> visited = new List<SpatialLookup>();
			List<SpatialLookup> leaves = new List<SpatialLookup>();
			Stack<SpatialLookup> stack = new Stack<SpatialLookup>();
			stack.Push(this);
			while (stack.Count > 0)
			{
				SpatialLookup node = stack.Pop();
				visited.Add(node);
				node._radius = 0f;
				if (node._children != null)
				{
					foreach (SpatialLookup child in node._children)
						stack.Push(child);
				}
				else
				{
					allPoints.AddRange(node._points!);
					node._points!.Clear();
					leaves.Add(node);
				}
			}

			// Re-bin every point top-down through the fixed centroid skeleton.
			foreach ((Vec3 p, int idx) in allPoints)
				ReBin(p, idx);

			// Propagate radii bottom-up.  Reversing the pre-order visited list yields post-order
			// (children before parent), so each parent sees its children's final radii.
			for (int i = visited.Count - 1; i >= 0; i--)
			{
				SpatialLookup node = visited[i];
				if (node._children == null)
					continue;
				foreach (SpatialLookup child in node._children)
				{
					float r = Vec3.Distance(node._centroid, child._centroid) + child._radius;
					if (r > node._radius)
						node._radius = r;
				}
			}

			if (freeze)
			{
				// Clear all leaf point lists so the structure is a pure spatial skeleton.
				// Points added after this via AddToSet go into leaves without moving any centroids.
				foreach (SpatialLookup leaf in leaves)
					leaf._points!.Clear();
				foreach (SpatialLookup node in visited)
				{
					node._frozen = true;
					node._pointCount = 0;
				}
			}

			_leaves = leaves.ToArray();
		}

		// Returns the baked leaf data after Rebuild() for serialization into NormalTableData.
		// centroids: packed x,y,z per leaf
		// radii: one float per leaf
		// ranges: packed (startIndex, count) per leaf — indexes into indices[]
		// indices: normal table indices grouped by leaf
		public (float[] centroids, float[] radii, int[] ranges, int[] indices) GetBakedData()
		{
			if (_leaves == null)
				throw new InvalidOperationException("Call Rebuild() before GetBakedData().");

			int leafCount = _leaves.Length;
			float[] centroids = new float[leafCount * 3];
			float[] radii = new float[leafCount];
			List<int> allIndices = new List<int>();
			int[] ranges = new int[leafCount * 2];

			for (int i = 0; i < leafCount; i++)
			{
				SpatialLookup leaf = _leaves[i];
				centroids[i * 3 + 0] = leaf._centroid.X;
				centroids[i * 3 + 1] = leaf._centroid.Y;
				centroids[i * 3 + 2] = leaf._centroid.Z;
				radii[i] = leaf._radius;
				ranges[i * 2 + 0] = allIndices.Count;
				ranges[i * 2 + 1] = leaf._points!.Count;
				foreach ((_, int idx) in leaf._points!)
					allIndices.Add(idx);
			}

			return (centroids, radii, ranges, allIndices.ToArray());
		}

		// Reconstructs a SpatialLookup directly from baked leaf data — no tree building.
		// getPoint maps a normal table index back to its Vec3 for populating leaf point lists.
		public static SpatialLookup FromBakedData(float[] centroids, float[] radii, int[] ranges, int[] indices, Func<int, Vec3> getPoint)
		{
			int leafCount = radii.Length;
			SpatialLookup root = new SpatialLookup();
			root._leaves = new SpatialLookup[leafCount];
			for (int i = 0; i < leafCount; i++)
			{
				SpatialLookup leaf = new SpatialLookup(new Vec3(centroids[i * 3], centroids[i * 3 + 1], centroids[i * 3 + 2]));
				leaf._radius = radii[i];
				int start = ranges[i * 2];
				int count = ranges[i * 2 + 1];
				leaf._points = new List<(Vec3, int)>(count);
				for (int j = 0; j < count; j++)
				{
					int idx = indices[start + j];
					leaf._points.Add((getPoint(idx), idx));
				}
				root._leaves[i] = leaf;
			}
			return root;
		}


		public void AddToSet(Vec3 p, int index)
		{
			if (!_frozen)
			{
				// Incrementally update centroid: new = (old * count + p) / (count + 1)
				_centroid = (_centroid * _pointCount + p) / (_pointCount + 1);
				_pointCount++;
			}

			if (_children != null)
			{
				FindNearestChild(p, index).AddToSet(p, index);
			}
			else
			{
				_points!.Add((p, index));
				if (!_frozen && _points.Count >= kMaxPoints)
					Split();
			}
		}

		private void Split()
		{
			Vec3[] seeds = FarthestPointSeeds(_points!, kBranchFactor);
			_children = new SpatialLookup[kBranchFactor];
			for (int i = 0; i < kBranchFactor; i++)
				_children[i] = new SpatialLookup(seeds[i]);

			foreach ((Vec3 v, int idx) in _points!)
				FindNearestChild(v, idx).AddToSet(v, idx);

			_points = null;
		}

		// Finds the child with the nearest centroid to p.  Ties are broken by pointIndex % tieCount.
		// Pass pointIndex = -1 when tie-breaking is not needed (query traversal).
		private SpatialLookup FindNearestChild(Vec3 p, int pointIndex)
		{
			float bestDistSq = float.MaxValue;
			for (int i = 0; i < _children!.Length; i++)
			{
				float d = Vec3.DistanceSquared(p, _children[i]._centroid);
				if (d < bestDistSq)
					bestDistSq = d;
			}

			int tieCount = 0;
			for (int i = 0; i < _children.Length; i++)
				if (Vec3.DistanceSquared(p, _children[i]._centroid) == bestDistSq)
					tieCount++;

			int pick = tieCount > 1 ? ((pointIndex % tieCount) + tieCount) % tieCount : 0;
			int seen = 0;
			for (int i = 0; i < _children.Length; i++)
			{
				if (Vec3.DistanceSquared(p, _children[i]._centroid) == bestDistSq)
				{
					if (seen == pick)
						return _children[i];
					seen++;
				}
			}
			return _children[0]; // unreachable
		}

		// Fast path (post-Rebuild): scan all leaves with centroid+radius pruning to cull hopeless buckets.
		// Slow path: tree traversal before Rebuild is called.
		public void FindClosestVec3(ref Vec3 p, ref float distSqrToClosest, ref int nearestIndex)
		{
			if (_leaves != null)
			{
				foreach (SpatialLookup leaf in _leaves)
				{
					float dist = Vec3.Distance(p, leaf._centroid);
					float closest = MathF.Max(0f, dist - leaf._radius);
					if (closest * closest >= distSqrToClosest)
						continue;
					foreach ((Vec3 v, int idx) in leaf._points!)
					{
						float d = Vec3.DistanceSquared(p, v);
						if (d < distSqrToClosest) { nearestIndex = idx; distSqrToClosest = d; }
					}
				}
				return;
			}
			if (_children != null)
			{
				SpatialLookup preferred = FindNearestChild(p, -1);
				preferred.FindClosestVec3(ref p, ref distSqrToClosest, ref nearestIndex);
				foreach (SpatialLookup child in _children)
				{
					if (child == preferred)
						continue;
					float dist = Vec3.Distance(p, child._centroid);
					float closest = MathF.Max(0f, dist - child._radius);
					if (closest * closest < distSqrToClosest)
						child.FindClosestVec3(ref p, ref distSqrToClosest, ref nearestIndex);
				}
			}
			else
			{
				foreach ((Vec3 v, int idx) in _points!)
				{
					float d = Vec3.DistanceSquared(p, v);
					if (d < distSqrToClosest) { nearestIndex = idx; distSqrToClosest = d; }
				}
			}
		}



		private void ReBin(Vec3 p, int index)
		{
			if (_children != null)
			{
				FindNearestChild(p, index).ReBin(p, index);
			}
			else
			{
				_points!.Add((p, index));
				float d = Vec3.Distance(_centroid, p);
				if (d > _radius)
					_radius = d;
			}
		}

		// Seeds k initial child centroids using farthest-point sampling: the first seed is the point
		// farthest from the bucket centroid, and each subsequent seed is the point farthest from all
		// previously chosen seeds.  This gives good spatial coverage for the initial k-means partition.
		private static Vec3[] FarthestPointSeeds(List<(Vec3 point, int index)> points, int k)
		{
			k = Math.Min(k, points.Count);
			Vec3[] seeds = new Vec3[k];

			Vec3 centroid = Vec3.Zero;
			foreach ((Vec3 p, _) in points)
				centroid += p;
			centroid /= points.Count;

			float[] minDistSqToSeeds = new float[points.Count];
			Array.Fill(minDistSqToSeeds, float.MaxValue);

			// First seed: farthest from the bucket centroid.
			int bestIdx = 0;
			float bestDistSq = -1f;
			for (int i = 0; i < points.Count; i++)
			{
				float d = Vec3.DistanceSquared(centroid, points[i].point);
				if (d > bestDistSq) { bestDistSq = d; bestIdx = i; }
			}
			seeds[0] = points[bestIdx].point;

			// Each subsequent seed: farthest from all previous seeds.
			for (int s = 1; s < k; s++)
			{
				Vec3 prev = seeds[s - 1];
				bestIdx = 0;
				bestDistSq = -1f;
				for (int i = 0; i < points.Count; i++)
				{
					float d = Vec3.DistanceSquared(prev, points[i].point);
					if (d < minDistSqToSeeds[i])
						minDistSqToSeeds[i] = d;
					if (minDistSqToSeeds[i] > bestDistSq) { bestDistSq = minDistSqToSeeds[i]; bestIdx = i; }
				}
				seeds[s] = points[bestIdx].point;
			}

			return seeds;
		}
	}
}

