using Vec3 = System.Numerics.Vector3;
using Quat = System.Numerics.Quaternion;

namespace ProgressiveNormals
{
	internal class Program
	{
		static void Main(string[] args)
		{
			if (args.Length > 0 && args[0] == "generate")
			{
				new NormalTableGenerator().GenerateNormalTable(32768, msg => Console.WriteLine(msg));
				return;
			}

			Random rng = new Random(42);
			int testCount = 10000;

			TestNormalTable(rng, testCount);
			TestOctMorton(rng, testCount);
			TestSmallestThreeQuat(rng, testCount);

			Console.WriteLine();
			Console.WriteLine("All three schemes trade bits for angular error the same essential way; oct+Morton and");
			Console.WriteLine("smallest-three additionally let a single full-precision encoding be truncated to ANY");
			Console.WriteLine("bit count, where the table only supports the baked 7- and 15-bit levels.");
		}

		// Baseline: the baked progressive table (7-bit and 15-bit prefixes only).
		static void TestNormalTable(Random rng, int testCount)
		{
			NormalLookups lookups = new NormalLookups();
			float maxDeg128 = 0f, maxDeg32768 = 0f;

			for (int i = 0; i < testCount; i++)
			{
				Vec3 v = RandomUnitVector(rng);

				// Loose precision so it doesn't fall through to 32k, then tight precision to force it.
				int idx128 = lookups.FindClosestVec3(v, 1.0f);
				maxDeg128 = MathF.Max(maxDeg128, AngleDeg(v, NormalTableData.GetNormalByIndex(idx128)));

				int idx32k = lookups.FindClosestVec3(v, 0.0f);
				maxDeg32768 = MathF.Max(maxDeg32768, AngleDeg(v, NormalTableData.GetNormalByIndex(idx32k)));
			}

			Console.WriteLine($"Baked table lookup, {testCount} random normals (regenerate with 'dotnet run -- generate' if results look wrong):");
			Console.WriteLine($"    bits  7: max error = {maxDeg128,7:F3} deg   (128-entry prefix)");
			Console.WriteLine($"    bits 15: max error = {maxDeg32768,7:F3} deg   (full 32768-entry table)");
		}

		// Oct+Morton: one 32-bit encode per vector, then every coarser level is just a truncation.
		static void TestOctMorton(Random rng, int testCount)
		{
			int[] bitCounts = { 6, 7, 8, 10, 12, 15, 16, 20, 24, 32 };
			float[] maxDeg = new float[bitCounts.Length];

			for (int i = 0; i < testCount; i++)
			{
				Vec3 v = RandomUnitVector(rng);
				uint code = OctMortonNormal.Encode(v);  // encoded exactly once
				for (int b = 0; b < bitCounts.Length; b++)
				{
					Vec3 decoded = OctMortonNormal.Decode(OctMortonNormal.Truncate(code, bitCounts[b]), bitCounts[b]);
					maxDeg[b] = MathF.Max(maxDeg[b], AngleDeg(v, decoded));
				}
			}

			Console.WriteLine();
			Console.WriteLine($"Oct+Morton, {testCount} random normals, each encoded once at 32 bits then truncated:");
			for (int b = 0; b < bitCounts.Length; b++)
				Console.WriteLine($"    bits {bitCounts[b],2}: max error = {maxDeg[b],7:F3} deg");
		}

		// Smallest-three quaternion: one 62-bit encode per rotation, truncated the same way.
		// Note a rotation is 3 DOF where a normal is 2, so compare bits-per-DOF, not raw bits.
		static void TestSmallestThreeQuat(Random rng, int testCount)
		{
			int[] bitCounts = { 8, 11, 14, 17, 20, 23, 26, 32, 38, 47, 62 };
			float[] maxDeg = new float[bitCounts.Length];

			for (int i = 0; i < testCount; i++)
			{
				Quat q = RandomUnitQuaternion(rng);
				ulong code = SmallestThreeQuat.Encode(q);  // encoded exactly once
				for (int b = 0; b < bitCounts.Length; b++)
				{
					Quat decoded = SmallestThreeQuat.Decode(SmallestThreeQuat.Truncate(code, bitCounts[b]), bitCounts[b]);
					maxDeg[b] = MathF.Max(maxDeg[b], RotationAngleDeg(q, decoded));
				}
			}

			Console.WriteLine();
			Console.WriteLine($"Smallest-three quaternion, {testCount} random rotations, each encoded once at 62 bits then truncated:");
			for (int b = 0; b < bitCounts.Length; b++)
				Console.WriteLine($"    bits {bitCounts[b],2}: max error = {maxDeg[b],7:F3} deg   ({(bitCounts[b] - 2) / 3} bits/component)");
		}

		static Vec3 RandomUnitVector(Random rng)
		{
			// Random point on unit sphere via rejection sampling.
			Vec3 v;
			do
			{
				v = new Vec3((float)(rng.NextDouble() * 2 - 1),
							 (float)(rng.NextDouble() * 2 - 1),
							 (float)(rng.NextDouble() * 2 - 1));
			} while (v.LengthSquared() > 1f || v.LengthSquared() < 1e-6f);
			return Vec3.Normalize(v);
		}

		static Quat RandomUnitQuaternion(Random rng)
		{
			// Four gaussians normalized = uniform over rotation space.
			float x = Gaussian(rng), y = Gaussian(rng), z = Gaussian(rng), w = Gaussian(rng);
			return Quat.Normalize(new Quat(x, y, z, w));
		}

		static float Gaussian(Random rng)
		{
			double u1 = 1.0 - rng.NextDouble();
			double u2 = rng.NextDouble();
			return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
		}

		// Error metrics use the atan2 form in double rather than acos(dot): near zero angle, acos
		// amplifies the ~1e-7 norm error of float-normalized inputs into a phantom ~0.03 degrees,
		// while in atan2 the norm factors cancel.
		static float AngleDeg(Vec3 a, Vec3 b)
		{
			double dot = (double)a.X * b.X + (double)a.Y * b.Y + (double)a.Z * b.Z;
			double cx = (double)a.Y * b.Z - (double)a.Z * b.Y;
			double cy = (double)a.Z * b.X - (double)a.X * b.Z;
			double cz = (double)a.X * b.Y - (double)a.Y * b.X;
			return (float)(Math.Atan2(Math.Sqrt(cx * cx + cy * cy + cz * cz), dot) * (180.0 / Math.PI));
		}

		static float RotationAngleDeg(Quat a, Quat b)
		{
			// Angle of the relative rotation r = a * conj(b); abs() handles the q/-q double cover.
			double rw = (double)a.W * b.W + (double)a.X * b.X + (double)a.Y * b.Y + (double)a.Z * b.Z;
			double rx = (double)b.W * a.X - (double)a.W * b.X - ((double)a.Y * b.Z - (double)a.Z * b.Y);
			double ry = (double)b.W * a.Y - (double)a.W * b.Y - ((double)a.Z * b.X - (double)a.X * b.Z);
			double rz = (double)b.W * a.Z - (double)a.W * b.Z - ((double)a.X * b.Y - (double)a.Y * b.X);
			return (float)(2.0 * Math.Atan2(Math.Sqrt(rx * rx + ry * ry + rz * rz), Math.Abs(rw)) * (180.0 / Math.PI));
		}
	}
}
