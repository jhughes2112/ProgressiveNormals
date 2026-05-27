using Vec3 = System.Numerics.Vector3;

namespace ProgressiveNormals
{
	internal class Program
	{
		static void Main(string[] args)
		{
			if (false)
			{
				new NormalTableGenerator().GenerateNormalTable(32768, msg => Console.WriteLine(msg));
			}
			else
			{
				NormalLookups lookups = new NormalLookups();
				Random rng = new Random(42);
				int testCount = 10000;
				float maxAngleDeg128 = 0f;
				float maxAngleDeg32768 = 0f;

				for (int i = 0; i < testCount; i++)
				{
					// Random point on unit sphere via rejection sampling
					Vec3 v;
					do
					{
						v = new Vec3((float)(rng.NextDouble() * 2 - 1),
									 (float)(rng.NextDouble() * 2 - 1),
									 (float)(rng.NextDouble() * 2 - 1));
					} while (v.LengthSquared() > 1f || v.LengthSquared() < 1e-6f);
					v = Vec3.Normalize(v);

					// Test 128-entry table (loose precision so it doesn't fall through to 32k)
					int idx128 = lookups.FindClosestVec3(v, 1.0f);
					Vec3 found128 = NormalTableData.GetNormalByIndex(idx128);
					float dot128 = Math.Clamp(Vec3.Dot(v, found128), -1f, 1f);
					float angle128 = MathF.Acos(dot128) * (180f / MathF.PI);
					if (angle128 > maxAngleDeg128) maxAngleDeg128 = angle128;

					// Test 32768-entry table (tight precision forces full lookup)
					int idx32k = lookups.FindClosestVec3(v, 0.0f);
					Vec3 found32k = NormalTableData.GetNormalByIndex(idx32k);
					float dot32k = Math.Clamp(Vec3.Dot(v, found32k), -1f, 1f);
					float angle32k = MathF.Acos(dot32k) * (180f / MathF.PI);
					if (angle32k > maxAngleDeg32768) maxAngleDeg32768 = angle32k;
				}

				Console.WriteLine($"Tested {testCount} random normals (against existing baked NormalTableData)");
				Console.WriteLine($"  128-entry  table: max error = {maxAngleDeg128:F3} degrees");
				Console.WriteLine($"  32768-entry table: max error = {maxAngleDeg32768:F3} degrees");
				Console.WriteLine("Note: regenerate the table first if results look wrong (e.g. 90 degrees = zero vector at index 0).");
			}
		}
	}
}
