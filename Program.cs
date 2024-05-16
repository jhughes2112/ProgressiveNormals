using Vec3 = System.Numerics.Vector3;

namespace ProgressiveNormals
{
    internal class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello, World!");

			NormalLookups _normalLookups = new NormalLookups();

	        int index = _normalLookups.FindClosestVec3(new Vec3(1, 0, 0), 0.1f);
	        Console.WriteLine($"XYZ {index}");
        }
    }
}
