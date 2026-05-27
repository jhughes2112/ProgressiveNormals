using Vec3 = System.Numerics.Vector3;
using ProgressiveNormals;

public class NormalLookups
{
    private SpatialLookup _normalTree128;
    private SpatialLookup _normalTree32768;

    public NormalLookups()
    {
        _normalTree128 = NormalTableData.BuildTree128();
        _normalTree32768 = NormalTableData.BuildTree32768();
    }

    public int FindClosestVec3(Vec3 normal, float precision)
    {
        float distSq = float.MaxValue;
        int nearestIdx = 0;
        _normalTree128.FindClosestVec3(ref normal, ref distSq, ref nearestIdx);

        if (distSq > precision * precision)
        {
            _normalTree32768.FindClosestVec3(ref normal, ref distSq, ref nearestIdx);
        }
        return nearestIdx;
    }
}
