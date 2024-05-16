using Vec3 = System.Numerics.Vector3;

public class NormalLookups
{
    private NormalOctree _normalTree128 = new NormalOctree(2.1f, new Vec3(0, 0, 0));
    private NormalOctree _normalTree32768 = new NormalOctree(2.1f, new Vec3(0, 0, 0));

    public NormalLookups()
    {
        for (int i = 0; i < 128; i++)
        {
            _normalTree128.AddToSet(NormalTableData.GetNormalByIndex(i), i);
        }
        for (int i = 0; i < 32768; i++)
        {
            _normalTree32768.AddToSet(NormalTableData.GetNormalByIndex(i), i);
        }
    }

    // This either returns an index <= 127 or an index <= 32767.
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
