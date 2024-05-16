using System.Collections.Generic;
using Vec3 = System.Numerics.Vector3;

// This is either a list of points or 8 child nodes split by midpoint coordinates.  Each node tracks the min/max of the actual content
// (and its recursive content) so you can quickly reject neighboring cells that don't have content that's close enough to scan.
public class NormalOctree
{
    private const int kMaxPoints = 32;
    private Vec3 _center;   // this is always maintained, whether this has points or has already been subdivided
    private float _width;    // this is the full width of a cell
    private Vec3 _maxEdge;  // keeping this updated in realtime is easy, and cuts down on unnecessary recursion into nodes that have no data
    private Vec3 _minEdge;
    private List<(Vec3, int)>? _points = new List<(Vec3, int)>();  // points go in here until there are kMaxPoints in this bucket, then we calculate the centroid and rebucket all the points, nulling out this list.
    private NormalOctree[]? _childNodes = null;

    // Because normals are unit length and spherical, the root of a tree should always be width of about 2.1 and centered around Vec3.Zero, just to make sure it's all enclosed nicely.
    public NormalOctree(float width, Vec3 center)
    {
        _width = width;
        _center = center;
        _minEdge = center;
        _maxEdge = center;
    }

    // Updates the distSqrToClosest and nearestIndex if it is better.
    public void FindClosestVec3(ref Vec3 p, ref float distSqrToClosest, ref int nearestIndex)
    {
        if (_childNodes != null)  // recurse if this node is split already
        {
            // First, we attempt to find the point in the 'correct' cell for p.  We do this first because it may reduce distSqrToClosest and we don't search any other cells.
            NormalOctree preferredChild = PickChildNodeForAdd(p);
            preferredChild.FindClosestVec3(ref p, ref distSqrToClosest, ref nearestIndex);

            // search all other child nodes if their nearest edge is closer than distSqrToClosest
            foreach (NormalOctree childNode in _childNodes)
            {
                if (childNode != preferredChild)
                {
                    bool closeEnough = (p.X - childNode._maxEdge.X) * (p.X - childNode._maxEdge.X) < distSqrToClosest || (p.X - childNode._minEdge.X) * (p.X - childNode._minEdge.X) < distSqrToClosest ||
                                        (p.Y - childNode._maxEdge.Y) * (p.Y - childNode._maxEdge.Y) < distSqrToClosest || (p.Y - childNode._minEdge.Y) * (p.Y - childNode._minEdge.Y) < distSqrToClosest ||
                                        (p.Z - childNode._maxEdge.Z) * (p.Z - childNode._maxEdge.Z) < distSqrToClosest || (p.Z - childNode._minEdge.Z) * (p.Z - childNode._minEdge.Z) < distSqrToClosest;
                    if (closeEnough)
                    {
                        childNode.FindClosestVec3(ref p, ref distSqrToClosest, ref nearestIndex);
                    }
                }
            }
        }
        else  // search the points in this node for the nearest point
        {
            foreach ((Vec3 v, int idx) in _points!)
            {
                float d = Vec3.DistanceSquared(p, v);
                if (d < distSqrToClosest)
                {
                    nearestIndex = idx;
                    distSqrToClosest = d;
                }
            }
        }
    }
    public void AddToSet(Vec3 p, int index)
    {
        // intermediary nodes also maintain their min/max edges to prevent unnecessary recursion
        _minEdge = Vec3.Min(_minEdge, p);
        _maxEdge = Vec3.Max(_maxEdge, p);

        if (_points != null)
        {
            _points.Add((p, index));
            if (_points.Count == kMaxPoints)  // split if we have enough points
            {
                Split();
            }
        }
        else
        {
            PickChildNodeForAdd(p).AddToSet(p, index);
        }
    }
    // This returns the correct child node based on position, regardless of whether points are there or not.
    private NormalOctree PickChildNodeForAdd(Vec3 p)
    {
        int index = (p.X > _center.X ? 1 : 0) + (p.Y > _center.Y ? 2 : 0) + (p.Z > _center.Z ? 4 : 0);
        return _childNodes![index];
    }
    private void Split()
    {
        // create 8 buckets based on being > or <= the center axis
        _childNodes = new NormalOctree[8];
        float halfWidth = _width * 0.5f;
        float quarterWidth = _width * 0.25f;
        _childNodes[0] = new NormalOctree(halfWidth, _center + new Vec3(-quarterWidth, -quarterWidth, -quarterWidth));
        _childNodes[1] = new NormalOctree(halfWidth, _center + new Vec3(quarterWidth, -quarterWidth, -quarterWidth));
        _childNodes[2] = new NormalOctree(halfWidth, _center + new Vec3(-quarterWidth, quarterWidth, -quarterWidth));
        _childNodes[3] = new NormalOctree(halfWidth, _center + new Vec3(quarterWidth, quarterWidth, -quarterWidth));
        _childNodes[4] = new NormalOctree(halfWidth, _center + new Vec3(-quarterWidth, -quarterWidth, quarterWidth));
        _childNodes[5] = new NormalOctree(halfWidth, _center + new Vec3(quarterWidth, -quarterWidth, quarterWidth));
        _childNodes[6] = new NormalOctree(halfWidth, _center + new Vec3(-quarterWidth, quarterWidth, quarterWidth));
        _childNodes[7] = new NormalOctree(halfWidth, _center + new Vec3(quarterWidth, quarterWidth, quarterWidth));

        // add each point to the appropriate bucket
        foreach ((Vec3 v, int idx) in _points!)
        {
            PickChildNodeForAdd(v).AddToSet(v, idx);
        }

        // null out points
        _points = null;
    }
}
