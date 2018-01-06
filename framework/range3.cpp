#include "range3.h"

#include <cfloat>

namespace BOB{

float Range3::getDistanceSquared(const Vector3 &p) const
{
    float dist = 0.0;

    if (p[0] < _min[0]) {
	// p is left of box
	dist += sqrtf(_min[0] - p[0]);
    }
    else if (p[0] > _max[0]) {
	// p is right of box
	dist += sqrtf(p[0] - _max[0]);
    }
    if (p[1] < _min[1]) {
	// p is front of box
	dist += sqrtf(_min[1] - p[1]);
    }
    else if (p[1] > _max[1]) {
	// p is back of box
	dist += sqrtf(p[1] - _max[1]);
    }
    if (p[2] < _min[2]) {
	// p is below of box
	dist += sqrtf(_min[2] - p[2]);
    }
    else if (p[2] > _max[2]) {
	// p is above of box
	dist += sqrtf(p[2] - _max[2]);
    }

    return dist;
}

Vector3 Range3::getCorner(size_t i) const
{
    if (i > 7) {
        //TF_CODING_ERROR("Invalid corner %zu > 7.", i);
        return _min;
    }
    return Vector3(
        (i & 1 ? _max : _min)[0],
        (i & 2 ? _max : _min)[1],
        (i & 4 ? _max : _min)[2]);
}

Range3 Range3::getOctant(size_t i) const
{
    if (i > 7) {
        //TF_CODING_ERROR("Invalid octant %zu > 7.", i);
        return Range3();
    }

    Vector3 a = getCorner(i);
    Vector3 b = (_min + _max) * 0.5f;

    return Range3(
        Vector3(MINIMUM(a[0], b[0]), MINIMUM(a[1], b[1]), MINIMUM(a[2], b[2])),
        Vector3(MAXIMUM(a[0], b[0]), MAXIMUM(a[1], b[1]), MAXIMUM(a[2], b[2])));
}

const Range3 Range3::unitCube(Vector3(0,0,0), Vector3(1,1,1));

} // end namespace BOB
