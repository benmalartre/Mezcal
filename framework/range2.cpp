#include "range2.h"

namespace BOB {
float Range2::getDistanceSquared(const Vector2 &p) const
{
    double dist = 0.0;

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

    return dist;
}

Vector2 Range2::getCorner(size_t i) const
{
    if (i > 3) {
        //TF_CODING_ERROR("Invalid corner %zu > 3.", i);
        return _min;
    }

    return Vector2((i & 1 ? _max : _min)[0], (i & 2 ? _max : _min)[1]);
}

Range2 Range2::getQuadrant(size_t i) const
{
    if (i > 3) {
        //TF_CODING_ERROR("Invalid quadrant %zu > 3.", i);
        return Range2();
    }

    Vector2 a = getCorner(i);
    Vector2 b = .5 * (_min + _max);

    return Range2(
        Vector2(MINIMUM(a[0], b[0]), MINIMUM(a[1], b[1])),
        GfVec2f(MAXIMUM(a[0], b[0]), MAXIMUM(a[1], b[1])));
}

const Range2 Range2::unitSquare(Vector2(0,0), Vector2(1,1));

} // end namespace BOB
