#include "range1.h"

namespace BOB{
float Range1::getDistanceSquared(float p) const
{
    float dist = 0.0;

    if (p < _min) {
	// p is left of box
	dist += GfSqr(_min - p);
    }
    else if (p > _max) {
	// p is right of box
	dist += GfSqr(p - _max);
    }

    return dist;
}
} // end namespace BOB
