//--------------------------------------------------------
// RAY
//--------------------------------------------------------
#ifndef _RAY_H_
#define _RAY_H_

#include "vector3.h"

namespace BOB{
class AABB;
class Triangle;

class Ray{
    public:
    Vector3 pos;
    Vector3 dir;

    Ray();
    Ray(float px, float py, float pz, float dx, float dy, float dz);
    Ray(const Vector3& p, const Vector3& d);
    ~Ray();
};
}// end namespace BOB
#endif /* _RAY_H_ */
