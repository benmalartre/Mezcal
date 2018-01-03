//--------------------------------------------------------
// RAY
//--------------------------------------------------------
#ifndef _RAY_H_
#define _RAY_H_

#include "vector2.h"
#include "vector3.h"
#include "vector4.h"
#include "aabb.h"
#include "line.h"
#include <limits>
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
    
    Ray& transform(const Matrix4& matrix);
    
    Vector3 closestPoint(const Vector3 &point, float *rayDistance = NULL) const;
    
    bool intersectPlane(const Vector3 &origin,
                        const Vector3& normal,
                        float *distance = NULL,
                        bool *frontFacing = NULL) const;
    
    bool intersectSphere(const Vector3 &center,
                         float radius,
                         float *enterDistance = NULL,
                         float *exitDistance = NULL ) const;
    
    bool intersectBox(const AABB &box,
                      float *enterDistance = NULL,
                      float *exitDistance = NULL) const;
    
    bool intersectTriangle(const Vector3 &p0,
                           const Vector3 &p1,
                           const Vector3 &p2,
                           float *distance = NULL,
                           Vector3 *barycentricCoords = NULL,
                           bool *frontFacing = NULL,
                           float maxDist = std::numeric_limits<float>::infinity()) const;
    
    bool intersectCylinder(const Vector3 &origin,
                           const Vector3 &axis,
                           const float  radius,
                           float *enterDistance = NULL,
                           float *exitDistance = NULL) const;
    
    bool solveQuadratic(const float a,
                        const float b,
                        const float c,
                        float *enterDistance,
                        float *exitDistance) const;
    
    //float intersectBox(const Matrix4 & tx, const Vector3 & br );
    //bool intersectBox(AABB* box);
    ~Ray();
};
}// end namespace BOB
#endif /* _RAY_H_ */
