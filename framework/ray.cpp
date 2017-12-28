//-------------------------------------------------------
// RAY IMPLEMENTATION
//-------------------------------------------------------
#include "ray.h"
namespace BOB{
Ray::Ray()
{
    pos.x = 0.0f;
    pos.y = 0.0f;
    pos.z = 0.0f;
    dir.x = 0.0f;
    dir.y = 0.0f;
    dir.z = 1.0f;
};

Ray::Ray(float px, float py, float pz, float dx, float dy, float dz)
{
    pos.x = px;
    pos.y = py;
    pos.z = pz;
    dir.x = dx;
    dir.y = dy;
    dir.z = dz;
}

Ray::Ray(const Vector3& p, const Vector3& d)
{
    pos.x = p.x;
    pos.y = p.y;
    pos.z = p.z;
    dir.x = d.x;
    dir.y = d.y;
    dir.z = d.z;
}
/*
bool Ray::intersectBox(const AABB& box)
{
    
}

bool Ray::intersectSphere(const Sphere& sphere)
{
    
}

bool Ray::intersectTriangle(const Triangle& tri)
{
    
}*/
} // end namespace BOB
