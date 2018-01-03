#include "line.h"

namespace BOB{

Vector3 Line::closestPoint(const Vector3 &point, float *t) const
{
    // Compute the vector from the start point to the given point.
    Vector3 v = point - pos;
    
    // Find the length of the projection of this vector onto the line.
    float lt = v.dot(dir);
    
    if (t)
        *t = lt;
    
    return getPoint( lt );
}

bool closestPoints( const Line &l1, const Line &l2,
                    Vector3 *closest1, Vector3 *closest2,
                    float *t1, float *t2 )
{
    const Vector3 &p1 = l1.pos;
    const Vector3 &d1 = l1.dir;
    const Vector3 &p2 = l2.pos;
    const Vector3 &d2 = l2.dir;

    float a = d1.dot(d2);
    float b = d1.dot(d1);
    float c = d1.dot(p1) - d1.dot(p2);
    float d = d2.dot(d2);
    float e = a;
    float f = d2.dot(p1) - d2.dot(p2);

    float denom = a * e - b * d;
    
    // denominator == 0 means the lines are parallel; no intersection.
    if (fabs(denom)< 1e-6 )
        return false;
    
    float lt1 = (c * d - a * f) / denom;
    float lt2 = (c * e - b * f) / denom;
    
    if ( closest1 )
        *closest1 = l1.getPoint( lt1 );
    
    if ( closest2 )
        *closest2 = l2.getPoint( lt2 );
    
    if ( t1 )
        *t1 = lt1;
    
    if ( t2 )
        *t2 = lt2;
    
    return true;
}
}//end namespace BOB
