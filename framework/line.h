//--------------------------------------------------------
// Line Declaration
//--------------------------------------------------------
#ifndef _LINE_H_
#define _LINE_H_

#include <float.h>
#include "vector3.h"
#include "matrix4.h"
#include "point.h"

namespace BOB{
class Line{
public:
    
    Line() {};
    
    Line(const Vector3 &p0, const Vector3 &dir ) {
        set( p0, dir );
    }
        
	float set(const Vector3 &p, const Vector3 &d )
    {
        float l = d.length();
        pos = p;
        dir = d.normalize();
        return l;
    }
        
    Vector3 getPoint( float t ) const { return pos + dir * t; }
    const Vector3 &getDirection() const { return dir; }
        
    Vector3 closestPoint(const Vector3 &point, float *t = NULL) const;
    
    bool operator ==(const Line &l) const {
    	return pos == l.pos &&	dir  == l.dir;
    }
    
    bool operator !=(const Line &r) const {
        return ! (*this == r);
    }
        
private:
    friend bool closestPoints( const Line &, const Line &,
                                        Vector3 *, Vector3 *,
                                        float *, float * );

    Vector3             pos;
    Vector3             dir;
};
    
/// Computes the closets points between two lines.
bool closestPoints(const Line &l1, const Line &l2,
                   Vector3 *p1 = nullptr, Vector3 *p2 = nullptr,
                   float *t1 = nullptr, float *t2 = nullptr);

};//end namespace BOB
#endif /* _AABB_H_ */
