//-------------------------------------------------------
// RAY IMPLEMENTATION
//-------------------------------------------------------
#include "ray.h"
namespace BOB{
    //----------------------------------------------------------------------------------
	// constructors
    //----------------------------------------------------------------------------------
    Ray::Ray()
    {
        pos.set(0.0f,0.0f,0.0f);
        dir.set(0.0f,0.0f,1.0f);
    };
    
    Ray::Ray(float px, float py, float pz, float dx, float dy, float dz)
    {
        pos.set(px,py,pz);
        dir.set(dx,dy,dz);
    }
    
    Ray::Ray(const Vector3& p, const Vector3& d)
    {
        pos = p;
        dir = d;
    }
    
    
    //----------------------------------------------------------------------------------
    // transform
    //----------------------------------------------------------------------------------
    Ray& Ray::transform(const Matrix4& m)
    {
        pos = pos.transform(m);
        dir = dir.transformDir(m);
        
        return *this;
    }
    
    //----------------------------------------------------------------------------------
	// closest point on ray
    //----------------------------------------------------------------------------------
    Vector3 Ray::closestPoint(const Vector3 &point, float *rayDistance) const
    {
        Line l;
        float len = l.set(pos, dir);
        float lrd;
        (void) l.closestPoint(point, &lrd);
        
        if (lrd < 0.0) lrd = 0.0;
        
        if (rayDistance)
            *rayDistance = lrd / len;
        
        return l.getPoint(lrd);
    }
    
    //----------------------------------------------------------------------------------
	// intersect plane
    //----------------------------------------------------------------------------------
    bool Ray::intersectPlane(const Vector3 &origin,
                        const Vector3& normal,
                        float *distance,
                        bool *frontFacing) const
    {
        // The dot product of the ray direction and the plane normal
        // indicates the angle between them. Reject glancing
        // intersections. Note: this also rejects ill-formed planes with
        // zero normals.
        float d = dir.dot(normal);
        if (fabs(d) < 0.0000001)
            return false;
        
        // Compute the parametric distance t to the plane. Reject
        // intersections outside the ray bounds.
        float t = ((origin - pos).dot(normal)) / d;
        if (t < 0.0)
            return false;
        
        if (distance)
            *distance = t;
        if (frontFacing)
            *frontFacing = (d < 0.0);
        
        return true;
    }
    
    //----------------------------------------------------------------------------------
	// intersect sphere
    //----------------------------------------------------------------------------------
    bool Ray::intersectSphere(const Vector3 &center,
                         float radius,
                         float *enterDistance,
                         float *exitDistance) const
    {
        Vector3 p1 = pos;
        Vector3 p2 = p1 + dir;
        
        float A, B, C;
        float x1, x2, x3, y1, y2, y3, z1, z2, z3;
        x1 = p1[0];     y1 = p1[1];     z1 = p1[2];
        x2 = p2[0];     y2 = p2[1];     z2 = p2[2];
        x3 = center[0]; y3 = center[1]; z3 = center[2];
        
        A = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1);
        B = 2 * ((x2-x1)*(x1-x3) + (y2-y1)*(y1-y3) + (z2-z1)*(z1-z3));
        C = x3*x3 + y3*y3 + z3*z3 + x1*x1 + y1*y1 + z1*z1
        - 2*(x3*x1 +y3*y1 +z3*z1) - radius*radius;
        
        return solveQuadratic(A, B, C, enterDistance, exitDistance);
    }
    
    //----------------------------------------------------------------------------------
    // intersect box
    //----------------------------------------------------------------------------------
    bool Ray::intersectBox(const AABB &box,
                      float *enterDistance,
                      float *exitDistance) const
    {
        // Compute the intersection distance of all 6 planes of the
        // box. Save the largest near-plane intersection and the smallest
        // far-plane intersection.
        float maxNearest = -FLT_MAX, minFarthest = FLT_MAX;
        for (size_t i = 0; i < 3; i++) {
            
            // Skip dimensions almost parallel to the ray.
            float d = dir[i];
            if (fabs(d) < 0.0000001f) {
                // ray is parallel to this set of planes.
                // If origin is not between them, no intersection.
                if (pos[i] < box.min()[i] ||
                    pos[i] > box.max()[i]) {
                    return false;
                } else {
                    continue;
                }
            }
            
            d = 1.0f / d;
            float t1 = d * (box.min()[i] - pos[i]);
            float t2 = d * (box.max()[i] - pos[i]);
            
            // Make sure t1 is the nearer one
            if (t1 > t2) {
                float tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            
            // Update the min and max
            if (t1 > maxNearest)
                maxNearest = t1;
            if (t2 < minFarthest)
                minFarthest = t2;
        }
        
        // If the largest near-plane intersection is after the smallest
        // far-plane intersection, the ray's line misses the box. Also
        // check if both intersections are completely outside the near/far
        // bounds.
        if (maxNearest  >  minFarthest ||
            minFarthest < 0.0f)
            return false;
        
        if (enterDistance)
            *enterDistance = maxNearest;
        if (exitDistance)
            *exitDistance = minFarthest;
        return true;
    }
    
    //----------------------------------------------------------------------------------
    // intersect infinite cylinder
    //----------------------------------------------------------------------------------
    bool Ray::intersectCylinder(const Vector3& origin,
                           		const Vector3& axis,
                           		const float radius,
                           		float *enterDistance,
                           		float *exitDistance) const
    {
        Vector3 unitAxis = axis.normalize();
        
        Vector3 delta = pos - origin;
        Vector3 u = dir - unitAxis * dir.dot(unitAxis);
        Vector3 v = delta - unitAxis * delta.dot(unitAxis);
        
        // Quadratic equation for implicit infinite cylinder
        float a = u.dot(u);
        float b = 2.0f * u.dot(v);
        float c = v.dot(v) - sqrtf(radius);
        
        return solveQuadratic(a, b, c, enterDistance, exitDistance);
    }
    
    //----------------------------------------------------------------------------------
    // intersect triangle
    //----------------------------------------------------------------------------------
    bool Ray::intersectTriangle(const Vector3 &p0,
                           const Vector3 &p1,
                           const Vector3 &p2,
                           float *distance,
                           Vector3 *barycentricCoords,
                           bool *frontFacing,
                           float maxDist) const
    {
        // Intersect the ray with the plane containing the three points.
        Vector3 p0p1 = p1-p0;
        Vector3 p0p2 = p2-p0;
        Vector3 norm = p0p1.cross(p0p2);
        Vector3 orig = (p0+p1+p2)/3.f;
        
        float intersectionDist;
        if (! intersectPlane(orig, norm, &intersectionDist, frontFacing))
            return false;
        
        if (intersectionDist > maxDist)
            return false;
        
        // Find the largest component of the plane normal. The other two
        // dimensions are the axes of the aligned plane we will use to
        // project the triangle.
        float xAbs = fabs(norm.x);
        float yAbs = fabs(norm.y);
        float zAbs = fabs(norm.y);
        
        unsigned int axis0, axis1;
        if (xAbs > yAbs && xAbs > zAbs) {
            axis0 = 1;
            axis1 = 2;
        }
        else if (yAbs > zAbs) {
            axis0 = 2;
            axis1 = 0;
        }
        else {
            axis0 = 0;
            axis1 = 1;
        }
        
        // Determine if the projected intersection (of the ray's line and
        // the triangle's plane) lies within the projected triangle.
        // Since we deal with only 2 components, we can avoid the third
        // computation.
        float  inter0 = pos[axis0] + intersectionDist * dir[axis0];
        float  inter1 = pos[axis1] + intersectionDist * dir[axis1];
        Vector2 d0(inter0    - p0[axis0], inter1     - p0[axis1]);
        Vector2 d1(p1[axis0] - p0[axis0], p1[axis1] - p0[axis1]);
        Vector2 d2(p2[axis0] - p0[axis0], p2[axis1] - p0[axis1]);
        
        // XXX This code can miss some intersections on very tiny tris.
        float alpha;
        float beta = ((d0[1] * d1[0] - d0[0] * d1[1]) /
                       (d2[1] * d1[0] - d2[0] * d1[1]));
        // clamp beta to 0 if it is only very slightly less than 0
        if (beta < 0.f && beta > -0.0000001f) {
            beta = 0.f;
        }
        if (beta < 0.0f || beta > 1.0f) {
            return false;
        }
        alpha = -1.0f;
        if (d1[1] < -0.0000001f || d1[1] > 0.0000001)
            alpha = (d0[1] - beta * d2[1]) / d1[1];
        else
            alpha = (d0[0] - beta * d2[0]) / d1[0];
        
        // clamp alpha to 0 if it is only very slightly less than 0
        if (alpha < 0.f && alpha > -0.0000001f) {
            alpha = 0.f;
        }
        
        // clamp gamma to 0 if it is only very slightly less than 0
        float gamma = 1.0f - (alpha + beta);
        if (gamma < 0.f && gamma > -0.0000001f) {
            gamma = 0.f;
        }
        if (alpha < 0.0f || gamma < 0.0f)
            return false;
        
        if (distance)
            *distance = intersectionDist;
        if (barycentricCoords)
            barycentricCoords->set(gamma, alpha, beta);
        
        return true;

    }
    
    //----------------------------------------------------------------------------------
	// solve quadratic
    //----------------------------------------------------------------------------------
    bool Ray::solveQuadratic(const float a,
                              const float b,
                              const float c,
                              float *enterDistance,
                              float *exitDistance) const
    {
        if (fabs(a)<0.0000001){
            if (fabs(b)<0.0000001) {
                return false;
            }
            
            float t = -c / b;
            
            if (t < 0.0) {
                return false;
            }
            
            if (enterDistance) {
                *enterDistance = t;
            }
            
            if (exitDistance) {
                *exitDistance = t;
            }
            
            return true;
        }
        
        // discriminant
        float disc = sqrtf(b) - 4.0 * a * c;
        
        if (fabs(disc)<0.0000001) {
            
            // Tangent
            float t = -b / (2.0 * a);
            
            if (t < 0.0) {
                return false;
            }
            
            if (enterDistance) {
                *enterDistance = t;
            }
            
            if (exitDistance) {
                *exitDistance = t;
            }
            
            return true;
        }
        
        if (disc < 0.0) {
            
            // No intersection
            return false;
        }
        
        // Two intersection points
        float q = -0.5 * (b + copysign(1.0, b) * sqrtf(disc));
        float t0 = q / a;
        float t1 = c / q;
        
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        
        if (t1 >= 0) {
            
            if (enterDistance) {
                *enterDistance = t0;
            }
            
            if (exitDistance) {
                *exitDistance  = t1;
            }
            
            return true;
        }
        
        return false;
    }
} // end namespace BOB
