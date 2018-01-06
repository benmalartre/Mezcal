//======================================================
// TRIANGLE IMPLEMENTATION
//======================================================

#include "triangle.h"
namespace BOB{
//-------------------------------------------------------
// Triangle Center
//-------------------------------------------------------
void Triangle::getCenter(const float* positions, Vector3& center)
{
    center.x = (positions[vertices[0]*3]+positions[vertices[1]*3]+positions[vertices[2]*3])/3.0f;
    center.y = (positions[vertices[0]*3+1]+positions[vertices[1]*3+1]+positions[vertices[2]*3+1])/3.0f;
    center.z = (positions[vertices[0]*3+2]+positions[vertices[1]*3+2]+positions[vertices[2]*3+2])/3.0f;
}

//-------------------------------------------------------
// Triangle Normal
//-------------------------------------------------------
void Triangle::getNormal(const float* positions, Vector3& normal)
{
    // get triangle edges
    Vector3 AB(positions[vertices[1]*3]-positions[vertices[0]*3],
               positions[vertices[1]*3+1]-positions[vertices[0]*3+1],
               positions[vertices[1]*3+2]-positions[vertices[0]*3+2]);
    Vector3 AC(positions[vertices[2]*3]-positions[vertices[0]*3],
               positions[vertices[2]*3+1]-positions[vertices[0]*3+1],
               positions[vertices[2]*3+2]-positions[vertices[0]*3+2]);
    
    // cross product
    normal = AB^AC;
    
    // normalize
    normal.normalize();
}

//-------------------------------------------------------
// Triangle Closest Point
//-------------------------------------------------------
void Triangle::closestPoint( const float* positions, const Vector3 &point , Vector3& closest, float& u, float& v, float& w)
{
    Vector3 edge0;
    Vector3 edge1;
    
    edge0.x = positions[vertices[1]*3]-positions[vertices[0]*3];
    edge0.y = positions[vertices[1]*3+1]-positions[vertices[0]*3+1];
    edge0.z = positions[vertices[1]*3+2]-positions[vertices[0]*3+2];
    
    edge1.x = positions[vertices[2]*3]-positions[vertices[0]*3];
    edge1.y = positions[vertices[2]*3+1]-positions[vertices[0]*3+1];
    edge1.z = positions[vertices[2]*3+2]-positions[vertices[0]*3+2];
    
    Vector3 v0;
    v0.x = positions[vertices[0]*3] - point.x;
    v0.y = positions[vertices[0]*3+1] - point.y;
    v0.z = positions[vertices[0]*3+2] - point.z;
    
    float a = edge0.dot(edge0);
    float b = edge0.dot(edge1);
    float c = edge1.dot(edge1);
    float d = edge0.dot(v0);
    float e = edge1.dot(v0);
    
    float det = a*c - b*b;
    float s = b*e - c*d;
    float t = b*d - a*e;
    
    if ( s + t < det )
    {
        if ( s < 0.f )
        {
            if ( t < 0.f )
            {
                if ( d < 0.f )
                {
                    s = CLAMP( -d/a, 0.f, 1.f );
                    t = 0.f;
                }
                else
                {
                    s = 0.f;
                    t = CLAMP( -e/c, 0.f, 1.f );
                }
            }
            else
            {
                s = 0.f;
                t = CLAMP( -e/c, 0.f, 1.f );
            }
        }
        else if ( t < 0.f )
        {
            s = CLAMP( -d/a, 0.f, 1.f );
            t = 0.f;
        }
        else
        {
            float invDet = 1.f / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else
    {
        if ( s < 0.f )
        {
            float tmp0 = b+d;
            float tmp1 = c+e;
            if ( tmp1 > tmp0 )
            {
                float numer = tmp1 - tmp0;
                float denom = a-2*b+c;
                s = CLAMP( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                t = CLAMP( -e/c, 0.f, 1.f );
                s = 0.f;
            }
        }
        else if ( t < 0.f )
        {
            if ( a+d > b+e )
            {
                float numer = c+e-b-d;
                float denom = a-2*b+c;
                s = CLAMP( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                s = CLAMP( -e/c, 0.f, 1.f );
                t = 0.f;
            }
        }
        else
        {
            float numer = c+e-b-d;
            float denom = a-2*b+c;
            s = CLAMP( numer/denom, 0.f, 1.f );
            t = 1.f - s;
        }
    }
    
    closest.x = positions[vertices[0]*3];
    closest.y = positions[vertices[0]*3+1];
    closest.z = positions[vertices[0]*3+2];
    
    v = s;
    w = t;
    u = 1.0f - v - w;
    
    closest += edge0 * s + edge1 * t;
}

//-------------------------------------------------------
// Plane Box Test
//-------------------------------------------------------
bool Triangle::planeBoxTest(const Vector3& normal, const Vector3& vert, const Vector3& maxbox)
{
    int q;
    
    Vector3 vmin,vmax;
    float v;
    
    for(q=0;q<=2;q++)
    {
        
        v=vert[q];
        if(normal[q]>0.0f)
            
        {
            vmin[q]=-maxbox[q] - v;
            vmax[q]= maxbox[q] - v;
        }
        
        else
            
        {
            vmin[q]= maxbox[q] - v;
            vmax[q]=-maxbox[q] - v;
        }
        
    }
    
    if((normal.dot(vmin))>0.0f) return false;
    if((normal.dot(vmax))>=0.0f) return true;
    
    return false;
}

//-------------------------------------------------------
// Triangle Intersect Bounding Box
//-------------------------------------------------------
bool Triangle::touch(const float* positions, const Vector3& center, const Vector3& boxhalfsize)
{
    /*
     use separating axis theorem to test overlap between triangle and box
     need to test for overlap in these directions:
     
     1) the {x,y,z}-directions (actually, since we use the AABB of the triangle
     we do not even need to test these)
     2) normal of the triangle
     3) crossproduct(edge from triangle, {x,y,z}-direction)
     
     this gives 3x3=9 more tests
     */
    
    float min,max,p0,p1,p2,rad,fex,fey,fez;
    
    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */
    Vector3 v0 = Vector3(positions[vertices[0]*3], positions[vertices[0]*3+1], positions[vertices[0]*3+2]) - center;
    Vector3 v1 = Vector3(positions[vertices[1]*3], positions[vertices[1]*3+1], positions[vertices[1]*3+2]) - center;
    Vector3 v2 = Vector3(positions[vertices[2]*3], positions[vertices[2]*3+1], positions[vertices[2]*3+2]) - center;
    
    /* compute triangle edges */
    Vector3 e0 = v1-v0;
    Vector3 e1 = v2-v1;
    Vector3 e2 = v0-v2;
    
    /*  test the 9 tests first (this was faster) */
    fex = fabsf(e0.x);
    fey = fabsf(e0.y);
    fez = fabsf(e0.z);
    
    AXISTEST_X01(e0.z, e0.y, fez, fey);
    AXISTEST_Y02(e0.z, e0.x, fez, fex);
    AXISTEST_Z12(e0.y, e0.x, fey, fex);
    
    fex = fabsf(e1.x);
    fey = fabsf(e1.y);
    fez = fabsf(e1.z);
    
    AXISTEST_X01(e1.z, e1.y, fez, fey);
    AXISTEST_Y02(e1.z, e1.x, fez, fex);
    AXISTEST_Z0(e1.y, e1.x, fey, fex);
    
    fex = fabsf(e2.x);
    fey = fabsf(e2.y);
    fez = fabsf(e2.z);
    
    AXISTEST_X2(e2.z, e2.y, fez, fey);
    AXISTEST_Y1(e2.z, e2.x, fez, fex);
    AXISTEST_Z12(e2.y, e2.x, fey, fex);
    
    /*
     first test overlap in the {x,y,z}-directions
     find min, max of the triangle each direction, and test for overlap in
     that direction -- this is equivalent to testing a minimal AABB around
     the triangle against the AABB
     */
    
    // test in X-direction
    FINDMINMAX(v0.x,v1.x,v2.x,min,max);
    if(min>boxhalfsize.x || max<-boxhalfsize.x) return false;
    
    // test in Y-direction
    FINDMINMAX(v0.y,v1.y,v2.y,min,max);
    if(min>boxhalfsize.y || max<-boxhalfsize.y) return false;
    
    // test in Z-direction
    FINDMINMAX(v0.z,v1.z,v2.z,min,max);
    if(min>boxhalfsize.z || max<-boxhalfsize.z) return false;
    
    /*
     test if the box intersects the plane of the triangle
     compute plane equation of triangle: normal*x+d=0
     */
    Vector3 normal = e0^e1;
    
    if(!planeBoxTest(normal,v0,boxhalfsize)) return false;
    
    return true;   /* box and triangle overlaps */
}
}// end namespace BOB
