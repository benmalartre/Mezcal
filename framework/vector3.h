//--------------------------------------------------------
// VECTOR3
//--------------------------------------------------------
#ifndef _VECTOR3_H_
#define _VECTOR3_H_

#include <math.h>
#include "matrix4.h"

namespace BOB{
class Vector3{
public:
    float x;
    float y;
    float z;
    // constructor
    Vector3(float X, float Y, float Z)
    {
        x=X;
        y=Y;
        z=Z;
    };
    
    // copy constructor
    Vector3(const Vector3& other)
    {
        x=other.x;
        y=other.y;
        z=other.z;
    };
    
    // empty constructor
    Vector3(){};
    
    // destructor
    ~Vector3(){};
    
    // access element
    float& operator [](long z)
    {
        return ((&x)[z]);
    }
    const float& operator [](long z) const
    {
        return ((&x)[z]);
    }
    
    // equal op
    Vector3& operator =(const Vector3& v)
    {
        x=v.x;
        y=v.y;
        z=v.z;
        return *this;
    };
    
    // add op
    Vector3& operator +=(const Vector3& v)
    {
        x+=v.x;
        y+=v.y;
        z+=v.z;
        return *this;
    };
    
    // in-place subtract op
    Vector3& operator -=(const Vector3& v)
    {
        x-=v.x;
        y-=v.y;
        z-=v.z;
        return *this;
    };
    
    // in-place multiply op
    Vector3& operator *=(float f)
    {
        x*=f;
        y*=f;
        z*=f;
        return *this;
    };
    
    // in-place divide op
    Vector3& operator /=(float f)
    {
        float invf = 1.0f/f;
        x*=invf;
        y*=invf;
        z*=invf;
        return *this;
    };
    
    // normalize in place
    Vector3& normalizeInPlace(void)
    {
        return (*this /= length());
    };
    
    // multiply by matrix in place
    Vector3& operator *(const Matrix4& m)
    {
        float tx, ty, tz, tw;
        tx = x * m[0] + y * m[4] + z * m[8] + m[12];
        ty = x * m[1] + y * m[5] + z * m[9] + m[13];
        tz = x * m[2] + y * m[6] + z * m[10] + m[14];
        tw = x * m[3] + y * m[7] + z * m[11] + m[15];
        x = tx/tw;
        y = ty/tw;
        z = tz/tw;
        return *this;
    }
    
    // length
    float length()
    {
        return(sqrt(x*x + y*y + z*z));
    };
    
    // length squared
    float lengthSquared()
    {
        return(x*x + y*y + z*z);
    };
    
    // negate op
    Vector3 operator -(void) const
    {
        return Vector3(-x, -y, -z);
    };
    
    // add op
    Vector3 operator +(const Vector3& v) const
    {
        return Vector3(x+v.x, y+v.y, z+v.z);
    };
    
    // subtract op
    Vector3 operator -(const Vector3& v) const
    {
        return Vector3(x-v.x, y-v.y, z-v.z);
    };
    
    // scale op
    Vector3 operator *(float f) const
    {
        return Vector3(x*f ,y*f ,z*f);
    };
    
    // multiply by matrix4 op
    Vector3 operator *(const Matrix4& m) const
    {
        float tx,ty,tz,tw;
        tx = x * m[0] + y * m[4] + z * m[8] + m[12];
        ty = x * m[1] + y * m[5] + z * m[9] + m[13];
        tz = x * m[2] + y * m[6] + z * m[10] + m[14];
        tw = x * m[3] + y * m[7] + z * m[11] + m[15];
        return Vector3(tx/tw, ty/tw, tz/tw);
    }
    
    // multiply by matrix4 in place op
    Vector3& operator *=(const Matrix4& m)
    {
        float tx,ty,tz,tw;
        tx = x * m[0] + y * m[4] + z * m[8] + m[12];
        ty = x * m[1] + y * m[5] + z * m[9] + m[13];
        tz = x * m[2] + y * m[6] + z * m[10] + m[14];
        tw = x * m[3] + y * m[7] + z * m[11] + m[15];
        x = tx/tw;
        y = ty/tw;
        z = tz/tw;
        return *this;
    }

	// divide by scalar op
    Vector3 operator /(float f) const
    {
        float invf=1.0f/f;
        return Vector3(x*invf, y*invf, z*invf);
    };
    
    // dot product op
    float operator *(const Vector3& v) const
    {
        return (x*v.x + y*v.y + z*v.z);
    };
    float dot(const Vector3& v) const
    {
        return (*this*v);
    };
    
    // cross product op
    Vector3 operator ^(const Vector3& v) const
    {
        return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    Vector3 cross(const Vector3& v) const
    {
        return (*this^v);
    }
    
    // normalize
    Vector3 normalize() const
    {
        return (Vector3(*this) /= (float)sqrt(x*x + y*y + z*z));
    }

};
} // end namespace BOB
#endif /* _VECTOR3_H_ */
