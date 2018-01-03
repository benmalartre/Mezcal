//--------------------------------------------------------
// VECTOR4
//--------------------------------------------------------
#ifndef _VECTOR4_H_
#define _VECTOR4_H_

#include <math.h>
#include "matrix4.h"

namespace BOB{
class Vector3;
class Vector4{
    public:
    float x;
    float y;
    float z;
    float w;
    
    Vector4(float X, float Y, float Z, float W)
    {
        x=X;
        y=Y;
        z=Z;
        w=W;
    };
    
    Vector4(const Vector3& other)
    {
        x=other.x;
        y=other.y;
        z=other.z;
        w=1.0f;
    };
    
    Vector4(const Vector3& other, float W)
    {
        x=other.x;
        y=other.y;
        z=other.z;
        w=W;
    }
    
    Vector4(const Vector4& other)
    {
        x=other.x;
        y=other.y;
        z=other.z;
        w=other.w;
    };
    
    Vector4(){};
    
    ~Vector4(){};
    
    float& operator [](long z)
    {
        return ((&x)[z]);
    }
    const float& operator [](long z) const
    {
        return ((&x)[z]);
    }
    
    Vector4& operator =(const Vector3& v)
    {
        x=v.x;
        y=v.y;
        z=v.z;
        w=1.0f;
        return *this;
    };
    
    Vector4& operator =(const Vector4& v)
    {
        x=v.x;
        y=v.y;
        z=v.z;
        w=v.w;
        return *this;
    };
    
    Vector4& operator +=(const Vector4& v)
    {
        x+=v.x;
        y+=v.y;
        z+=v.z;
        w+=v.w;
        return *this;
    };
    
    Vector4& operator -=(const Vector4& v)
    {
        x-=v.x;
        y-=v.y;
        z-=v.z;
        w-=v.w;
        return *this;
    };
    
    Vector4& operator /(float f)
    {
        float invf=1.0f/f;
        x*=invf;
        y*=invf;
        z*=invf;
        w*=invf;
        return *this;
    };
    
    Vector4& operator *=(float f)
    {
        x*=f;
        y*=f;
        z*=f;
        w*=f;
        return *this;
    };
    
    Vector4& operator /=(float f)
    {
        float invf = 1.0f/f;
        x*=invf;
        y*=invf;
        z*=invf;
        w*=invf;
        return *this;
    };
    
    // comparison
    bool operator==(const Vector4& other) const
    {
        return (x==other.x && y==other.y && z==other.z && w==other.w);
    }
    
    bool operator!=(const Vector4& other) const
    {
        return (x!=other.x || y!=other.y || z!=other.z || w!=other.w);
    }
    
    // length squared
    float lengthSquared() const
    {
        return(x*x + y*y + z*z + w*w);
    };
    
    // length
    float length() const
    {
        return(sqrt(lengthSquared()));
    };
    
    // normalize in place
    Vector4& normalize(void)
    {
        float l = length();
        return (*this /= l);
    };
    
    // multiply by matrix in place
    Vector4& operator *(const Matrix4& m)
    {
        float tx,ty,tz,tw;

        tx = x * m[0] + y * m[1] + z * m[2] + w * m[3];
        ty = x * m[4] + y * m[5] + z * m[6] + w * m[7];
        tz = x * m[8] + y * m[9] + z * m[10] + w * m[11];
        tw = x * m[12] + y * m[13] + z * m[15] + w * m[15];

        x = tx;
        y = ty;
        z = tz;
        w = tw;
        return *this;
    }
    
    // negate
    Vector4 operator -(void) const
    {
        return Vector4(-x, -y, -z, -w);
    };
    
    // add
    Vector4 operator +(const Vector4& v) const
    {
        return Vector4(x+v.x, y+v.y, z+v.z, w+v.w);
    };
    
    // subtract
    Vector4 operator -(const Vector4& v) const
    {
        return Vector4(x-v.x, y-v.y, z-v.z, w-v.w);
    };
    
    // scale
    Vector4 operator *(float f) const
    {
        return Vector4(x*f ,y*f ,z*f, w*f);
    };
    
    // multiply by matrix4
    Vector4 operator *(const Matrix4& m) const
    {
        Vector4 v;

        v.x = x * m[0]  + y * m[1]  + z * m[2]  + w * m[3] ;
        v.y = x * m[4]  + y * m[5]  + z * m[6]  + w * m[7] ;
        v.z = x * m[8]  + y * m[9]  + z * m[10] + w * m[11];
        v.w = x * m[12] + y * m[13] + z * m[15] + w * m[15];

        return v;
    }
    
    // multiply by matrix4 in place
    Vector4& operator *=(const Matrix4& m)
    {
        float tx,ty,tz,tw;
        tx = x * m[0]  + y * m[1]  + z * m[2]  + w * m[3] ;
        ty = x * m[4]  + y * m[5]  + z * m[6]  + w * m[7] ;
        tz = x * m[8]  + y * m[9]  + z * m[10] + w * m[11];
        tw = x * m[12] + y * m[13] + z * m[15] + w * m[15];

        x = tx;
        y = ty;
        z = tz;
        w = tw;
        return *this;
    }

    
    Vector4 operator /(float f) const
    {
        float invf=1.0f/f;
        return Vector4(x*invf, y*invf, z*invf, w*invf);
    };
    
    // dot product
    float operator *(const Vector4& v) const
    {
        return (x*v.x + y*v.y + z*v.z + w*v.w);
    };
    
    // normalize
    Vector4 normalize(void) const
    {
        float l = length();
        return (Vector4(*this) /= l);
    }
};
} // end namespace BOB
#endif /* _VECTOR4_H_ */
