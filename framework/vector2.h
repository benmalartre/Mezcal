//--------------------------------------------------------
// VECTOR2
//--------------------------------------------------------
#ifndef _VECTOR2_H_
#define _VECTOR2_H_

#include <math.h>

namespace BOB{
class Vector2{
public:
    float x;
    float y;
    // constructor
    Vector2(float X, float Y)
    {
        x=X;
        y=Y;
    };
    
    // copy constructor
    Vector2(const Vector2& other)
    {
        x=other.x;
        y=other.y;
    };
    
    // empty constructor
    Vector2()
    {
        x=0.0f;
        y=0.0f;
    };
    
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
    Vector2& operator =(const Vector2& v)
    {
        x=v.x;
        y=v.y;
        return *this;
    };
    
    // add op
    Vector2& operator +=(const Vector2& v)
    {
        x+=v.x;
        y+=v.y;
        return *this;
    };
    
    // in-place subtract op
    Vector2& operator -=(const Vector2& v)
    {
        x-=v.x;
        y-=v.y;
        return *this;
    };
    
    // in-place multiply op
    Vector2& operator *=(float f)
    {
        x*=f;
        y*=f;
        return *this;
    };
    
    // in-place divide op
    Vector2& operator /=(float f)
    {
        float invf = 1.0f/f;
        x*=invf;
        y*=invf;
        return *this;
    };
    
    // normalize in place
    Vector2& normalizeInPlace(void)
    {
        return (*this /= length());
    };
    
    // length
    float length()
    {
        return(sqrt(x*x + y*y));
    };
    
    // length squared
    float lengthSquared()
    {
        return(x*x + y*y);
    };
    
    // negate op
    Vector2 operator -(void) const
    {
        return Vector2(-x, -y);
    };
    
    // add op
    Vector2 operator +(const Vector2& v) const
    {
        return Vector2(x+v.x, y+v.y);
    };
    
    // subtract op
    Vector2 operator -(const Vector2& v) const
    {
        return Vector2(x-v.x, y-v.y);
    };
    
    // scale op
    Vector2 operator *(float f) const
    {
        return Vector2(x*f ,y*f);
    };

	// divide by scalar op
    Vector2 operator /(float f) const
    {
        float invf=1.0f/f;
        return Vector2(x*invf, y*invf);
    };
    
    // dot product op
    float operator *(const Vector2& v) const
    {
        return (x*v.x + y*v.y);
    };
    float dot(const Vector2& v) const
    {
        return (*this*v);
    };
    
    // cross product op
    Vector2 operator ^(const Vector2& v) const
    {
        return Vector2(x * v.y - y * v.x);
    }
    Vector2 cross(const Vector2& v) const
    {
        return (*this^v);
    }
    
    // normalize
    Vector2 normalize() const
    {
        return (Vector2(*this) /= (float)sqrt(x*x + y*y));
    }

};
} // end namespace BOB
#endif /* _VECTOR2_H_ */
