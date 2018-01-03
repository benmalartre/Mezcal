//--------------------------------------------------------
// QUATERNION
//--------------------------------------------------------
#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include <math.h>
#include <iostream>
#include "common.h"

namespace BOB{
class Quaternion{
    public:
    float x;
    float y;
    float z;
    float w;

    Quaternion()
    {
        x=0.0f;
        y=0.0f;
        z=0.0f;
        w=1.0f;
    };

    Quaternion(float in_x, float in_y, float in_z, float in_w)
    {
        x = in_x;
        y = in_y;
        z = in_z;
        w = in_w;
    };
    
    Quaternion(const Quaternion& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
    }
    
    void set(float in_x, float in_y, float in_z, float in_w)
    {
        x = in_x;
        y = in_y;
        z = in_z;
        w = in_w;
    };
    
    Quaternion& operator -(void)
    {
        x = -x;
        y = -y;
        z = -z;
        w = -w;
        return *this;
    }
    
    Quaternion operator -(const Quaternion& other) const
    {
        return Quaternion(-other.x, -other.y, -other.z, -other.w);
    }
    
    float operator *(const Quaternion& other) const
    {
        return (x*other.x + y*other.y + z*other.z + w*other.w);
    }
    
    Quaternion conjugate(void) const
    {
        return Quaternion(-x,-y,-z,w);
    }
    
    Quaternion& conjugateInPlace(void)
    {
        x = -x;
        y = -y;
        z = -z;
        return *this;
    }
    
    Quaternion multiply(const Quaternion& o) const
    {
        float _x,_y,_z,_w;
        _x = w*o.x + x*o.w + y*o.z + z*o.y;
        _y = w*o.y + y*o.w + z*o.x + x*o.z;
        _z = w*o.z + z*o.w + x*o.y + y*o.x;
        _w = w*o.w + x*o.x + y*o.y + z*o.z;
        return Quaternion(_x,_y,_z,_w);
    }
    
    Quaternion& multiply(const Quaternion& q1, const Quaternion& q2)
    {
        float _x,_y,_z,_w;
        _x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z + q1.z*q2.y;
        _y = q1.w*q2.y + q1.y*q2.w + q1.z*q2.x + q1.x*q2.z;
        _z = q1.w*q2.z + q1.z*q2.w + q1.x*q2.y + q1.y*q2.x;
        _w = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
        x=_x;
        y=_y;
        z=_z;
        w=_w;
        return *this;
    }
    
    void linearInterpolate(Quaternion& q1, Quaternion& q2,float blend)
    {
        x = (1-blend) * q1.x + blend * q2.x;
        y = (1-blend) * q1.y + blend * q2.y;
        z = (1-blend) * q1.z + blend * q2.z;
        w = (1-blend) * q1.w + blend * q2.w;
    }
    
    void slerp(Quaternion& q1, Quaternion& q2,float blend)
    {
    	if(blend<0)
        {
            x = q1.x;
            y = q1.y;
            z = q1.z;
            w = q1.w;
            
            return;
        }
        else if(blend>1.0)
        {
            x = q2.x;
            y = q2.y;
            z = q2.z;
            w = q2.w;
            
            return;
        }
    
    	float dp = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
   		float theta, st,sut, sout, coeff1, coeff2;
    
        blend *= 0.5;
    
        theta = acosf(dp);
        if(theta<0)theta *= -1;
    
        st = sinf(theta);
        sut = sinf(blend*theta);
        sout = sinf((1-blend)*theta);
        coeff1 = sout/st;
        coeff2 = sut/st;
    
        x = coeff1 * q1.x + coeff2 * q2.x;
        y = coeff1 * q1.y + coeff2 * q2.y;
        z = coeff1 * q1.z + coeff2 * q2.z;
        w = coeff1 * q1.w + coeff2 * q2.w;
    
    }
    
    Quaternion& identity()
    {
        x=0.0f;
        y=0.0f;
        z=0.0f;
        w=1.0f;
        return *this;
    }
    
    Quaternion& setFromAxisAngle(float in_x, float in_y, float in_z, float angle)
    {
        // normalize axis
        float l = sqrtf(in_x*in_x + in_y*in_y + in_z*in_z);
        float invl=1.0f/l;
        float ax = in_x*invl;
        float ay = in_x*invl;
        float az = in_y*invl;
        
        float halfAngle, sinAngle;
        halfAngle = angle*0.5;
        sinAngle = sinf(halfAngle);
        x = ax * sinAngle;
        y = ay * sinAngle;
        z = az * sinAngle;
        w = cosf(halfAngle);
        return *this;
    }
    
    Quaternion& setFromAxisAngle(float* axis, float angle)
    {
        return setFromAxisAngle(axis[0], axis[1], axis[2], angle);
    }
    
    Quaternion& normalizeInPlace()
    {
        float mag2 = x * x + y * y + z * z + w * w;
        if(mag2 == 0.0f) return *this;
        
    	if(fabs(mag2 - 1.0)>0.0001)
        {
            float mag = sqrtf(mag2);
    
            x /= mag;
            y /= mag;
            z /= mag;
            w /= mag;
        }
        return *this;
    }
    
    Quaternion normalize() const
    {
        Quaternion q(*this);
        float mag2 = x * x + y * y + z * z + w * w;
        if(mag2 == 0.0f) return q;
        
        if(fabs(mag2 - 1.0)>0.0001)
        {
            float mag = sqrtf(mag2);
            
            q.x = x/mag;
            q.y = y/mag;
            q.z = z/mag;
            q.w = w/mag;
        }
        return q;
    }
    
    Quaternion& setFromEulerAngles(float pitch, float yaw, float roll)
    {
        float p,y,r;
        p = pitch * DEGREE_TO_RADIAN * 0.5;
        y = yaw * DEGREE_TO_RADIAN * 0.5;
        r = roll * DEGREE_TO_RADIAN * 0.5;
    
        float sinp,siny,sinr,cosp,cosy,cosr;
        sinp = sinf(p);
        siny = sinf(y);
        sinr = sinf(r);
        cosp = cosf(p);
        cosy = cosf(y);
        cosr = cosf(r);
    
        x = sinr * cosp * cosy - cosr * sinp * siny;
        y = cosr * sinp * cosy + sinr * cosp * siny;
        z = cosr * cosp * siny - sinr * sinp * cosy;
        w = cosr * cosp * cosy + sinr * sinp * siny;
        normalize();
        
        return *this;
    }
    
    ~Quaternion(){};
};
}// end namespace BOB
#endif /* _QUATERNION_H_ */
