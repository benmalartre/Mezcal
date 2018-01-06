//--------------------------------------------------------
// MATRIX3
//--------------------------------------------------------
#ifndef _MATRIX3_H_
#define _MATRIX3_H_

#include <math.h>
#include "common.h"

namespace BOB{
class Matrix3{
public:
    float v[9];
    Matrix3()
    {
        identity();
    }
    Matrix3(float m00, float m01, float m02,
            float m10, float m11, float m12,
            float m20, float m21, float m22)
    {
        v[0] = m00;
        v[1] = m01;
        v[2] = m02;
        v[3] = m10;
        v[4] = m11;
        v[5] = m12;
        v[6] = m20;
        v[7] = m21;
        v[8] = m22;
    };
    
    Matrix3(const Matrix3& other){for(unsigned ii=0;ii<9;ii++)v[ii] = other[ii];};

    ~Matrix3(){};
    void identity()
    {
        v[0] = 1.0f;
        v[1] = 0.0f;
        v[2] = 0.0f;
        v[3] = 0.0f;
        v[4] = 1.0f;
        v[5] = 0.0f;
        v[6] = 0.0f;
        v[7] = 0.0f;
        v[8] = 1.0f;
    }
    float& operator [](int k){return v[k];}
    const float& operator [](int k) const{return v[k];}
    
    // multiply in place
    Matrix3& operator *(const Matrix3& other)
    {
        unsigned x,y;
        for(y=0;y<3;y++)
        {
            for(x=0;x<3;x++)
            {
                v[3*x+y] = other.v[3*x]*v[y] + other[3*x+1]*v[y+3] + other.v[3*x+2]*v[y+6];
            }
        }
        return *this;
    }
    
    // multiply
    Matrix3 operator *(const Matrix3& other) const
    {
        Matrix3 m;
        unsigned x,y;
        for(y=0;y<3;y++)
        {
            for(x=0;x<3;x++)
            {
                m.v[3*x+y] = other.v[3*x]*v[y] + other[3*x+1]*v[y+3] + other.v[3*x+2]*v[y+6];
            }
        }
        return m;
    }
    
    // compute inverse
    bool computeInverse(const Matrix3& other, bool transpose=false)
    {
        unsigned x, y, z, cnt;
        float fSys[3][6];
        float fTemp;
        
        // initialize fSys array
        for(x=0;x<3;x++)
        {
            for(y=0;y<3;y++)
            {
                fSys[x][y] = other[x*3+y];
                if(x==y)fSys[x][y+3] = 1.0f;
                else fSys[x][y+3] = 0.0f;
            }
        }
        
        // compute inverse
        for(y=0;y<3;y++)
        {
            if(fabs(fSys[y][y])<0.0000001)
            {
                cnt = y+1;
                for(x=0;x<3;x++)
                {
                    if(fabs(fSys[x][y])<0.0000001) cnt++;
                }
                // check singular matrix
                if(cnt == 3)return false;
                else
                {
                    for(z=0;z<6;z++)
                    {
                        fTemp = fSys[x][z];
                        fSys[x][z] = fSys[y][z];
                        fSys[y][z] = fTemp;
                    }
                }
            }
            if(fSys[y][y]!=0.0f)fTemp = 1.0f / fSys[y][y];
            for(x=0;x<6;x++) fSys[y][x] *= fTemp;
            
            for(x=0;x<3;x++)
            {
                if(x!=y)
                {
                    fTemp = -fSys[x][y];
                    for(z=0;z<6;z++)
                    {
                        fSys[x][z] += (fSys[y][z]*fTemp);
                    }
                }
            }
        }
        
        // copy result
        if(transpose)
        {
            for(x=0;x<3;x++)
            {
                for(y=0;y<3;y++)
                {
                    v[x+y*3] = fSys[x][y+3];
                }
            }
        }
        else
        {
            for(x=0;x<3;x++)
            {
                for(y=0;y<3;y++)
                {
                    v[x*3+y] = fSys[x][y+3];
                }
            }
        }
        return true;
    }
    
    Matrix3& inverse()
    {
        computeInverse(*this);
        return *this;
    };
    
    Matrix3 inverse() const
    {
        Matrix3 m;
        m.computeInverse(*this);
        return m;
    };
    
    Matrix3& setFromTwoVectors(const Vector3& dir, const Vector3& up)
    {
        Vector3 forward, side, up2;
        forward = dir.normalize();
        up2 = up.normalize();
        
        side = forward.cross(up2);
        up2 = side.cross(forward);
        
        side.normalize();
        up2.normalize();
        
        v[0] = side.x;
        v[1] = side.y;
        v[2] = side.z;
        v[3] = up.x;
        v[4] = up.y;
        v[5] = up.z;
        v[6] = -forward.x;
        v[7] = -forward.y;
        v[8] = -forward.z;
        return *this;
    }
    
    Matrix3 setFromTwoVectors(const Vector3& dir, const Vector3& up) const
    {
        Matrix3 m;
        m.setFromTwoVectors(dir, up);
        return m;
    }
    
    void setRow(unsigned index, const Vector3& value)
    {
        v[index*3+0] = value.x;
        v[index*3+1] = value.y;
        v[index*3+2] = value.z;
    }
    
    Quaternion getQuaternion(bool transpose=false)
    {
        float t, s;
        Quaternion q;
        if(transpose)
        {
            t = 1+v[0]+v[4]+v[8];
            if(t >0.00000001)
            {
                s = sqrt(t)*2;
                q.x = (v[7]-v[5])/s;
                q.y = (v[2]-v[6])/s;
                q.z = (v[3]-v[1])/s;
                q.w = 0.25 * s;
            }
            else
            {
                if(v[0]>v[4] && v[0]>v[8])
                {
                    s = sqrt(1+ v[0] - v[4] - v[8])*2;
                    q.x = 0.25 * s;
                    q.y = (v[3] + v[1])/s;
                    q.z = (v[2] + v[6])/s;
                    q.w = (v[7] - v[5])/s;
                }
                else if(v[4]>v[8])
                {
                    s = sqrt(1+ v[4] - v[0] - v[8])*2;
                    q.x = (v[3] + v[1])/s;
                    q.y = 0.25 * s;
                    q.z = (v[7] + v[5])/s;
                    q.w = (v[2] - v[6])/s;
                }
				else
                {
                    s = sqrt(1+ v[8] - v[0] - v[4])*2;
                    q.x = (v[2] + v[6])/s;
                    q.y = (v[7] + v[5])/s;
                    q.z = 0.25 * s;
                    q.w = (v[3] - v[1])/s;
                }
            }
        }
        else
        {
            t = 1+v[0]+v[4]+v[8];
            if(t >0.00000001)
            {
                s = sqrt(t)*2;
                q.x = (v[5]-v[7])/s;
                q.y = (v[6]-v[2])/s;
                q.z = (v[1]-v[3])/s;
                q.w = 0.25 * s;
            }
            else
            {
                if(v[0]>v[4] && v[0]>v[8])
                {
                    s = sqrt(1+ v[0] - v[4] - v[8])*2;
                    q.x = 0.25 * s;
                    q.y = (v[1] + v[3])/s;
                    q.z = (v[6] + v[2])/s;
                    q.w = (v[5] - v[7])/s;
                }
                else if(v[4]>v[8])
                {
                    s = sqrt(1+ v[4] - v[0] - v[8])*2;
                    q.x = (v[1] + v[3])/s;
                    q.y = 0.25 * s;
                    q.z = (v[5] + v[7])/s;
                    q.w = (v[6] - v[2])/s;
                }
                else
                {
                    s = sqrt(1+ v[8] - v[0] - v[4])*2;
                    q.x = (v[6] + v[2])/s;
                    q.y = (v[5] + v[7])/s;
                    q.z = 0.25 * s;
                    q.w = (v[1] - v[3])/s;
                }
            }
        }
        return q;
    }
    
    float getDeterminant() const
    {
        return (v[0] * v[4] * v[8] +
                v[1] * v[5] * v[6] +
                v[2] * v[3] * v[7] -
                v[0] * v[5] * v[7] -
                v[1] * v[3] * v[8] -
                v[2] * v[4] * v[6]);
    }

    
    float getHandedness() const
    {
        return SIGN(getDeterminant());
    }
};
    
}//end namespace BOB
#endif /* _MATRIX3_H_ */
