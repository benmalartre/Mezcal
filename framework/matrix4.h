//--------------------------------------------------------
// MATRIX4
//--------------------------------------------------------
#ifndef _MATRIX4_H_
#define _MATRIX4_H_

#include <math.h>
#include "quaternion.h"

namespace BOB{
class Matrix4{
public:
    float v[16];
    Matrix4()
    {
        identity();
    }
    Matrix4(float m00, float m01, float m02, float m03,
            float m10, float m11, float m12, float m13,
            float m20, float m21, float m22, float m23,
            float m30, float m31, float m32, float m33)
    {
        v[0] = m00;
        v[1] = m01;
        v[2] = m02;
        v[3] = m03;
        v[4] = m10;
        v[5] = m11;
        v[6] = m12;
        v[7] = m13;
        v[8] = m20;
        v[9] = m21;
        v[10] = m22;
        v[11] = m23;
        v[12] = m30;
        v[13] = m31;
        v[14] = m32;
        v[15] = m33;
    };
    
    Matrix4(const Matrix4& other){for(unsigned ii=0;ii<16;ii++)v[ii] = other[ii];};

    ~Matrix4(){};
    void identity()
    {
        v[0] = 1.0f;
        v[1] = 0.0f;
        v[2] = 0.0f;
        v[3] = 0.0f;
        v[4] = 0.0f;
        v[5] = 1.0f;
        v[6] = 0.0f;
        v[7] = 0.0f;
        v[8] = 0.0f;
        v[9] = 0.0f;
        v[10] = 1.0f;
        v[11] = 0.0f;
        v[12] = 0.0f;
        v[13] = 0.0f;
        v[14] = 0.0f;
        v[15] = 1.0f;
    }
    float& operator [](int k){return v[k];}
    const float& operator [](int k) const{return v[k];}
    
    // multiply in place
    Matrix4& operator *(const Matrix4& other)
    {
        float tmp1, tmp2, tmp3, tmp4;
        unsigned x, y;
        for(y=0;y<4;y++)
        {
            tmp1 = v[y];
            tmp2 = v[y+4];
            tmp3 = v[y+8];
            tmp4 = v[y+12];
            for(x=0;x<4;x++)
            {
                v[4*x+y] = other.v[4*x]*tmp1 + other.v[4*x+1]*tmp2 + other.v[4*x+2]*tmp3 + other.v[4*x+3]*tmp4;
            }
        }
        return *this;
    }
    
    // multiply
    Matrix4 operator *(const Matrix4& other) const
    {
        Matrix4 m;
        unsigned x,y;
        for(y=0;y<4;y++)
        {
            for(x=0;x<4;x++)
            {
                m.v[4*x+y] = other.v[4*x]*v[y] + other[4*x+1]*v[y+4] + other.v[4*x+2]*v[y+8] +other[4*x+3]*v[y+12];
            }
        }
        return m;
    }
    
    // compute inverse
    bool computeInverse(const Matrix4& other, bool transpose=false)
    {
        unsigned x, y, z, cnt;
        float fSys[4][8];
        float fTemp;
        
        // initialize fSys array
        for(x=0;x<4;x++)
        {
            for(y=0;y<4;y++)
            {
                fSys[x][y] = other[x*4+y];
                if(x==y)fSys[x][y+4] = 1.0f;
                else fSys[x][y+4] = 0.0f;
            }
        }
        
        // compute inverse
        for(y=0;y<4;y++)
        {
            if(fabs(fSys[y][y])<0.0000001)
            {
                cnt = y+1;
                for(x=0;x<4;x++)
                {
                    if(fabs(fSys[x][y])<0.0000001) cnt++;
                }
                // check singular matrix
                if(cnt == 4)return false;
                else
                {
                    for(z=0;z<8;z++)
                    {
                        fTemp = fSys[x][z];
                        fSys[x][z] = fSys[y][z];
                        fSys[y][z] = fTemp;
                    }
                }
            }
            if(fSys[y][y]!=0.0f)fTemp = 1.0f / fSys[y][y];
            for(x=0;x<8;x++) fSys[y][x] *= fTemp;
            
            for(x=0;x<4;x++)
            {
                if(x!=y)
                {
                    fTemp = -fSys[x][y];
                    for(z=0;z<8;z++)
                    {
                        fSys[x][z] += (fSys[y][z]*fTemp);
                    }
                }
            }
        }
        
        // copy result
        if(transpose)
        {
            for(x=0;x<4;x++)
            {
                for(y=0;y<4;y++)
                {
                    v[x+y*4] = fSys[x][y+4];
                }
            }
        }
        else
        {
            for(x=0;x<4;x++)
            {
                for(y=0;y<4;y++)
                {
                    v[x*4+y] = fSys[x][y+4];
                }
            }
        }
        return true;
    }
    
    Matrix4& inverse()
    {
        computeInverse(*this);
        return *this;
    };
    
    Matrix4 inverse() const
    {
        Matrix4 m;
        m.computeInverse(*this);
        return m;
    };
    
    Matrix4& transpose()
    {
        unsigned x, y;
        Matrix4 o(*this);
        for(x=0;x<4;x++)
        {
            for(y=0;y<4;y++)
            {
                v[x+y*4] = o[x*4+y];
            }
        }
        return *this;
    };
    
    Matrix4 transpose() const
    {
        unsigned x, y;
        Matrix4 o;
        for(x=0;x<4;x++)
        {
            for(y=0;y<4;y++)
            {
                o.v[x+y*4] = v[x*4+y];
            }
        }
        return o;
    };
    
    Quaternion getQuaternion()
    {
        Quaternion Q;
        float tr = v[0] + v[5] + v[10];
        float S;
        
        if(tr>0.0f)
        {
            S = sqrt(tr+1.0f)*2.0f;
            Q.w = 0.25f * S;
            Q.x = (v[9] - v[6]) / S;
            Q.y = (v[2] - v[8]) / S;
            Q.z = (v[4] - v[1]) / S;
        }
        else if(v[0] > v[5] && v[0] > v[10])
        {
            S = sqrt(1.0f + v[0] - v[5] - v[10]) * 2.0f;
            Q.w = (v[9] - v[6]) / S;
            Q.x = 0.25f * S;
            Q.y = (v[1] + v[4]) / S;
            Q.z = (v[2] + v[8]) / S;
        }
        else if(v[5] > v[10])
        {
            S = sqrt(1.0f + v[5] - v[0] - v[10]) * 2.0f;
            Q.w = (v[2] - v[8]) / S;
            Q.x = (v[1] + v[4]) / S;
            Q.y = 0.25f * S;
            Q.z = (v[6] + v[9]) / S;
        }
        else
        {
            S = sqrt(1.0f + v[10] - v[0] - v[5]) * 2.0f;
            Q.w = (v[4] - v[1]) / S;
            Q.x = (v[2] + v[8]) / S;
            Q.y = (v[6] + v[9]) / S;
            Q.z = 0.25 * S;
        }
        return Q;
    }
};
    
}//end namespace BOB
#endif /* _MATRIX4_H_ */
