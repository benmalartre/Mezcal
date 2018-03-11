//--------------------------------------------------------
// MATRIX4
//--------------------------------------------------------
#ifndef _MATRIX4_H_
#define _MATRIX4_H_

#include <math.h>
#include "quaternion.h"

class Vector3;
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
        unsigned i, j, k;
        float fSys[4][8];
        float fTemp;
        
        // initialize fSys array
        for(i=0;i<4;i++)
        {
            for(j=0;j<4;j++)
            {
                fSys[i][j] = other[i*4+j];
                if(i==j)fSys[i][j+4] = 1.0f;
                else fSys[i][j+4] = 0.0f;
            }
        }
        
        // Compute inverse:
        for(j=0; j<4; j++)
        {
        	if(fabs(fSys[j][j]) < 0.0000001)
            {
           		for(i=j + 1; (i<4) && (fabs(fSys[i][j]) < 0.0000001 ); i++)
                {};
                if(i==4)
                {
                	return(false);
                }
                else
                {
                	for(k=0; k<8; k++)
                    {
                    	fTemp = fSys[i][k];
                        fSys[i][k] = fSys[j][k];
                        fSys[j][k] = fTemp;
                    }
                }
            }
            fTemp = 1.0f / fSys[j][j];
            for(i=0; i<8; i++)
            {
            	fSys[j][i] *= fTemp;
            }
            for(i=0; i<4; i++)
            {
            	if(i != j)
                {
                	fTemp = - fSys[i][j];
                    for(k=0; k<8; k++)
                    {
                    	fSys[i][k] += fSys[j][k] * fTemp;
                    }
                }
            }
        }
        /*
        // compute inverse
        for(j=0;j<4;j++)
        {
            if(fabs(fSys[j][j])<0.0000001)
            {
                cnt = j+1;
                for(i=j+1;i<4;i++)
                {
                    if(fabs(fSys[i][j])<0.0000001) cnt++;
                }
                // check singular matrix
                if(cnt == 4)return false;
                else
                {
                    for(k=0;k<8;k++)
                    {
                        fTemp = fSys[i][k];
                        fSys[i][k] = fSys[j][k];
                        fSys[j][k] = fTemp;
                    }
                }
            }
            if(fSys[j][j]!=0.0f)fTemp = 1.0f / fSys[j][j];
            for(i=0;i<8;i++) fSys[j][i] *= fTemp;
            
            for(i=0;i<4;i++)
            {
                if(i!=j)
                {
                    fTemp = -fSys[i][j];
                    for(k=0;k<8;k++)
                    {
                        fSys[i][k] += (fSys[j][k]*fTemp);
                    }
                }
            }
        }
         */
        
        // copy result
        if(transpose)
        {
            for(i=0;i<4;i++)
            {
                for(j=0;j<4;j++)
                {
                    v[i+j*4] = fSys[i][j+4];
                }
            }
        }
        else
        {
            for(i=0;i<4;i++)
            {
                for(j=0;j<4;j++)
                {
                    v[i*4+j] = fSys[i][j+4];
                }
            }
        }
        return true;
    }
    
    Matrix4& inverseInPlace()
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
    
    Matrix4& transposeInPlace()
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
            S = sqrtf(tr+1.0f)*2.0f;
            Q.w = 0.25f * S;
            Q.x = (v[9] - v[6]) / S;
            Q.y = (v[2] - v[8]) / S;
            Q.z = (v[4] - v[1]) / S;
        }
        else if(v[0] > v[5] && v[0] > v[10])
        {
            S = sqrtf(1.0f + v[0] - v[5] - v[10]) * 2.0f;
            Q.w = (v[9] - v[6]) / S;
            Q.x = 0.25f * S;
            Q.y = (v[1] + v[4]) / S;
            Q.z = (v[2] + v[8]) / S;
        }
        else if(v[5] > v[10])
        {
            S = sqrtf(1.0f + v[5] - v[0] - v[10]) * 2.0f;
            Q.w = (v[2] - v[8]) / S;
            Q.x = (v[1] + v[4]) / S;
            Q.y = 0.25f * S;
            Q.z = (v[6] + v[9]) / S;
        }
        else
        {
            S = sqrtf(1.0f + v[10] - v[0] - v[5]) * 2.0f;
            Q.w = (v[4] - v[1]) / S;
            Q.x = (v[2] + v[8]) / S;
            Q.y = (v[6] + v[9]) / S;
            Q.z = 0.25 * S;
        }
        return Q;
    }
    
    Matrix4& setRotate(float ax, float ay, float az , float angle)
    {
        v[0] = 1.0f - 2.0f * (ay * ay + az * az);
        v[1] =       2.0f * (ax * ay + az *    angle);
        v[2] =       2.0f * (az * ax - ay *    angle);
        v[3] = 0.0f;
        
        v[4] =       2.0f * (ax * ay - az *    angle);
        v[5] = 1.0f - 2.0f * (az * az + ax * ax);
        v[6] =       2.0f * (ay * az + ax *    angle);
        v[7] = 0.0f;
        
        v[8] =       2.0f * (az * ax + ay *    angle);
        v[9] =       2.0f * (ay * az - ax *    angle);
        v[10] = 1.0f - 2.0f * (ay * ay + ax* ax);
        v[11] = 0.0f;
        
        v[12] = 0.0f;
        v[13] = 0.0f;
        v[14] = 0.0f;
        v[15] = 1.0f;
        
        return *this;
    }
    
    
    float getDeterminant() const
    {
        return (- v[3] * getDeterminant3(1, 2, 3, 0, 1, 2)
                + v[7] * getDeterminant3(0, 2, 3, 0, 1, 2)
                - v[11] * getDeterminant3(0, 1, 3, 0, 1, 2)
                + v[15] * getDeterminant3(0, 1, 2, 0, 1, 2));
    }
    
    float getDeterminant3(size_t row1, size_t row2, size_t row3,
                                 size_t col1, size_t col2, size_t col3) const
    {
        return (v[row1*4+col1] * v[row2*4+col2] * v[row3*4+col3] +
                v[row1*4+col2] * v[row2*4+col3] * v[row3*4+col1] +
                v[row1*4+col3] * v[row2*4+col1] * v[row3*4+col2] -
                v[row1*4+col1] * v[row2*4+col3] * v[row3*4+col2] -
                v[row1*4+col2] * v[row2*4+col1] * v[row3*4+col3] -
                v[row1*4+col3] * v[row2*4+col2] * v[row3*4+col1]);
    }
    
    float getHandedness() const
    {
        return SIGN(getDeterminant());
    }
    
    bool orthogonalizeBasis(float *tx, float *ty, float *tz,
                         bool normalize, float eps)
    {
        GfVec3f ax,bx,cx,ay,by,cy,az,bz,cz;
        
        if (normalize) {
            GfNormalize(tx);
            GfNormalize(ty);
            GfNormalize(tz);
            ax = *tx;
            ay = *ty;
            az = *tz;
        } else {
            ax = *tx;
            ay = *ty;
            az = *tz;
            ax.Normalize();
            ay.Normalize();
            az.Normalize();
        }
        
        /* Check for colinear vectors. This is not only a quick-out: the
         * error computation below will evaluate to zero if there's no change
         * after an iteration, which can happen either because we have a good
         * solution or because the vectors are colinear.   So we have to check
         * the colinear case beforehand, or we'll get fooled in the error
         * computation.
         */
        if (GfIsClose(ax,ay,eps) || GfIsClose(ax,az,eps) || GfIsClose(ay,az,eps)) {
            return false;
        }
        
        const int MAX_ITERS = 20;
        int iter;
        for (iter = 0; iter < MAX_ITERS; ++iter) {
            bx = *tx;
            by = *ty;
            bz = *tz;
            
            bx -= GfDot(ay,bx) * ay;
            bx -= GfDot(az,bx) * az;
            
            by -= GfDot(ax,by) * ax;
            by -= GfDot(az,by) * az;
            
            bz -= GfDot(ax,bz) * ax;
            bz -= GfDot(ay,bz) * ay;
            
            cx = 0.5*(*tx + bx);
            cy = 0.5*(*ty + by);
            cz = 0.5*(*tz + bz);
            
            if (normalize) {
                cx.Normalize();
                cy.Normalize();
                cz.Normalize();
            }
            
            GfVec3f xDiff = *tx - cx;
            GfVec3f yDiff = *ty - cy;
            GfVec3f zDiff = *tz - cz;
            
            double error =
            GfDot(xDiff,xDiff) + GfDot(yDiff,yDiff) + GfDot(zDiff,zDiff);
            
            // error is squared, so compare to squared tolerance
            if (error < GfSqr(eps))
                break;
            
            *tx = cx;
            *ty = cy;
            *tz = cz;
            
            ax = *tx;
            ay = *ty;
            az = *tz;
            
            if (!normalize) {
                ax.Normalize();
                ay.Normalize();
                az.Normalize();
            }
        }
        
        return iter < MAX_ITERS;
    }
    
    bool orthonormalize(bool issueWarning)
    {
        // orthogonalize and normalize row vectors
        float r0[3] = {v[0],v[1],v[2]};
        float r1[3] = {v[4],v[5],v[6]};
        float r2[3] = {v[8],v[9],v[10]};
        bool result = orthogonalizeBasis(&r0, &r1, &r2, true);
        v[0] = r0[0];
        v[1] = r0[1];
        v[2] = r0[2];
        v[4] = r1[0];
        v[5] = r1[1];
        v[6] = r1[2];
        v[8] = r2[0];
        v[9] = r2[1];
        v[10] = r2[2];
        
        // divide out any homogeneous coordinate - unless it's zero
        if (v[15] != 1.0 && fabsf(v[15])>0.0000001f)
        {
            v[12] /= v[15];
            v[13] /= v[15];
            v[14] /= v[15];
            v[15] = 1.0;
        }
        
        /*
        if (!result && issueWarning)
            TF_WARN("OrthogonalizeBasis did not converge, matrix may not be "
                    "orthonormal.");
        */
        
        return result;
    }
    
    Matrix4 getOrthonormalized(bool issueWarning) const
    {
        Matrix4 result = *this;
        result.orthonormalize(issueWarning);
        return result;
    }
};
    
}//end namespace BOB
#endif /* _MATRIX4_H_ */
