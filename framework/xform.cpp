//-------------------------------------------------------
// XFORM IMPLEMENTATION
//-------------------------------------------------------
#include "xform.h"
#include "utils.h"

namespace BOB{
    void XForm::setMatrixFromSRT()
    {
        Vector3 x(scl.x,0.0f,0.0f);
        Vector3 y(0.0f,scl.y,0.0f);
        Vector3 z(0.0f,0.0f,scl.z);
        
        rotateVector(x,rot);
        rotateVector(y,rot);
        rotateVector(z,rot);
        
        m.v[0] = x.x;
        m.v[1] = x.y;
        m.v[2] = x.z;
        m.v[3] = 0.0f;
        m.v[4] = y.x;
        m.v[5] = y.y;
        m.v[6] = y.z;
        m.v[7] = 0.0f;
        m.v[8] = z.x;
        m.v[9] = z.y;
        m.v[10] = z.z;
        m.v[11] = 0.0f;
        m.v[12] = pos.x;
        m.v[13] = pos.y;
        m.v[14] = pos.z;
        m.v[15] = 1.0f;
    }
    
    void XForm::setSRTFromMatrix()
    {
        // Extract the x,y,z axes
        Vector3 x(m.v[0],m.v[1],m.v[2]);
        Vector3 y(m.v[4],m.v[5],m.v[6]);
        Vector3 z(m.v[8],m.v[9],m.v[10]);
        
        // Set Scale
        scl.x = x.length();
        scl.y = y.length();
        scl.z = z.length();

        // Set Quaternion
        float tr = m.v[0] + m.v[5] + m.v[10];
        float S;
        if(tr > 0)
        {
            S = sqrt(tr+1.0) * 2;
            rot.w = 0.25 * S;
            rot.x = (m.v[9] - m.v[6]) / S;
            rot.y = (m.v[2] - m.v[8]) / S;
            rot.z = (m.v[4] - m.v[1]) / S;
        }
        else if((m.v[0] > m.v[5])&&(m.v[0] > m.v[10]))
        {
            S = sqrt(1.0 + m.v[0] - m.v[5] - m.v[10]) * 2;
            rot.w = (m.v[9] - m.v[6]) / S;
            rot.x = 0.25 * S;
            rot.y = (m.v[1] + m.v[4]) / S;
            rot.z = (m.v[2] + m.v[8]) / S;
        }
        else if(m.v[5] > m.v[10])
        {
            S = sqrt(1.0 + m.v[5] - m.v[0] - m.v[10]) * 2;
            rot.w = (m.v[2] - m.v[8]) / S;
            rot.x = (m.v[1] + m.v[4]) / S;
            rot.y = 0.25 * S;
            rot.z = (m.v[6] + m.v[9]) / S;
        }
        else
        {
            S = sqrt(1.0 + m.v[10] - m.v[0] - m.v[5]) * 2;
            rot.w = (m.v[4] - m.v[1]) / S;
            rot.x = (m.v[2] + m.v[8]) / S;
            rot.y = (m.v[6] + m.v[9]) / S;
            rot.z = 0.25 * S;
        }
        
        // finally set the position!
        pos.x = m.v[12];
        pos.y = m.v[13];
        pos.z = m.v[14];
    }
    
    // scale
    void XForm::setScale(const Vector3& s)
    {
        scl = s;
        srtDirty = true;
    }
    
    // position
    void XForm::setPosition(const Vector3& p)
    {
        pos = p;
        srtDirty = true;
    }
    
    // rotation (quaternion)
    void XForm::setRotation(const Quaternion& q)
    {
        rot = q;
        srtDirty = true;
    }
    
    // rotation (euler)
    void XForm::setRotationFromEuler(float x, float y, float z)
    {
        srtDirty = true;
    }
    
    // matrix
    void XForm::setMatrix(const Matrix4& matrix)
    {
        m = matrix;
        matrixDirty = true;
    }

    // update matrix (world)
    Matrix4 XForm::update(const XForm& parentXfo)
    {
        setMatrixFromSRT();
        Matrix4 tm = m*parentXfo.m;
        matrixDirty = false;
        srtDirty = false;
        return tm;
    }
}//end namespace BOB

