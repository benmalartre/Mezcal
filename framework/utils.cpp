//--------------------------------------------------------
// UTILS
//--------------------------------------------------------
#include "utils.h"

namespace BOB{
    /*
    // Execute a system command
    int executeCommand(const char* cmd)
    {

        FILE *fp = popen(cmd, "r");
        char buf[256];
        cout << "##########################################" << endl;
        while (fgets(buf, 256, fp)) {
            cout << buf;
        }
        cout << "##########################################" << endl;
        cout << "DONE!!!" << endl;
        return EXIT_SUCCESS;

        return 0;
    }
    */
    
    Vector3 linearInterpolate(const Vector3& A, const Vector3& B, float blend)
    {
        Vector3 interpolated;
        interpolated.x = LINEAR_INTERPOLATE(A.x,B.x,blend);
        interpolated.y = LINEAR_INTERPOLATE(A.y,B.y,blend);
        interpolated.z = LINEAR_INTERPOLATE(A.z,B.z,blend);
        return interpolated;
    }
    
    Vector3 cubicInterpolate(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& D, float blend)
    {
        Vector3 interpolated;
        interpolated.x = CUBIC_INTERPOLATE(A.x,B.x,C.x,D.x,blend);
        interpolated.y = CUBIC_INTERPOLATE(A.y,B.y,C.y,D.y,blend);
        interpolated.z = CUBIC_INTERPOLATE(A.z,B.z,C.z,D.z,blend);
        return interpolated;
    }
    
    Vector3 hermiteInterpolate(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& D, float blend, float tension, float bias)
    {
        Vector3 interpolated;
        interpolated.x = HERMITE_INTERPOLATE(A.x,B.x,C.x,D.x,blend, tension, bias);
        interpolated.y = HERMITE_INTERPOLATE(A.y,B.y,C.y,D.y,blend, tension, bias);
        interpolated.z = HERMITE_INTERPOLATE(A.z,B.z,C.z,D.z,blend, tension, bias);
        return interpolated;
    }
    
    float quaternionDot(Quaternion& q1, Quaternion& q2)
    {
        return 0.0f;
    };
    
    Quaternion quaternionSlerp(Quaternion& qA, Quaternion& qB, float t)
    {
        return qA;
    };
    Quaternion quaternionSlerp2(Quaternion qA, Quaternion qB, double blend)
    {
        return qA;
    };
    Quaternion eulerToQuaternion(double x, double y, double z)
    {
        return Quaternion(x,y,z,1.0f);
    };
    Quaternion rotationBetweenVectors(const Vector3& start, const Vector3& dest)
    {
        return Quaternion();
    }
    Quaternion quaternionLookAt(const Vector3& dir, const Vector3& up)
    {
        Matrix3 m;
        m.setFromTwoVectors(dir,up);
        return m.getQuaternion(false);
    }
    Quaternion quaternionFromAxisAngle(const Vector3& axis, float angle)
    {
        Quaternion q;
        Vector3 n = axis.normalize();
        float halfAngle,sinAngle;
        
        halfAngle = angle*0.5f;
        sinAngle = sin(halfAngle);
        q.x = n.x * sinAngle;
        q.y = n.y * sinAngle;
        q.z = n.z * sinAngle;
        q.w = cos(halfAngle);
        return q;
    }
    
    Matrix4 matrixFromTwoVector(const Vector3& dir, const Vector3& up)
    {
        return Matrix4();
    }
    Matrix4 matrixFromAxisAngle(const Vector3& axis, float angle)
    {
        return Matrix4();
    }
    void interpolateMatrix(Matrix4& start, Matrix4& end, float blend, Matrix4& io)
    {
        
    }
    
    float getUFromPoint(const Vector3& point, const Matrix4& A, const Matrix4& B)
    {
        return 0.0f;
    }
    
    void rotateVector(Vector3& v, const Quaternion& q)
    {
        float l = v.length();
        
        v.normalize();
        
        Quaternion vecQuat, conjQuat, resQuat;
        vecQuat.x = v.x;
        vecQuat.y = v.y;
        vecQuat.z = v.z;
        vecQuat.w = 0.0;
        
        conjQuat = q.conjugate();
        resQuat.multiply(vecQuat,conjQuat);
        resQuat.multiply(q,resQuat);
        
        v.x = resQuat.x;
        v.y = resQuat.y;
        v.z = resQuat.z;
        
        v.normalize();
        
        v.x *= l;
        v.y *= l;
        v.z *= l;
    };
    
    Vector3 rotateVector(Vector3& v, Quaternion& q)
    {
        float l = v.length();
        
        Vector3 o = v.normalize();
        
        Quaternion vecQuat, conjQuat, resQuat;
        vecQuat.x = o.x;
        vecQuat.y = o.y;
        vecQuat.z = o.z;
        vecQuat.w = 0.0;
        
        conjQuat = q.conjugate();
        resQuat.multiply(vecQuat,conjQuat);
        resQuat.multiply(q,resQuat);
        
        o.x = resQuat.x;
        o.y = resQuat.y;
        o.z = resQuat.z;
        o.normalize();
        o *= l;
        return o;
    };
    
    Quaternion lookAt(Vector3& dir, Vector3& up,bool transpose=false)
    {
        Matrix3 m;
        m.setFromTwoVectors(dir, up);
        return m.getQuaternion(transpose);
    }
    
    inline Vector3 euclideanProjection(const Vector4& homogenous)
    {
        float inv = (homogenous[3] != 0.0f) ? 1.0f/homogenous[3] : 1.0f;
        return Vector3(inv * homogenous[0], inv * homogenous[1], inv * homogenous[2]);
    }
}
