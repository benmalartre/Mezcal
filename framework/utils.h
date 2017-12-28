//--------------------------------------------------------
// UTILS
//--------------------------------------------------------
#ifndef _UTILS_H_
#define _UTILS_H_

#include "vector3.h"
#include "quaternion.h"
#include "matrix3.h"
#include "matrix4.h"
#include "xform.h"

namespace BOB{

template<typename T>
T LINEAR_INTERPOLATE(T y1,T y2,T mu)
{
    return y1*(1-mu)+y2*mu;
}
    
template<typename T>
T CUBIC_INTERPOLATE(T y0,T y1,T y2,T y3,T mu)
{
    T a0,a1,a2,a3,mu2;
    mu2 = (mu*mu);
    a0 = y3 - y2 - y0 + y1;
    a1 = y0 - y1 - a0;
    a2 = y2 - y0;
    a3 = y1;
    return a0*mu*mu2+a1*mu2+a2*mu+a3;
}

template<typename T>
T HERMITE_INTERPOLATE(T y0,T y1,T y2,T y3,T mu,T tension,T bias)
{
    T m0,m1,mu2,mu3;
    T a0,a1,a2,a3;
    mu2 = mu * mu;
    mu3 = mu2 * mu;
    m0  = (y1-y0)*(1+bias)*(1-tension)/2;
    m0 += (y2-y1)*(1-bias)*(1-tension)/2;
    m1  = (y2-y1)*(1+bias)*(1-tension)/2;
    m1 += (y3-y2)*(1-bias)*(1-tension)/2;
    a0  = 2*mu3 - 3*mu2 + 1;
    a1  = mu3 - 2*mu2 + mu;
    a2  = mu3 -   mu2;
    a3  = -2*mu3 + 3*mu2;
    return a0*y1+a1*m0+a2*m1+a3*y2;
}

    Vector3 linearInterpolate(const Vector3& A, const Vector3& B, float blend);
    Vector3 cubicInterpolate(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& D, float blend);
    Vector3 hermiteInterpolate(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& D, float blend, float tension, float bias);
    float quaternionDot(Quaternion& q1, Quaternion& q2);
	Quaternion quaternionSlerp(Quaternion& a, Quaternion& b, float t);
	Quaternion quaternionSlerp2(Quaternion qA, Quaternion qB, double blend);
	Quaternion eulerToQuaternion(double x, double y, double z);
	Quaternion rotationBetweenVectors(const Vector3& start, const Vector3& dest);
    Quaternion quaternionLookAt(const Vector3& dir, const Vector3& up);
    Quaternion quaternionFromAxisAngle(const Vector3& axis, float angle);

	Matrix4 matrixFromTwoVector(const Vector3& dir, const Vector3& up);
	Matrix4 matrixFromAxisAngle(const Vector3& axis, float angle);
	void interpolateMatrix(const Matrix4& start, const Matrix4& end, float blend, Matrix4& io);

    void rotateVector(Vector3& v, const Quaternion& q);
    Vector3 rotateVector(Vector3& v, Quaternion& q);

    // Execute a system command
    //int executeCommand(const char* cmd);
}
#endif /* _COMMON_H_ */
