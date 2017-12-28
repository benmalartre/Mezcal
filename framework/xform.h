//--------------------------------------------------------
// XFORM DECLARATION
//--------------------------------------------------------
#ifndef _XFORM_H_
#define _XFORM_H_

#include <float.h>
#include "vector3.h"
#include "quaternion.h"
#include "matrix4.h"

namespace BOB{
class XForm{
public:
    void setSRTFromMatrix();
    void setMatrixFromSRT();
    void setScale(const Vector3& s);
    void setPosition(const Vector3& p);
    void setRotation(const Quaternion& q);
    void setRotationFromEuler(float x, float y, float z);
    void setMatrix(const Matrix4& matrix);
    Matrix4& getMatrix(){return m;};
    Matrix4 update(const XForm& parentXfo);
private:
    Vector3 pos;
    Quaternion rot;
    Vector3 scl;
    Matrix4 m;
    bool srtDirty;
    bool matrixDirty;
};
}//end namespace BOB
#endif /* _XFORM_H_ */
