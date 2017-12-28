//--------------------------------------------------------
// Object
//--------------------------------------------------------
#ifndef _OBJECT_H_
#define _OBJECT_H_

#include "matrix4.h"
#include "vector3.h"
#include "quaternion.h"
#include "xform.h"

namespace BOB{
class Object{
public:
    Object();
    Object(Object* parent);
    ~Object();
    
    void setLocalPosition(const Vector3& p);
    void setLocalRotation(const Quaternion& q);
    void setLocalRotation(float x, float y, float z);
    void setLocalScale(const Vector3& s);
    void setLocalMatrix(const Matrix4& m);
    
    void setGlobalPosition(const Vector3& p);
    void setGlobalRotation(const Quaternion& q);
    void setGlobalRotation(float x, float y, float z);
    void setGlobalScale(const Vector3& s);
    void setGlobalMatrix(const Matrix4& m);
    
    void updateTransform();
    
private:
    Object* parent;
    XForm localXfo;
    XForm globalXfo;
};

}// end namespace BOB
#endif /* _OBJECT_H_ */
