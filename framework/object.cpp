//-------------------------------------------------------
// OBJECT IMPLEMENTATION
//-------------------------------------------------------
#include "object.h"
namespace BOB{
    Object::Object()
    {
        parent=0;
    };
    
    Object::Object(Object* _parent)
    {
        parent=_parent;
    };

    Object::~Object()
    {
        
    }
    
    // Local Xform
    void Object::setLocalScale(const Vector3& s)
    {
        localXfo.setScale(s);
    }
    
    void Object::setLocalPosition(const Vector3& p)
    {
        localXfo.setPosition(p);
    }
    
    void Object::setLocalRotation(const Quaternion& q)
    {
        localXfo.setRotation(q);
    }
    
    void Object::setLocalMatrix(const Matrix4& m)
    {
        localXfo.setMatrix(m);
    }
    
    // Global Xform
    void Object::setGlobalScale(const Vector3& s)
    {
        globalXfo.setScale(s);
    }
    
    void Object::setGlobalPosition(const Vector3& p)
    {
        globalXfo.setPosition(p);
    }
    
    void Object::setGlobalRotation(const Quaternion& q)
    {
        globalXfo.setRotation(q);
    }
    
    void Object::setGlobalMatrix(const Matrix4& m)
    {
        globalXfo.setMatrix(m);
    }
    
    // Update
    void Object::updateTransform()
    {
        if(parent)
        {
            localXfo.setMatrixFromSRT();
            globalXfo.setMatrix(localXfo.update(parent->globalXfo));
        }
        else
        {
            globalXfo.setMatrix(localXfo.getMatrix());
        }
    }
}
