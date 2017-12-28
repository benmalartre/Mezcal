//--------------------------------------------------------
// ATTRIBUTE
//--------------------------------------------------------
#ifndef _ATTRIBUTE_H_
#define _ATTRIBUTE_H_

#include "common.h"

namespace BOB{

    enum ATTR_DATA_TYPE{
        ATTR_TYPE_UNDEFINED 	= 0x00,
        ATTR_TYPE_NEW			= 0x01,
        ATTR_TYPE_BOOLEAN		= 0x02,
        ATTR_TYPE_LONG 			= 0x03,
        ATTR_TYPE_INTEGER		= 0x04,
        ATTR_TYPE_FLOAT			= 0x05,
        ATTR_TYPE_VECTOR2		= 0x06,
        ATTR_TYPE_VECTOR3		= 0x07,
        ATTR_TYPE_VECTOR4 		= 0x08,
        ATTR_TYPE_COLOR 		= 0x09,
        ATTR_TYPE_ROTATION		= 0x10,
        ATTR_TYPE_QUATERNION	= 0x11,
        ATTR_TYPE_MATRIX3  		= 0x12,
        ATTR_TYPE_MATRIX4		= 0x13,
        ATTR_TYPE_STRING		= 0x14
    };
    
    enum ATTR_DATA_STRUCT{
		ATTR_STRUCT_SINGLE,
		ATTR_STRUCT_ARRAY,
		ATTR_STRUCT_ANY
    };
    
    enum ATTR_DATA_CTXT{
		ATTR_CTXT_SINGLETON,
		ATTR_CTXT_COMPONENT0D,
		ATTR_CTXT_COMPONENT1D,
		ATTR_CTXT_COMPONENT2D,
		ATTR_CTXT_COMPONENT0D2D,
		ATTR_CTXT_GENERATOR,
		ATTR_CTXT_ANY
    };


class Attribute
{
public:
    Attribute();
    ~Attribute();
    
private:
    std::string name;
    ATTR_DATA_TYPE T;
    ATTR_DATA_STRUCT S;
    ATTR_DATA_CTXT C;
    
    void *data;
    bool constant;
    bool readonly;
    bool dirty;
    bool writable;
};

}// end namespace BOB
#endif /* _CAMERA_H_ */
