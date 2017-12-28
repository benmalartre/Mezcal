//--------------------------------------------------------
// POINT
//--------------------------------------------------------
#ifndef _POINT_H_
#define _POINT_H_

#include <float.h>
#include <vector>
//#include <map>
#include "vector3.h"
#include "common.h"
//#include "attribute.h"

namespace BOB{
class Point{
public:
    Point();
    Point(float x, float y, float z);
    ~Point();
    
    unsigned getID(){return ID;};
    //void addAttribute(std::string name, ATTR_DATA_CTXT ctxt, ATTR_DATA_TYPE type, ATTR_DATA_STRUCT struc);
    
//private:
    unsigned ID;
    std::vector<Point*> closests;
    float fixed;
    Vector3 position;
    Vector3 normal;
    Vector3 tangent;
    Vector3 origin;
    Vector3 velocity;
    Vector3 up;
    char border;
    float mass;
    
    //std::map<std::string, Attribute> attributes;
    
};
}// end namespace BOB
#endif /* _POINT_H_ */
