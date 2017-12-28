//-------------------------------------------------------
// POINT IMPLEMENTATION
//-------------------------------------------------------
#include "point.h"

namespace BOB{
Point::Point(){};

Point::Point(float x, float y, float z)
{
    position.x = x;
    position.y = y;
    position.z = z;
    
    origin.x = x;
    origin.y = y;
    origin.z = z;
}

Point::~Point()
{
    closests.clear();
}

    
}//end namespace BOB
