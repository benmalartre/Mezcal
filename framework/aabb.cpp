//-------------------------------------------------------
// AABB IMPLEMENTATION
//-------------------------------------------------------
#include "aabb.h"

namespace BOB{
//-------------------------------------------------------
// Compute AABB from Mesh
//-------------------------------------------------------
void AABB::compute(const float* positions, int num_points)
{
	_min.x = FLT_MAX;
	_min.y = FLT_MAX;
	_min.z = FLT_MAX;
	_max.x = -FLT_MAX;
	_max.y = -FLT_MAX;
	_max.z = -FLT_MAX;

	for(int i=0;i<num_points;i++)
	{
		if(positions[i*3]<_min.x)_min.x = positions[i*3];
		if(positions[i*3+1]<_min.y)_min.y = positions[i*3+1];
		if(positions[i*3+2]<_min.z)_min.z = positions[i*3+2];

		if(positions[i*3]>_max.x)_max.x = positions[i*3];
		if(positions[i*3+1]>_max.y)_max.y = positions[i*3+1];
		if(positions[i*3+2]>_max.z)_max.z = positions[i*3+2];
	}
}

//-------------------------------------------------------
// Compute AABB from Point Position and Vertex Indices Array
//-------------------------------------------------------
void AABB::compute( const float* positions, const int* vertices, int num_points)
{
	_min.x = FLT_MAX;
	_min.y = FLT_MAX;
	_min.z = FLT_MAX;
	_max.x = -FLT_MAX;
	_max.y = -FLT_MAX;
	_max.z = -FLT_MAX;

	int j=0;
	for(int i=0;i<num_points;i++)
	{
		j = vertices[i];
		if(positions[j*3]<_min.x)_min.x = positions[j*3];
		if(positions[j*3+1]<_min.y)_min.y = positions[j*3+1];
		if(positions[j*3+2]<_min.z)_min.z = positions[j*3+2];

		if(positions[i*3]>_max.x)_max.x = positions[i*3];
		if(positions[i*3+1]>_max.y)_max.y = positions[i*3+1];
		if(positions[i*3+2]>_max.z)_max.z = positions[i*3+2];
	}
}


//-------------------------------------------------------
// Compute AABB from Point Array
//-------------------------------------------------------
void AABB::compute(std::vector<Point>& points)
{
    _min.x = FLT_MAX;
    _min.y = FLT_MAX;
    _min.z = FLT_MAX;
    _max.x = -FLT_MAX;
    _max.y = -FLT_MAX;
    _max.z = -FLT_MAX;
    
    Vector3 pos;
    Vector3 norm;
    std::vector<Point>::iterator it = points.begin();
    for(;it<points.end();it++)
    {
        pos = (Vector3)(*it).position;
        norm = (*it).normal;
        
        if(pos.x<_min.x)_min.x = pos.x;
        if(pos.y<_min.y)_min.y = pos.y;
        if(pos.z<_min.z)_min.z = pos.z;
        
        if(pos.x>_max.x)_max.x = pos.x;
        if(pos.y>_max.y)_max.y = pos.y;
        if(pos.z>_max.z)_max.z = pos.z;
    }
}

//-------------------------------------------------------
// Compute AABB from Vertex Array and Vertex Indices Array
//-------------------------------------------------------
void AABB::compute(const std::vector<Point>& points, const std::vector<unsigned>& indices)
{
    _min.x = FLT_MAX;
    _min.y = FLT_MAX;
    _min.z = FLT_MAX;
    _max.x = -FLT_MAX;
    _max.y = -FLT_MAX;
    _max.z = -FLT_MAX;
    
    int j=0;
    Vector3 pos;
    Vector3 norm;
    for(unsigned i=0;i<points.size();i++)
    {
        j = indices[i];

        pos = (Vector3)points[j].position;
        norm = points[j].normal;
        
        if(pos.x<_min.x)_min.x = pos.x;
        if(pos.y<_min.y)_min.y = pos.y;
        if(pos.z<_min.z)_min.z = pos.z;
        
        if(pos.x>_max.x)_max.x = pos.x;
        if(pos.y>_max.y)_max.y = pos.y;
        if(pos.z>_max.z)_max.z = pos.z;
    }
}

//-------------------------------------------------------
// Vector3 Is Inside
//-------------------------------------------------------
bool AABB::isInside(const Vector3& pos)
{
	bool inside = pos.x >= _min.x &&
				pos.x <= _max.x &&
				pos.y >= _min.y &&
				pos.y <= _max.y &&
				pos.z >= _min.z &&
				pos.z <= _max.z;
	return inside;
}
    
//-------------------------------------------------------
// Point Is Inside
//-------------------------------------------------------
bool AABB::isInside(const Point& pnt)
{
    bool inside = (pnt.position.x >= _min.x &&
    pnt.position.x <= _max.x &&
    pnt.position.y >= _min.y &&
    pnt.position.y <= _max.y &&
    pnt.position.z >= _min.z &&
    pnt.position.z <= _max.z);
    return inside;
}

//-------------------------------------------------------
// AABB Intersect Other
//-------------------------------------------------------
bool AABB::intersect(const AABB& other)
{
	bool intersect = _min.x < other._max.x &&
					_max.x > other._min.x &&
					_min.y < other._max.y &&
					_max.y > other._min.y &&
					_min.z < other._max.z &&
					_max.z > other._min.z;
	return intersect;
}

//-------------------------------------------------------
// AABB Transform
// Only Transform the 8 corners of the Bounding Box
// Should be faster but not really accurate
//-------------------------------------------------------
void AABB::transform(const Matrix4& M)
{

    float P[6] = {
    			(float)_min.x,
    			(float)_min.y,
				(float)_min.z,
				(float)_max.x,
				(float)_max.y,
				(float)_max.z
    };

    int permutation[24] = {0,1,2,3,1,2,0,1,5,3,1,5,
    						0,4,2,3,4,2,0,4,5,3,4,5};
    
    _min.x = FLT_MAX;
    _min.y = FLT_MAX;
    _min.z = FLT_MAX;
    _max.x = -FLT_MAX;
    _max.y = -FLT_MAX;
    _max.z = -FLT_MAX;


    for(int i=0;i<8;i++)
    {
        Vector3 T(P[permutation[i*3]],P[permutation[i*3+1]],P[permutation[i*3+2]]);
        T*=M;
        if(T.x<_min.x)_min.x = T.x;
        if(T.y<_min.y)_min.y = T.y;
        if(T.z<_min.z)_min.z = T.z;
        
        if(T.x>_max.x)_max.x = T.x;
        if(T.y>_max.y)_max.y = T.y;
        if(T.z>_max.z)_max.z = T.z;
    }
}

}//end namespace BOB

