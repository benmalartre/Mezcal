//-------------------------------------------------------
// VERTEX IMPLEMENTATION
//-------------------------------------------------------
#include "vertex.h"

namespace BOB{
    /*
void Vertex::projectOnPlane(Vector3& point, Vector3& orig, Vector3& normal, Vector3& projected)
{
	Vector3 delta = point-orig;
	float distance = delta.x*normal.x + delta.y*normal.y + delta.z*normal.z;
	projected = point - (normal*distance);
}

void Vertex::project(Vector3& point, Vector3& projected)
{
	Vector3 delta = point-hitPoint;
	float distance = delta.x*hitNormal.x + delta.y*hitNormal.y + delta.z*hitNormal.z;
	projected = point - (hitNormal*distance);
}
*/
Vertex::Vertex(){
    index=0;
};

Vertex::Vertex(const int ID){
	// Set Index
    index = ID;
    
};

Vertex::~Vertex(){
	neighbors.clear();
}

void Vertex::transform(const Matrix4& M){
	position *= M;
	normal *= M;
	normal.normalize();
}

}// end namespace BOB
