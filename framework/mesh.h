//------------------------------------------------
// MESH DECLARATIONS
//------------------------------------------------

#ifndef _MESH_H_
#define _MESH_H_

#include <vector>
#include "triangle.h"
#include "vector3.h"
#include "matrix4.h"

namespace BOB{
class Mesh{
public:
    Mesh(){};
    Mesh(unsigned numVertices, const float* positions, unsigned num_desc, int* desc);
    ~Mesh();
    
    void set(unsigned num_points, const float * positions, const float* normals, Matrix4& M);
    void set(unsigned num_points, const float* positions);
    
	unsigned nbVertices;
	unsigned nbFaces;
	unsigned nbTriangles;
    unsigned nbSamples;
    
    Vector3 _min;
    Vector3 _max;

    std::vector<Vector3> normals;
    std::vector<Vector3> positions;
    std::vector<int	> description;
    std::vector<unsigned> triangles;
    Matrix4 matrix;
    Matrix4 invMatrix;
    
    void getNumFaces();
    void getTriangles();
    void getBoundingBox(bool worldSpace=true);
    
};
}//end namespace BOB
#endif
