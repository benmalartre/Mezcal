//--------------------------------------------------
// INTERSECTOR DECLARATION
//--------------------------------------------------

#ifndef _INTERSECTOR_H_
#define _INTERSECTOR_H_

#include <vector>
#include <set>
#include <float.h>
#include "triangle.h"
#include "mesh.h"
#include "octree.h"
#include "vector3.h"

#ifdef USE_GLDRAWER
#include "shape.h"
#endif

namespace BOB{
class ClosestPoint;

class Intersector{
private:
    std::vector<Triangle> triangles;
    const float* positions;
    void barycentric(Vector3& p, Vector3& a, Vector3& b, Vector3& c, float &u, float &v, float &w);
    void getClosestCell(const Vector3& point, Octree*& closestCell);
    void recurseGetClosestCell(const Vector3& point, Octree* cell, float& closestDistance, Octree*& closestCell);
    void getNearbyCells(const Vector3& point, Octree* cell, std::vector<Octree*>& cells);
    void recurseGetNearbyCells(Octree* cell, const Vector3& center, const float radius, std::vector<Octree*>& cells);
    Octree octree;
public:
    Intersector(){};
    ~Intersector(){};
    
    std::vector<Triangle>& getTriangles(){return triangles;};
    /*
    MStatus create(MFnMesh& mesh);
    MStatus create(MFnMesh& mesh, MIntArray& indices);
	*/
    bool create(const float* positions, unsigned num_points, const unsigned* indices, unsigned num_tris);
    bool create(const float* positions, unsigned num_points, const unsigned* indices, const unsigned* map_indices, unsigned num_tris);
    void getClosestPoint(const Vector3& point, ClosestPoint& closest);
    void getClosestPointBruteForce(const Vector3& point, ClosestPoint& closest);
#ifdef USE_GLDRAWER
    void draw(GLDrawer::Compound*);
#endif
};

class ClosestPoint{
private:
    friend class Intersector;
    float U;
    float V;
    float W;
    int triangleID;
    int triangleMapID;
    Vector3 position;
    Vector3 normal;
public:
    int triangleIndex() {return triangleID;};
    int triangleMapIndex() {return triangleMapID;};
    Vector3& getPosition() {return position;};
    Vector3& getNormal() {return normal;};
    void getBarycentricCoordinates(float* uvw){uvw[0]=U; uvw[1]=V; uvw[2]=W;};
};
}//end namespace BOB
#endif /* _INTERSECTOR_H_ */
