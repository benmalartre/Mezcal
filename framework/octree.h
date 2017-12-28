//======================================================
// OCTREE DECLARATION
//======================================================
#ifndef _OCTREE_H_
#define _OCTREE_H_

#include "vector3.h"
#include "triangle.h"
#include "mesh.h"
#include <float.h>

#ifdef USE_GLDRAWER
	#include "shape.h"
#endif

namespace BOB{
class Octree
{
public:
	Octree() : _depth(0), _min(-1, -1, -1), _max(1, 1, 1), _isLeaf(true)
	{ for (int i=0; i<8; i++) _child[i] = NULL; };
    
	Octree(const Vector3& minp, const Vector3& maxp, int depth = 0) :
	_depth(depth), _min(minp), _max(maxp), _isLeaf(true)
	{ for (int i=0; i<8; i++) _child[i] = NULL; };
    
    ~Octree();

	// depth in octree
	int getDepth() const { return _depth; };

	// bounding box
	const Vector3& getMin() const { return _min; };
	const Vector3& getMax() const { return _max; };

	// center
	Vector3 getCenter(){return (Vector3((_min+_max)*0.5f));};
	Vector3 getHalfSize(){return (Vector3((_max-_min)*0.5f));};

    // squared value
    inline float squared(float v){return v * v;};
    
	// distance
    inline float getDistance1D(float p, float lower, float upper)
    {
        if(p < lower)return lower - p;
        if(p > upper)return p - upper;
        return fmin(p - lower, upper - p);
    };
	float getDistance(const Vector3& point);
    
    // intersect sphere
    bool intersectSphere(const Vector3& center, const float radius);

	// leaf
	bool isLeaf() const { return _isLeaf; };

	// get cell
	Octree* getCell(int x) {return _child[x];}

	// triangles size
	int getSize() const { return _triangles.size(); };
	std::vector<Triangle*>& getTriangles() { return _triangles; };

	// insert a triangle
	void insert(Triangle *t) { _triangles.push_back(t); };

	// split into 8
	void split(const float* positions);

	void setMin(Vector3 min){_min=min;};
	void setMax(Vector3 max){_max=max;};
    
    // get bounding box
    void getBoundingBox(const float* positions, const int* vertices, int num_vertices);
    
    // get furthest corner
    void getFurthestCorner(const Vector3& point, Vector3& corner);

	// build the tree
    void buildTree(std::vector<Triangle>& triangles, const float* positions);
	void clearTree();
/*
#ifdef USE_GLDRAWER
    void draw(GLDrawer::Compound*);
#endif
*/
private:
	// depth in octree
	int _depth;

	// bounding box
	Vector3 _min, _max;

	// leaf ?
	bool _isLeaf;

	// children
	Octree* _child[8];

	// triangles
	std::vector<Triangle*> _triangles;

	// positions
	// const float* _positions;

	// static members
	static const int MAX_TRIANGLES_NUMBER;
/*
#ifdef USE_GLDRAWER
    float _r,_g,_b;
#endif
 */
};
}//end namespace BOB
#endif /* _OCTREE_H_ */
