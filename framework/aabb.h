//--------------------------------------------------------
// Axis Aligned Bounding Box Declaration
//--------------------------------------------------------
#ifndef _AABB_H_
#define _AABB_H_

#include <float.h>
#include "vector3.h"
#include "matrix4.h"
#include "point.h"

namespace BOB{
class AABB{
	public:
		void compute(const float* positions, int num_points);
		void compute(const float* positions, const int* vertices, int num_points);
    	void compute(std::vector<Point>& vertices);
    void compute(const std::vector<Point>& vertices, const std::vector<unsigned>& indices);
		bool intersect(const AABB& other);
		bool isInside(const Vector3& pos);
    	bool isInside(const Point& pnt);
		void transform(const Matrix4& M);
        Vector3& min(){return _min;};
		Vector3& max(){return _max;};
	private:
		Vector3 _min;
		Vector3 _max;
};
}//end namespace BOB
#endif /* _AABB_H_ */
