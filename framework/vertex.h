//------------------------------------------------
// VERTEX DECLARATIONS
//------------------------------------------------

#ifndef VERTEX_H_
#define VERTEX_H_
#include <vector>
#include <float.h>
#include "vector3.h"
#include "matrix4.h"

namespace BOB{
class Vertex{
	public:
		Vertex();
		Vertex(const int ID);
		~Vertex();

		void transform(const Matrix4& M);

    	unsigned index;
    	Vector3 position;
    	Vector3 normal;
    	Vector3 tangent;
		std::vector<Vertex*> neighbors;
        std::vector<Vertex*> adjacents;
		std::vector<float> distances;
		std::vector<float> weights;
};
} // end namespace BOB
#endif /* VERTEX_H_ */
