#include "plane.h"
#include "matrix4.h"
#include "range3.h"


namespace  BOB {
void Plane::set(const Vector3 &normal, const Vector3 &point)
{
    _normal = normal.normalize();
    _distance = _normal.dot(point);
}

void Plane::set(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2)
{
    Vector3 p0p1 = p1 - p0;
    Vector3 p0p2 = p2 - p0;
    _normal = p0p1.cross(p0p2);
    _normal.normalizeInPlace();
    _distance = _normal.dot(p0);
}

Plane & Plane::transform(const Matrix4 &matrix)
{
    // Compute the point on the plane along the normal from the origin.
    Vector3 pointOnPlane = _normal * _normal;

    // Transform the plane normal by the adjoint of the matrix to get
    // the new normal.  The adjoint (inverse transpose) is used to
    // multiply normals so they are not scaled incorrectly.
    Matrix4 adjoint = matrix.inverse();
    adjoint.transposeInPlace();
    //_normal = adjoint.TransformDir(_normal).GetNormalized();
    _normal = _normal.transformDir(adjoint);
    _normal.normalizeInPlace();

    // Transform the point on the plane by the matrix.
    //pointOnPlane = matrix.Transform(pointOnPlane);
    pointOnPlane = pointOnPlane.transform(matrix);

    // The new distance is the projected distance of the vector to the
    // transformed point onto the (unit) transformed normal. This is
    // just a dot product.
    _distance = pointOnPlane.dot(_normal);

    return *this;
}

bool Plane::intersectsPositiveHalfSpace(const Range3 &box) const
{
    if (box.isEmpty())
	return false;
    
    // Test each vertex of the box against the positive half
    // space. Since the box is aligned with the coordinate axes, we
    // can test for a quick accept/reject at each stage.

// This macro tests one corner using the given inequality operators.
#define CORNER_TEST(X, Y, Z, XOP, YOP, ZOP)                               \
    if (X + Y + Z >= _distance)                                               \
        return true;                                                          \
    else if (_normal[0] XOP 0.0 && _normal[1] YOP 0.0 && _normal[2] ZOP 0.0)  \
        return false

    // The sum of these values is GfDot(box.GetMin(), _normal)
    float xmin = _normal[0] * box.getMin()[0];
    float ymin = _normal[1] * box.getMin()[1];
    float zmin = _normal[2] * box.getMin()[2];

    // We can do the all-min corner test right now.
    CORNER_TEST(xmin, ymin, zmin, <=, <=, <=);

    // The sum of these values is GfDot(box.GetMax(), _normal)
    float xmax = _normal[0] * box.getMax()[0];
    float ymax = _normal[1] * box.getMax()[1];
    float zmax = _normal[2] * box.getMax()[2];

    // Do the other 7 corner tests.
    CORNER_TEST(xmax, ymax, zmax, >=, >=, >=);
    CORNER_TEST(xmin, ymin, zmax, <=, <=, >=);
    CORNER_TEST(xmin, ymax, zmin, <=, >=, <=);
    CORNER_TEST(xmin, ymax, zmax, <=, >=, >=);
    CORNER_TEST(xmax, ymin, zmin, >=, <=, <=);
    CORNER_TEST(xmax, ymin, zmax, >=, <=, >=);
    CORNER_TEST(xmax, ymax, zmin, >=, >=, <=);

    return false;

#undef CORNER_TEST
}

} // end namespace BOB
