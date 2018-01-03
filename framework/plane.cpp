//
// Copyright 2016 Pixar
//
// Licensed under the Apache License, Version 2.0 (the "Apache License")
// with the following modification; you may not use this file except in
// compliance with the Apache License and the following modification to it:
// Section 6. Trademarks. is deleted and replaced with:
//
// 6. Trademarks. This License does not grant permission to use the trade
//    names, trademarks, service marks, or product names of the Licensor
//    and its affiliates, except as required to comply with Section 4(c) of
//    the License and to reproduce the content of the NOTICE file.
//
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the Apache License with the above modification is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the Apache License for the specific
// language governing permissions and limitations under the Apache License.
//

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
    Vector3 pointOnPlane = _distance * _normal;

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
    pointOnPlane = pointOnPlane.tranform(matrix);

    // The new distance is the projected distance of the vector to the
    // transformed point onto the (unit) transformed normal. This is
    // just a dot product.
    _distance = pointOnPlane.dot(_normal);

    return *this;
}

bool Plane::intersectsPositiveHalfSpace(const Range3 &box) const
{
    if (box.IsEmpty())
	return false;
    
    // Test each vertex of the box against the positive half
    // space. Since the box is aligned with the coordinate axes, we
    // can test for a quick accept/reject at each stage.

// This macro tests one corner using the given inequality operators.
#define _CORNER_TEST(X, Y, Z, XOP, YOP, ZOP)                               \
    if (X + Y + Z >= _distance)                                               \
        return true;                                                          \
    else if (_normal[0] XOP 0.0 && _normal[1] YOP 0.0 && _normal[2] ZOP 0.0)  \
        return false

    // The sum of these values is GfDot(box.GetMin(), _normal)
    float xmin = _normal[0] * box.GetMin()[0];
    float ymin = _normal[1] * box.GetMin()[1];
    float zmin = _normal[2] * box.GetMin()[2];

    // We can do the all-min corner test right now.
    _CORNER_TEST(xmin, ymin, zmin, <=, <=, <=);

    // The sum of these values is GfDot(box.GetMax(), _normal)
    float xmax = _normal[0] * box.GetMax()[0];
    float ymax = _normal[1] * box.GetMax()[1];
    float zmax = _normal[2] * box.GetMax()[2];

    // Do the other 7 corner tests.
    _CORNER_TEST(xmax, ymax, zmax, >=, >=, >=);
    _CORNER_TEST(xmin, ymin, zmax, <=, <=, >=);
    _CORNER_TEST(xmin, ymax, zmin, <=, >=, <=);
    _CORNER_TEST(xmin, ymax, zmax, <=, >=, >=);
    _CORNER_TEST(xmax, ymin, zmin, >=, <=, <=);
    _CORNER_TEST(xmax, ymin, zmax, >=, <=, >=);
    _CORNER_TEST(xmax, ymax, zmin, >=, >=, <=);

    return false;

#undef _GF_CORNER_TEST
}

} // end namespace BOB
