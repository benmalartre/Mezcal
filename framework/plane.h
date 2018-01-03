
#ifndef _PLANE_H
#define _PLANE_H

#include "vector3.h"

namespace BOB{
class Range3;
class Matrix4;

class Plane
{
  public:

    /// The default constructor leaves the plane parameters undefined.
    Plane() {
    }

    /// This constructor sets this to the plane perpendicular to \p normal and
    /// at \p distance units from the origin. The passed-in normal is
    /// normalized to unit length first.
    Plane(const Vector3 &normal, float distanceToOrigin) {
        set(normal, distanceToOrigin);
    }

    /// This constructor sets this to the plane perpendicular to \p normal and
    /// that passes through \p point. The passed-in normal is normalized to
    /// unit length first.
    Plane(const Vector3 &normal, const Vector3 &point) {
        set(normal, point);
    }

    /// This constructor sets this to the plane that contains the three given
    /// points. The normal is constructed from the cross product of (\p p1 -
    /// \p p0) (\p p2 - \p p0). Results are undefined if the points are
    /// collinear.
    Plane(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2) {
        set(p0, p1, p2);
    }

    /// Sets this to the plane perpendicular to \p normal and at \p distance
    /// units from the origin. The passed-in normal is normalized to unit
    /// length first.
    void set(const Vector3 &normal, float distanceToOrigin) {
        _normal = normal.normalize();
        _distance = distanceToOrigin;
    }

    /// This constructor sets this to the plane perpendicular to \p normal and
    /// that passes through \p point. The passed-in normal is normalized to
    /// unit length first.
    void set(const Vector3 &normal, const Vector3 &point);

    /// This constructor sets this to the plane that contains the three given
    /// points. The normal is constructed from the cross product of (\p p1 -
    /// \p p0) (\p p2 - \p p0). Results are undefined if the points are
    /// collinear.
    void set(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2);

    /// Returns the unit-length normal vector of the plane.
    const Vector3 &   getNormal() const {
        return _normal;
    }

    /// Returns the distance of the plane from the origin.
    float getDistanceFromOrigin() const {
        return _distance;
    }

    /// Component-wise equality test. The normals and distances must match
    /// exactly for planes to be considered equal.
    bool		operator ==(const Plane &p) const {
	return (_normal   == p._normal &&
		_distance == p._distance);
    }

    /// Component-wise inequality test. The normals and distances must match
    /// exactly for planes to be considered equal.
    bool		operator !=(const Plane &p) const {
	return ! (*this == p);
    }

    /// Returns the distance of point \p from the plane. This distance will be
    /// positive if the point is on the side of the plane containing the
    /// normal.
    float getDistance(const Vector3 &p) const {
        return p * _normal - _distance;
    }

    /// Return the projection of \p p onto the plane.
    Vector3 project(const Vector3& p) const {
        return p - _normal * getDistance(p);
    }

    /// Transforms the plane by the given matrix.
    Plane &  transform(const Matrix4 &matrix);

    /// Flip the plane normal (if necessary) so that \p p is in the positive
    /// halfspace.
    void    reorient(const Vector3& p) {
        if (getDistance(p) < 0.0f) {
            _normal = -_normal;
            _distance = -_distance;
        }
    }

    /// Returns \c true if the given aligned bounding box is at least
    /// partially on the positive side (the one the normal points into) of the
    /// plane.
    bool intersectsPositiveHalfSpace(const Range3 &box) const;

    /// Returns true if the given point is on the plane or within its positive
    /// half space.
    bool intersectsPositiveHalfSpace(const Vector3 &pt) const {
        return getDistance(pt) >= 0.0f;
    }

  private:
    /// The normal to the plane. Points in direction of half-space.
    Vector3             _normal;

    /// Distance from the plane to the origin.
    float              _distance;
};

} // end namespace BOB
#endif // _PLANE_H
