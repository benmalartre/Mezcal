#ifndef _RANGE3_H
#define _RANGE3_H

#include "vector3.h"
#include <cfloat>
#include <cstddef>
#include <iosfwd>

namespace BOB {
class Range3
{
public:

    static const size_t dimension = 3;
    /// Sets the range to an empty interval
    // TODO check whether this can be deprecated.
    void inline setEmpty() {
        _min[0] = _min[1] = _min[2] =  FLT_MAX;
        _max[0] = _max[1] = _max[2] = -FLT_MAX;
    }

    /// The default constructor creates an empty range.
    Range3() {
        setEmpty();
    }

    /// This constructor initializes the minimum and maximum points.
    Range3(const Vector3 &min, const Vector3 &max)
        : _min(min), _max(max)
    {
    }

    /// Returns the minimum value of the range.
    const Vector3 &getMin() const { return _min; }

    /// Returns the maximum value of the range.
    const Vector3 &getMax() const { return _max; }

    /// Returns the size of the range.
    Vector3 getSize() const { return _max - _min; }

    /// Returns the midpoint of the range, that is, 0.5*(min+max).
    /// Note: this returns zero in the case of default-constructed ranges,
    /// or ranges set via SetEmpty().
    Vector3 getMidpoint() const {
        return Vector3(0.5f,0.5f,0.5f) * _min
               + Vector3(0.5f,0.5f,0.5f) * _max;
    }

    /// Sets the minimum value of the range.
    void setMin(const Vector3 &min) { _min = min; }

    /// Sets the maximum value of the range.
    void setMax(const Vector3 &max) { _max = max; }

    /// Returns whether the range is empty (max < min).
    bool isEmpty() const {
        return _min[0] > _max[0] || _min[1] > _max[1] || _min[2] > _max[2];
    }

    /// Modifies the range if necessary to surround the given value.
    /// \deprecated Use UnionWith() instead.
    void extendBy(const Vector3 &point) { unionWith(point); }

    /// Modifies the range if necessary to surround the given range.
    /// \deprecated Use UnionWith() instead.
    void extendBy(const Range3 &range) { unionWith(range); }

    /// Returns true if the \p point is located inside the range. As with all
    /// operations of this type, the range is assumed to include its extrema.
    bool contains(const Vector3 &point) const {
        return (point[0] >= _min[0] && point[0] <= _max[0]
             && point[1] >= _min[1] && point[1] <= _max[1]
             && point[2] >= _min[2] && point[2] <= _max[2]);
    }

    /// Returns true if the \p range is located entirely inside the range. As
    /// with all operations of this type, the ranges are assumed to include
    /// their extrema.
    bool contains(const Range3 &range) const {
        return contains(range._min) && contains(range._max);
    }

    /// Returns true if the \p point is located inside the range. As with all
    /// operations of this type, the range is assumed to include its extrema.
    /// \deprecated Use Contains() instead.
    bool isInside(const Vector3 &point) const {
        return contains(point);
    }

    /// Returns true if the \p range is located entirely inside the range. As
    /// with all operations of this type, the ranges are assumed to include
    /// their extrema.
    /// \deprecated Use Contains() instead.
    bool isInside(const Range3 &range) const {
        return contains(range);
    }

    /// Returns true if the \p range is located entirely outside the range. As
    /// with all operations of this type, the ranges are assumed to include
    /// their extrema.
    bool isOutside(const Range3 &range) const {
        return ((range._max[0] < _min[0] || range._min[0] > _max[0])
             || (range._max[1] < _min[1] || range._min[1] > _max[1])
             || (range._max[2] < _min[2] || range._min[2] > _max[2]));
    }

    /// Returns the smallest \c Range3 which contains both \p a and \p b.
    static Range3 getUnion(const Range3 &a, const Range3 &b) {
        Range3 res = a;
        findMin(res._min,b._min);
        findMax(res._max,b._max);
        return res;
    }

    /// Extend \p this to include \p b.
    const Range3 &unionWith(const Range3 &b) {
        findMin(_min,b._min);
        findMax(_max,b._max);
        return *this;
    }

    /// Extend \p this to include \p b.
    const Range3 &unionWith(const Vector3 &b) {
        findMin(_min,b);
        findMax(_max,b);
        return *this;
    }

    /// Returns a \c Range3 that describes the intersection of \p a and \p b.
    static Range3 getIntersection(const Range3 &a, const Range3 &b) {
        Range3 res = a;
        findMax(res._min,b._min);
        findMin(res._max,b._max);
        return res;
    }

    /// Returns a \c Range3 that describes the intersection of \p a and \p b.
    /// \deprecated Use GetIntersection() instead.
    static Range3 intersection(const Range3 &a, const Range3 &b) {
        return getIntersection(a, b);
    }

    /// Modifies this range to hold its intersection with \p b and returns the
    /// result
    const Range3 &intersectWith(const Range3 &b) {
        findMax(_min,b._min);
        findMin(_max,b._max);
        return *this;
    }

    /// Modifies this range to hold its intersection with \p b and returns the
    /// result.
    /// \deprecated Use IntersectWith() instead.
    const Range3 &intersection(const Range3 &b) {
        return intersectWith(b);
    }

    /// unary sum.
    Range3 operator +=(const Range3 &b) {
        _min += b._min;
        _max += b._max;
        return *this;
    }

    /// unary difference.
    Range3 operator -=(const Range3 &b) {
        _min -= b._max;
        _max -= b._min;
        return *this;
    }

    /// unary multiply.
    Range3 operator *=(float m) {
        if (m > 0) {
            _min *= m;
            _max *= m;
        } else {
            Vector3 tmp = _min;
            _min = _max * m;
            _max = tmp * m;
        }
        return *this;
    }

    /// unary division.
    Range3 operator /=(float m) {
        return *this *= (1.0 / m);
    }

    /// binary sum.
    Range3 operator +(const Range3 &b) const {
        return Range3(_min + b._min, _max + b._max);
    }


    /// binary difference.
    Range3 operator -(const Range3 &b) const {
        return Range3(_min - b._max, _max - b._min);
    }

    /// scalar multiply.
    friend Range3 operator *(float m, const Range3 &r) {
        return (m > 0 ? 
            Range3(r._min*m, r._max*m) : 
            Range3(r._max*m, r._min*m));
    }

    /// scalar multiply.
    friend Range3 operator *(const Range3 &r, float m) {
        return (m > 0 ? 
            Range3(r._min*m, r._max*m) : 
            Range3(r._max*m, r._min*m));
    }

    /// scalar divide.
    friend Range3 operator /(const Range3 &r, float m) {
        return r * (1.0 / m);
    }

    /// hash.
    /*
    friend inline size_t hash_value(const Range3 &r) {
        size_t h = 0;
        boost::hash_combine(h, r._min);
        boost::hash_combine(h, r._max);
        return h;
    }
     */

    /// The min and max points must match exactly for equality.
    bool operator ==(const Range3 &b) const {
        return (_min == b._min && _max == b._max);
    }

    bool operator !=(const Range3 &b) const {
        return !(*this == b);
    }

    /// Compute the squared distance from a point to the range
    float getDistanceSquared(const Vector3 &p) const;

    /// Returns the ith corner of the range, in the following order:
    /// LDB, RDB, LUB, RUB, LDF, RDF, LUF, RUF. Where L/R is left/right,
    /// D/U is down/up, and B/F is back/front.
    Vector3 getCorner(size_t i) const;

    /// Returns the ith octant of the range, in the following order:
    /// LDB, RDB, LUB, RUB, LDF, RDF, LUF, RUF. Where L/R is left/right,
    /// D/U is down/up, and B/F is back/front.
    Range3 getOctant(size_t i) const;

    /// The unit cube.
    static const Range3 unitCube;

  private:
    /// Minimum and maximum points.
    Vector3 _min, _max;

    /// Extends minimum point if necessary to contain given point.
    static void findMin(Vector3 &dest, const Vector3 &point) {
        if (point[0] < dest[0]) dest[0] = point[0];
        if (point[1] < dest[1]) dest[1] = point[1];
        if (point[2] < dest[2]) dest[2] = point[2];
    }

    /// Extends maximum point if necessary to contain given point.
    static void findMax(Vector3 &dest, const Vector3 &point) {
        if (point[0] > dest[0]) dest[0] = point[0];
        if (point[1] > dest[1]) dest[1] = point[1];
        if (point[2] > dest[2]) dest[2] = point[2];
    }
};

} // end namespace BOB
#endif // GF_RANGE3F_H
