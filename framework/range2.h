#ifndef _RANGE2_H
#define _RANGE2_H

#include "common.h"
#include "vector2.h"

#include <cfloat>

namespace BOB{
class Range2
{
public:

    static const size_t dimension = 2;

    /// Sets the range to an empty interval
    // TODO check whether this can be deprecated.
    void inline setEmpty() {
        _min[0] = _min[1] =  FLT_MAX;
        _max[0] = _max[1] = -FLT_MAX;
    }

    /// The default constructor creates an empty range.
    Range2() {
        setEmpty();
    }

    /// This constructor initializes the minimum and maximum points.
    Range2(const Vector2 &min, const Vector2 &max)
        : _min(min), _max(max)
    {
    }

    /// Returns the minimum value of the range.
    const Vector2 &getMin() const { return _min; }

    /// Returns the maximum value of the range.
    const Vector2 &getMax() const { return _max; }

    /// Returns the size of the range.
    Vector2 getSize() const { return _max - _min; }

    /// Returns the midpoint of the range, that is, 0.5*(min+max).
    /// Note: this returns zero in the case of default-constructed ranges,
    /// or ranges set via SetEmpty().
    Vector2 getMidpoint() const {
        return Vector2(0.5f,0.5f) * _min
               + Vector2(0.5f,0.5f) * _max;
    }

    /// Sets the minimum value of the range.
    void setMin(const Vector2 &min) { _min = min; }

    /// Sets the maximum value of the range.
    void setMax(const Vector2 &max) { _max = max; }

    /// Returns whether the range is empty (max < min).
    bool isEmpty() const {
        return _min[0] > _max[0] || _min[1] > _max[1];
    }

    /// Modifies the range if necessary to surround the given value.
    /// \deprecated Use UnionWith() instead.
    void extendBy(const Vector2 &point) { unionWith(point); }

    /// Modifies the range if necessary to surround the given range.
    /// \deprecated Use UnionWith() instead.
    void extendBy(const Range2 &range) { unionWith(range); }

    /// Returns true if the \p point is located inside the range. As with all
    /// operations of this type, the range is assumed to include its extrema.
    bool contains(const Vector2 &point) const {
        return (point[0] >= _min[0] && point[0] <= _max[0]
             && point[1] >= _min[1] && point[1] <= _max[1]);
    }

    /// Returns true if the \p range is located entirely inside the range. As
    /// with all operations of this type, the ranges are assumed to include
    /// their extrema.
    bool contains(const Range2 &range) const {
        return contains(range._min) && contains(range._max);
    }

    /// Returns true if the \p point is located inside the range. As with all
    /// operations of this type, the range is assumed to include its extrema.
    /// \deprecated Use Contains() instead.
    bool isInside(const Vector2 &point) const {
        return contains(point);
    }

    /// Returns true if the \p range is located entirely inside the range. As
    /// with all operations of this type, the ranges are assumed to include
    /// their extrema.
    /// \deprecated Use Contains() instead.
    bool isInside(const Range2 &range) const {
        return contains(range);
    }

    /// Returns true if the \p range is located entirely outside the range. As
    /// with all operations of this type, the ranges are assumed to include
    /// their extrema.
    bool isOutside(const Range2 &range) const {
        return ((range._max[0] < _min[0] || range._min[0] > _max[0])
             || (range._max[1] < _min[1] || range._min[1] > _max[1]));
    }

    /// Returns the smallest \c GfRange2f which contains both \p a and \p b.
    static Range2 getUnion(const Range2 &a, const Range2 &b) {
        Range2 res = a;
        findMin(res._min,b._min);
        findMax(res._max,b._max);
        return res;
    }

    /// Extend \p this to include \p b.
    const Range2 &unionWith(const Range2 &b) {
        findMin(_min,b._min);
        findMax(_max,b._max);
        return *this;
    }

    /// Extend \p this to include \p b.
    const Range2 &unionWith(const Vector2 &b) {
        findMin(_min,b);
        findMax(_max,b);
        return *this;
    }

    /// Returns a \c GfRange2f that describes the intersection of \p a and \p b.
    static Range2 getIntersection(const Range2 &a, const Range2 &b) {
        Range2 res = a;
        findMax(res._min,b._min);
        findMin(res._max,b._max);
        return res;
    }

    /// Returns a \c GfRange2f that describes the intersection of \p a and \p b.
    /// \deprecated Use GetIntersection() instead.
    static Range2 intersection(const Range2 &a, const Range2 &b) {
        return getIntersection(a, b);
    }

    /// Modifies this range to hold its intersection with \p b and returns the
    /// result
    const Range2 &intersectWith(const Range2 &b) {
        findMax(_min,b._min);
        findMin(_max,b._max);
        return *this;
    }

    /// Modifies this range to hold its intersection with \p b and returns the
    /// result.
    /// \deprecated Use IntersectWith() instead.
    const Range2 &intersection(const Range2 &b) {
        return intersectWith(b);
    }

    /// unary sum.
    Range2 operator +=(const Range2 &b) {
        _min += b._min;
        _max += b._max;
        return *this;
    }

    /// unary difference.
    Range2 operator -=(const Range2 &b) {
        _min -= b._max;
        _max -= b._min;
        return *this;
    }

    /// unary multiply.
    Range2 operator *=(double m) {
        if (m > 0) {
            _min *= m;
            _max *= m;
        } else {
            Vector2 tmp = _min;
            _min = _max * m;
            _max = tmp * m;
        }
        return *this;
    }

    /// unary division.
    Range2 operator /=(double m) {
        return *this *= (1.0 / m);
    }

    /// binary sum.
    Range2 operator +(const Range2 &b) const {
        return Range2(_min + b._min, _max + b._max);
    }


    /// binary difference.
    Range2 operator -(const Range2 &b) const {
        return Range2(_min - b._max, _max - b._min);
    }

    /// scalar multiply.
    friend Range2 operator *(double m, const Range2 &r) {
        return (m > 0 ? 
            Range2(r._min*m, r._max*m) :
            Range2(r._max*m, r._min*m));
    }

    /// scalar multiply.
    friend Range2 operator *(const Range2 &r, double m) {
        return (m > 0 ? 
            Range2(r._min*m, r._max*m) :
            Range2(r._max*m, r._min*m));
    }

    /// scalar divide.
    friend Range2 operator /(const Range2 &r, double m) {
        return r * (1.0 / m);
    }

    /// hash.
    /*
    friend inline size_t hash_value(const Range2 &r) {
        size_t h = 0;
        boost::hash_combine(h, r._min);
        boost::hash_combine(h, r._max);
        return h;
    }
*/
    /// The min and max points must match exactly for equality.
    bool operator ==(const Range2 &b) const {
        return (_min == b._min && _max == b._max);
    }

    bool operator !=(const Range2 &b) const {
        return !(*this == b);
    }

    /// Compute the squared distance from a point to the range.
    float getDistanceSquared(const Vector2 &p) const;

    /// Returns the ith corner of the range, in the following order:
    /// SW, SE, NW, NE.
    Vector2 getCorner(size_t i) const;

    /// Returns the ith quadrant of the range, in the following order:
    /// SW, SE, NW, NE.
    Range2 getQuadrant(size_t i) const;

    /// The unit square.
    static const Range2 unitSquare;

  private:
    /// Minimum and maximum points.
    Vector2 _min, _max;

    /// Extends minimum point if necessary to contain given point.
    static void findMin(Vector2 &dest, const Vector2 &point) {
        if (point[0] < dest[0]) dest[0] = point[0];
        if (point[1] < dest[1]) dest[1] = point[1];
    }

    /// Extends maximum point if necessary to contain given point.
    static void findMax(Vector2 &dest, const Vector2 &point) {
        if (point[0] > dest[0]) dest[0] = point[0];
        if (point[1] > dest[1]) dest[1] = point[1];
    }
};
} // end namespace BOB
#endif // GF_RANGE2F_H
