#ifndef _RANGE1_H
#define _RANGE1_H

#include "common.h"
#include <cfloat>
#include <math.h>

namespace BOB{
class Range1
{
public:

    static const size_t dimension = 1;

    /// Sets the range to an empty interval
    // TODO check whether this can be deprecated.
    void inline setEmpty() {
		_min =  FLT_MAX;
		_max = -FLT_MAX;
    }

    /// The default constructor creates an empty range.
    Range1() {
        setEmpty();
    }

    /// This constructor initializes the minimum and maximum points.
    Range1(float min, float max)
        : _min(min), _max(max)
    {
    }

    /// Returns the minimum value of the range.
    float getMin() const { return _min; }

    /// Returns the maximum value of the range.
    float getMax() const { return _max; }

    /// Returns the size of the range.
    float getSize() const { return _max - _min; }

    /// Returns the midpoint of the range, that is, 0.5*(min+max).
    /// Note: this returns zero in the case of default-constructed ranges,
    /// or ranges set via SetEmpty().
    float getMidpoint() const {
        return static_cast<float>(0.5) * _min
               + static_cast<float>(0.5) * _max;
    }

    /// Sets the minimum value of the range.
    void setMin(float min) { _min = min; }

    /// Sets the maximum value of the range.
    void setMax(float max) { _max = max; }

    /// Returns whether the range is empty (max < min).
    bool isEmpty() const {
        return _min > _max;
    }

    /// Modifies the range if necessary to surround the given value.
    /// \deprecated Use UnionWith() instead.
    void extendBy(float point) { unionWith(point); }

    /// Modifies the range if necessary to surround the given range.
    /// \deprecated Use UnionWith() instead.
    void extendBy(const Range1 &range) { unionWith(range); }

    /// Returns true if the \p point is located inside the range. As with all
    /// operations of this type, the range is assumed to include its extrema.
    bool contains(float point) const {
        return (point >= _min && point <= _max);
    }

    /// Returns true if the \p range is located entirely inside the range. As
    /// with all operations of this type, the ranges are assumed to include
    /// their extrema.
    bool contains(const Range1 &range) const {
        return contains(range._min) && contains(range._max);
    }

    /// Returns true if the \p point is located inside the range. As with all
    /// operations of this type, the range is assumed to include its extrema.
    /// \deprecated Use Contains() instead.
    bool isInside(float point) const {
        return contains(point);
    }

    /// Returns true if the \p range is located entirely inside the range. As
    /// with all operations of this type, the ranges are assumed to include
    /// their extrema.
    /// \deprecated Use Contains() instead.
    bool isInside(const Range1 &range) const {
        return contains(range);
    }

    /// Returns true if the \p range is located entirely outside the range. As
    /// with all operations of this type, the ranges are assumed to include
    /// their extrema.
    bool isOutside(const Range1 &range) const {
        return (range._max < _min || range._min > _max);
    }

    /// Returns the smallest \c Range1 which contains both \p a and \p b.
    static Range1 getUnion(const Range1 &a, const Range1 &b) {
        Range1 res = a;
        findMin(res._min,b._min);
        findMax(res._max,b._max);
        return res;
    }

    /// Extend \p this to include \p b.
    const Range1 &unionWith(const Range1 &b) {
        findMin(_min,b._min);
        findMax(_max,b._max);
        return *this;
    }

    /// Extend \p this to include \p b.
    const Range1 &unionWith(float b) {
        findMin(_min,b);
        findMax(_max,b);
        return *this;
    }

    /// Returns a \c Range1 that describes the intersection of \p a and \p b.
    static Range1 getIntersection(const Range1 &a, const Range1 &b) {
        Range1 res = a;
        findMax(res._min,b._min);
        findMin(res._max,b._max);
        return res;
    }

    /// Returns a \c Range1 that describes the intersection of \p a and \p b.
    /// \deprecated Use GetIntersection() instead.
    static Range1 intersection(const Range1 &a, const Range1 &b) {
        return getIntersection(a, b);
    }

    /// Modifies this range to hold its intersection with \p b and returns the
    /// result
    const Range1 &intersectWith(const Range1 &b) {
        findMax(_min,b._min);
        findMin(_max,b._max);
        return *this;
    }

    /// Modifies this range to hold its intersection with \p b and returns the
    /// result.
    /// \deprecated Use IntersectWith() instead.
    const Range1 &intersection(const Range1 &b) {
        return intersectWith(b);
    }

    /// unary sum.
    Range1 operator +=(const Range1 &b) {
        _min += b._min;
        _max += b._max;
        return *this;
    }

    /// unary difference.
    Range1 operator -=(const Range1 &b) {
        _min -= b._max;
        _max -= b._min;
        return *this;
    }

    /// unary multiply.
    Range1 operator *=(float m) {
        if (m > 0) {
            _min *= m;
            _max *= m;
        } else {
            float tmp = _min;
            _min = _max * m;
            _max = tmp * m;
        }
        return *this;
    }

    /// unary division.
    Range1 operator /=(float m) {
        return *this *= (1.0 / m);
    }

    /// binary sum.
    Range1 operator +(const Range1 &b) const {
        return Range1(_min + b._min, _max + b._max);
    }


    /// binary difference.
    Range1 operator -(const Range1 &b) const {
        return Range1(_min - b._max, _max - b._min);
    }

    /// scalar multiply.
    friend Range1 operator *(float m, const Range1 &r) {
        return (m > 0 ? 
            Range1(r._min*m, r._max*m) : 
            Range1(r._max*m, r._min*m));
    }

    /// scalar multiply.
    friend Range1 operator *(const Range1 &r, float m) {
        return (m > 0 ? 
            Range1(r._min*m, r._max*m) : 
            Range1(r._max*m, r._min*m));
    }

    /// scalar divide.
    friend Range1 operator /(const Range1 &r, float m) {
        return r * (1.0 / m);
    }

    /// hash.
    /*
    friend inline size_t hash_value(const Range1 &r) {
        size_t h = 0;
        boost::hash_combine(h, r._min);
        boost::hash_combine(h, r._max);
        return h;
    }
     */

    /// The min and max points must match exactly for equality.
    bool operator ==(const Range1 &b) const {
        return (_min == b._min && _max == b._max);
    }

    bool operator !=(const Range1 &b) const {
        return !(*this == b);
    }

    /// Compute the squared distance from a point to the range.
    float getDistanceSquared(float p) const;


  private:
    /// Minimum and maximum points.
    float _min, _max;

    /// Extends minimum point if necessary to contain given point.
    static void findMin(float &dest, float point) {
        if (point < dest) dest = point;
    }

    /// Extends maximum point if necessary to contain given point.
    static void findMax(float &dest, float point) {
        if (point > dest) dest = point;
    }
};

}// end namespace BOB

#endif // _RANGE1_H
