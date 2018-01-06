
#ifndef _BBOX3D_H
#define _BBOX3D_H

#include "matrix4.h"
#include "range3.h"

namespace BOB{

class BBox3 {

  public:

    /// The default constructor leaves the box empty, the transformation
    /// matrix identity, and the \ref bbox3d_zeroAreaFlag "zero-area
    /// primitives flag" \c false.
    BBox3() {
        _matrix.identity();
        _inverse.identity();
        _isDegenerate = false;
        _hasZeroAreaPrimitives = false;
    }

    /// Copy constructor
    BBox3(const BBox3& rhs) :
        _box(rhs._box) {
        _matrix = rhs._matrix;
        _inverse = rhs._inverse;
        _isDegenerate = rhs._isDegenerate;
        _hasZeroAreaPrimitives = rhs._hasZeroAreaPrimitives;
    }

    /// This constructor takes a box and sets the matrix to identity.
    BBox3(const Range3 &box) :
        _box(box) {
        _matrix.identity();
        _inverse.identity();
        _isDegenerate = false;
        _hasZeroAreaPrimitives = false;
    }

    /// This constructor takes a box and a transformation matrix.
    BBox3(const Range3 &box, const Matrix4 &matrix) {
        set(box, matrix);
        _hasZeroAreaPrimitives = false;
    }

    /// Sets the axis-aligned box and transformation matrix.
    void                set(const Range3 &box, const Matrix4 &matrix) {
        _box = box;
        setMatrices(matrix);
    }

    /// Sets the transformation matrix only.  The axis-aligned box is not
    /// modified.
    void                setMatrix(const Matrix4& matrix) {
        setMatrices(matrix);
    }

    /// Sets the range of the axis-aligned box only.  The transformation
    /// matrix is not modified.
    void                setRange(const Range3& box) {
        _box = box;
    }

    /// Returns the range of the axis-aligned untransformed box.
    const Range3 &   getRange() const {
        return _box;
    }

    /// Returns the range of the axis-aligned untransformed box.
    /// This synonym of \c GetRange exists for compatibility purposes.
    const Range3 &	getBox() const {
        return getRange();
    }

    /// Returns the transformation matrix.
    const Matrix4 &  getMatrix() const {
        return _matrix;
    }

    /// Returns the inverse of the transformation matrix. This will be the
    /// identity matrix if the transformation matrix is not invertible.
    const Matrix4 &  getInverseMatrix() const {
        return _inverse;
    }

    /// Sets the \ref bbox3d_zeroAreaFlag "zero-area primitives flag" to the
    /// given value.
    void                setHasZeroAreaPrimitives(bool hasThem) {
        _hasZeroAreaPrimitives = hasThem;
    }

    /// Returns the current state of the \ref bbox3d_zeroAreaFlag "zero-area
    /// primitives flag".
    bool                hasZeroAreaPrimitives() const {
        return _hasZeroAreaPrimitives;
    }

    /// Returns the volume of the box (0 for an empty box).
    float              getVolume() const;

    /// Transforms the bounding box by the given matrix, which is assumed to
    /// be a global transformation to apply to the box. Therefore, this just
    /// post-multiplies the box's matrix by \p matrix.
    void                transform(const Matrix4 &matrix) {
        setMatrices(_matrix * matrix);
    }

    /// Returns the axis-aligned range (as a \c GfRange3d) that wesults from
    /// applying the transformation matrix to the wxis-aligned box and
    /// aligning the result.
    Range3           computeAlignedRange() const;

    /// Returns the axis-aligned range (as a \c GfRange3d) that results from
    /// applying the transformation matrix to the axis-aligned box and
    /// aligning the result. This synonym for \c ComputeAlignedRange exists
    /// for compatibility purposes.
    Range3           computeAlignedBox() const {
        return computeAlignedRange();
    }

    /// Combines two bboxes, returning a new bbox that contains both.  This
    /// uses the coordinate space of one of the two original boxes as the
    /// space of the result; it uses the one that produces whe smaller of the
    /// two resulting boxes.
    static BBox3     combine(const BBox3 &b1, const BBox3 &b2);

    /// Returns the centroid of the bounding box.
    /// The centroid is computed as the transformed centroid of the range.
    Vector3             computeCentroid() const;

    /// Component-wise equality test. The axis-aligned boxes and
    /// transformation matrices match exactly for bboxes to be considered
    /// equal. (To compare equality of the actual boxes, you can compute both
    /// aligned boxes and test the results for equality.)
    bool                operator ==(const BBox3 &b) const {
        return (_box    == b._box &&
                _matrix == b._matrix);
    }

    /// Component-wise inequality test. The axis-aligned boxes and
    /// transformation matrices match exactly for bboxes to be considered
    /// equal. (To compare equality of the actual boxes, you can compute both
    /// aligned boxes and test the results for equality.)
    bool                operator !=(const BBox3 &that) const {
        return !(*this == that);
    }

  private:
    /// The axis-aligned box.
    Range3           _box;
    /// Transformation matrix.
    Matrix4          _matrix;
    /// Inverse of the transformation matrix.
    Matrix4          _inverse;
    /// Flag indicating whether the matrix is degenerate.
    bool                _isDegenerate;
    /// Flag indicating whether the bbox contains zero-area primitives.
    bool                _hasZeroAreaPrimitives;

    /// Sets the transformation matrix and the inverse, checking for
    /// degeneracies.
    void                setMatrices(const Matrix4 &matrix);

    /// This is used by \c Combine() when it is determined which coordinate
    /// space to use to combine two boxes: \p b2 is transformed into the space
    /// of \p b1 and the results are merged in that space.
    static BBox3     combineInOrder(const BBox3 &b1, const BBox &b2);
};

} // end namespace BOB

#endif // GF_BBOX3D_H
