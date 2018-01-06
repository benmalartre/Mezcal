#ifndef _ROTATION_H
#define _ROTATION_H

#include "common.h"
#include "quaternion.h"
#include "matrix4.h"
#include "vector3.h"

namespace BOB{
class Rotation {

  public:

    /// The default constructor leaves the rotation undefined.
    Rotation() {
    }

    /// This constructor initializes the rotation to be \p angle
    /// degrees about \p axis.
    Rotation(const Vector3 &axis, float angle) {
        setAxisAngle(axis, angle);
    }

    /// This constructor initializes the rotation from a quaternion.
    Rotation(const Quaternion &quaternion) {
        setQuaternion(quaternion);
    }

    /// This constructor initializes the rotation to one that brings
    /// the \p rotateFrom vector to align with \p rotateTo. The passed
    /// vectors need not be unit length.
    Rotation(const Vector3 &rotateFrom, const Vector3 &rotateTo) {
        setRotateInto(rotateFrom, rotateTo);
    }

    /// Sets the rotation to be \p angle degrees about \p axis.
    Rotation &        setAxisAngle(const Vector3 &axis, float angle) {
        _axis = axis;
        _angle = angle;
        if (_axis.dot(_axis)-1.0f>0.0000001f)
            _axis.normalizeInPlace();
        return *this;
    }

    /// Sets the rotation from a quaternion.  Note that this method accepts
    /// GfQuatf and GfQuath since they implicitly convert to GfQuatd.
    Rotation &        setQuaternion(const Quaternion &quat);

    /// Sets the rotation to one that brings the \p rotateFrom vector
    /// to align with \p rotateTo. The passed vectors need not be unit
    /// length.
    Rotation &        setRotateInto(const Vector3 &rotateFrom,
                                      const Vector3 &rotateTo);

    /// Sets the rotation to an identity rotation.
    /// (This is chosen to be 0 degrees around the positive X axis.)
    Rotation &        setIdentity() {
        _axis.set(1.0, 0.0, 0.0);
        _angle = 0.0;
        return *this;
    }

    /// Returns the axis of rotation.
    const Vector3 &     getAxis() const {
        return _axis;
    }

    /// Returns the rotation angle in degrees.
    float              getAngle() const {
        return _angle;
    }

    /// Returns the rotation expressed as a quaternion.
    Quaternion      getQuaternion() const;

    /// Returns the inverse of this rotation.
    Rotation          getInverse() const {
        return Rotation(_axis, -_angle);
    }

    /// Decompose rotation about 3 orthogonal axes. 
    /// If the axes are not orthogonal, warnings will be spewed.
    Vector3 decompose( const Vector3 &axis0,
                       const Vector3 &axis1,
                       const Vector3 &axis2 ) const;

    // Full-featured method to  Decompose a rotation matrix into Cardarian 
    // angles.
    // Axes have must be normalized. If useHint is specified
    // then the current values stored within thetaTw, thetaFB, thetaLR,
    // and thetaSw will be treated as hint and  used to help choose 
    // an equivalent rotation that is as close as possible to the hints.
    //
    // One can use this routine to generate any combination of the three 
    // angles by passing in NULL for the angle that is to be omitted.
    // 
    // Passing in valid pointers for all four angles will decompose into
    // Tw, FB, and LR but allows Sw to be used for best matching of hint 
    // values.  It also allows an swShift value to be passed in as a 
    // Sw that is applied after the rotation matrix to get a best fit rotation
    // in four angles.
    //
    // Angles are in radians.
    //
    // Specify \p handedness as -1.0 or 1.0, same as for MultiRotate.
    //
    // NOTE:
    // Geppetto math function Originally brought over to extMover 
    // from //depot/main/tools/src/menv/lib/gpt/util.h [10/16/06]
    // And moved into Rotation[12/1/08].  Updated for any 
    // combination of three angles [12/1/11].
    //
    static void decomposeRotation(const Matrix4 &rot,
                                  const Vector3 &TwAxis,
                                  const Vector3 &FBAxis,
                                  const Vector3 &LRAxis,
                                  float handedness,
                                  float *thetaTw,
                                  float *thetaFB,
                                  float *thetaLR,
                                  float *thetaSw = NULL,
                                  bool   useHint=false,
                                  const float *swShift=NULL);

    // This function projects the vectors \p v1 and \p v2 onto the plane 
    // normal to \p axis, and then returns the rotation about \p axis that 
    // brings \p v1 onto \p v2.
    static Rotation rotateOntoProjected(const Vector3 &v1,
                                          const Vector3 &v2,
                                          const Vector3 &axis);
    
    static Matrix4 rotateOntoProjected(const Vector3 &v1,
                                       const Vector3 &v2,
                                       const Vector3 &axisParam,
                                       float *thetaInRadians);

    /// Transforms row vector \p vec by the rotation, returning the result. 
    Vector3 transformDir( const Vector3 &vec ) const;

    /// Hash.
    /*
    friend inline size_t hash_value(const Rotation &r) {
        size_t h = 0;
        boost::hash_combine(h, r._axis);
        boost::hash_combine(h, r._angle);
        return h;
    }
	*/
    /// Component-wise rotation equality test. The axes and angles must match
    /// exactly for rotations to be considered equal. (To compare equality of
    /// the actual rotations, you can convert both to quaternions and test the
    /// results for equality.)
    bool        operator ==(const Rotation &r) const {
        return (_axis  == r._axis &&
            _angle == r._angle);
    }

    /// Component-wise rotation inequality test. The axes and angles must
    /// match exactly for rotations to be considered equal. (To compare
    /// equality of the actual rotations, you can convert both to quaternions
    /// and test the results for equality.)
    bool        operator !=(const Rotation &r) const {
        return ! (*this == r);
    }

    /// Post-multiplies rotation \p r into this rotation.
    Rotation &        operator *=(const Rotation &r);

    /// Scales rotation angle by multiplying by \p scale.
    Rotation &        operator *=(float scale) {
        _angle *= scale;
        return *this;
    }

    /// Scales rotation angle by dividing by \p scale.
    Rotation &    operator /=(float scale) {
        _angle /= scale;
        return *this;
    }

    /// Returns composite rotation of rotations \p r1 and \p r2.
    friend Rotation   operator *(const Rotation &r1,
                   const Rotation &r2) {
        Rotation r  = r1;
        return     r *= r2;
    }

    /// Returns a rotation equivalent to \p r with its angle multiplied
    /// by \p scale.
    friend Rotation   operator *(const Rotation &r, float scale) {
        Rotation rTmp  = r;
        return     rTmp *= scale;
    }

    /// Returns a rotation equivalent to \p r with its angle multiplied
    /// by \p scale.
    friend Rotation   operator *(float scale, const Rotation &r) {
        return (r * scale);
    }

    /// Returns a rotation equivalent to \p r with its angle divided
    /// by \p scale.
    friend Rotation   operator /(const Rotation &r, float scale) {
        Rotation rTmp  = r;
        return     rTmp /= scale;
    }

  private:
    /// Axis storage.
    /// This axis is normalized to unit length whenever it is set.
    Vector3     _axis;
    /// Angle storage (represented in degrees).
    float      _angle;
};

}// end namespace BOB

#endif // GF_ROTATION_H
