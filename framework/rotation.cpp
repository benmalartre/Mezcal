
#include "rotation.h"
#include "matrix3.h"
#include "matrix4.h"
#include "utils.h"
namespace BOB{



Rotation & Rotation::setRotateInto(const Vector3 &rotateFrom, const Vector3 &rotateTo)
{
    Vector3 from = rotateFrom.normalize();
    Vector3 to   = rotateTo.normalize();

    float cos = from.dot(to);

    // If vectors are close enough to parallel, use the identity
    // rotation
    if (cos > 0.9999999f)
        return setIdentity();

    // If vectors are opposite, rotate by 180 degrees around an axis
    // vector perpendicular to the original axis.
    if (cos < -0.9999999f) {
        // Try cross product with X axis first.  If that's too close
        // to the original axis, use the Y axis
        Vector3 tmp = from.cross(Vector3(1.0, 0.0, 0.0));
        if (tmp.length() < 0.00001f)
            tmp = from.cross(Vector3(0.0, 1.0, 0.0));
        return setAxisAngle(tmp.normalize(), 180.0);
    }

    // Generic case: compute the rotation to bring the vectors
    // together.
    Vector3 axis = rotateFrom.cross(rotateTo);
    axis.normalizeInPlace();
    return setAxisAngle(axis, acos(cos) * RADIAN_TO_DEGREE);
}

Quaternion Rotation::getQuaternion() const
{
    float radians = (_angle * DEGREE_TO_RADIAN) *0.5f;
    float sinR, cosR;
    sinR = sinf(radians);
    cosR = cosf(radians);
    Vector3 axis = _axis *sinR;
    //Quaternion q(axis, cosR);
    Quaternion q = quaternionFromAxisAngle(axis, cosR);
    q.normalizeInPlace();
    return q;
}

// helper function for Decompose and DecomposeRotation
static double getEpsilon() {
    return 1e-6;
}

Vector3 Rotation::decompose( const Vector3 &axis0,
                            const Vector3 &axis1,
                       		const Vector3 &axis2 ) const
{
    Matrix4 mat;
    mat.setRotate(_axis.x, _axis.y, _axis.z, _angle );

    // Build the axes tensors
    Vector3 nAxis0 = axis0.normalize();
    Vector3 nAxis1 = axis1.normalize();
    Vector3 nAxis2 = axis2.normalize();

    // Use GF_MIN_ORTHO_TOLERANCE to match OrthogonalizeBasis().
    // XXX Should add GfAreOrthogonal(v0, v1, v2) (which also
    //     GfMatrix4d::HasOrthogonalRows3() could use).
    if (!(fabsf(nAxis0.dot(nAxis1))>0.0000001f &&
        fabsf(nAxis0.dot(nAxis2))>0.0000001f &&
    	fabsf(nAxis1.dot(nAxis2))>0.0000001f))
    {
        //TF_WARN("Rotation axes are not orthogonal.");
    }

    Matrix4 axes( nAxis0[0], nAxis1[0], nAxis2[0], 0.0f,
                	nAxis0[1], nAxis1[1], nAxis2[1], 0.0f,
                    nAxis0[2], nAxis1[2], nAxis2[2], 0.0f,
                     0.0f, 0.0f, 0.0f, 1.0f );

    // get a transformation that takes the given axis into a coordinate
    // frame that has those axis aligned with the x,y,z axis.
    Matrix4 m = axes.transpose() * mat * axes;

    // Decompose to the 3 rotations around the major axes.
    // The following code was taken from Graphic Gems 4 p 222. 
    // Euler Angle Conversion by Ken Shoemake.
    float r0, r1, r2;
    float cy = sqrtf(m[0]*m[0] + m[4]*m[4]);
    if (cy > getEpsilon()) {
        r0 = atan2f(m[9], m[10]);
        r1 = atan2f(-m[8], cy);
        r2 = atan2f(m[4], m[0]);
    } else {
        r0 = atan2f(-m[6], m[5]);
        r1 = atan2f(-m[8], cy);
        r2 = 0;
    }

    // Check handedness of matrix
    Vector3 axisCross = nAxis0.cross(nAxis1 );
    float axisHand = axisCross.dot(nAxis2 );
    if (axisHand >= 0.0f) {
        r0 = -r0;
        r1 = -r1;
        r2 = -r2;
    }

    return Vector3( r0 * RADIAN_TO_DEGREE,
                    r1 * RADIAN_TO_DEGREE,
                    r2 * RADIAN_TO_DEGREE);
}

// Brought over to ExtMover 
// from //depot/main/tools/src/menv/lib/gpt/util.h [10/16/06] 
//
//  CfgAffineMapd -> GfMatrix4d
//    ::Rotation(org, axis, *theta) ->  SetRotate(GfRotation(axis, *theta)) +  set position
//    ::Apply(CfgAffineMapd)  ->    mx4a.Apply(mx4b)    -> Compose(*mx4a, mx4b) -> mx4b * mx4a.  I think.
//  CfgVectord -> GfVec3d
//    ::Dot -> GfDot
//    ::DualCross -> GfCross  ?
//  CfgPointd -> GfVec3d

Rotation Rotation::rotateOntoProjected(const Vector3 &v1,
                                const Vector3 &v2,
                                const Vector3 &axisParam)
{
    Vector3 axis = axisParam.normalize();
    
    Vector3 v1Proj = v1 - axis * v1.dot(axis);
    Vector3 v2Proj = v2 - axis * v2.dot(axis);
    v1Proj.normalizeInPlace();
    v2Proj.normalizeInPlace();
    Vector3 crossAxis = v1Proj.cross(v2Proj);
    float sinTheta = crossAxis.dot(axis);
    float cosTheta = v1Proj.dot(v2Proj);
    float theta = 0;
    if (!(fabs(sinTheta) < getEpsilon() && fabs(cosTheta) < getEpsilon()))
        theta = atan2f(sinTheta, cosTheta);

    const float toDeg = (180.0f)/M_PI;
    return Rotation(axis, theta * toDeg); // Rotation takes angle in degrees
}

// helper function for decomposeRotation: Gets the rotation as a matrix and
// returns theta in radians instead of degrees.
    Matrix4 Rotation::rotateOntoProjected(const Vector3 &v1,
                    const Vector3 &v2,
                    const Vector3 &axisParam,
                    float *thetaInRadians)
{
    Matrix4 mat;
    Rotation r = Rotation::rotateOntoProjected(v1, v2, axisParam);
    Vector3 axis = r.getAxis();
    mat.setRotate(axis.x,axis.y,axis.z,r.getAngle());
    if (thetaInRadians) {
        const float toDeg = (180.0f)/M_PI;
        *thetaInRadians = r.getAngle() / toDeg;
    }

    return mat;
}

// helper function for DecomposeRotation
// Given a vector of hint euler angles, alter the desired attempt values such 
// that each is the closest multiple of itself of 2pi to its respective hint.
static Vector4 piShift(
    const Vector4 &hint, const Vector4 &attempt, float mul=2*M_PI)
{
    Vector4 result(attempt);
    for (int i = 0; i < 4; i++)
    {
        float      mod1 = fmodf(attempt[i], mul);
        float      mod2 = fmodf(hint[i], mul);
        result[i] = (hint[i]-mod2)+mod1;
        if (fabsf(hint[i]-result[i])>mul/2.0)
            result[i]+=(hint[i]<0?-mul:mul);
    }
    return result;
}

// Another helper function to readjust the first and last angles of a three
// euler anlge solution when the middle angle collapses first and last angles'
// axes onto each other.
static void shiftGimbalLock(
    float middleAngle, float *firstAngle, float *lastAngle)
{
    // If the middle angle is PI or -PI, we flipped the axes so use the
    // difference of the two angles.
    if (fabsf(fabsf(middleAngle) - M_PI) < getEpsilon()) {
        float diff = *lastAngle - *firstAngle;
        *lastAngle = diff/2;
        *firstAngle = -diff/2;
    }

    // If the middle angle is 0, then the two axes have the same effect so use
    // the sum of the angles.
    if (fabsf(middleAngle) < getEpsilon() ) {
        float sum = *lastAngle + *firstAngle;
        *lastAngle = sum/2;
        *firstAngle = sum/2;
    }
}

void Rotation::decomposeRotation(const Matrix4 &rot,
                           const Vector3 &TwAxis,
                           const Vector3 &FBAxis,
                           const Vector3 &LRAxis,
                           float handedness,
                           float *thetaTw,
                           float *thetaFB,
                           float *thetaLR,
                           float *thetaSw,
                           bool    useHint,
                           const float *swShift)
{
    // Enum of which angle is being zeroed out when decomposing the roatation.
    // This is determined by which angle output (if any) is NULL.
    enum ZeroAngle {
        ZERO_NONE = 0,
        ZERO_TW,
        ZERO_FB,
        ZERO_LR,
        ZERO_SW
    };
    ZeroAngle zeroAngle = ZERO_NONE;

    float angleStandin = 0.0f, hintTw=0.0f, hintFB=0.0f, hintLR=0.0f,hintSw=0.0f;
    if (thetaTw == NULL) {
        zeroAngle = ZERO_TW;
        thetaTw = &angleStandin;
    }
    if (thetaFB == NULL) {
        if (zeroAngle != ZERO_NONE) {
            //TF_CODING_ERROR("Need three angles to correctly decompose rotation");
            return;
        }
        zeroAngle = ZERO_FB;
        thetaFB = &angleStandin;
    }
    if (thetaLR == NULL) {
        if (zeroAngle != ZERO_NONE) {
            //TF_CODING_ERROR("Need three angles to correctly decompose rotation");
            return;
        }
        zeroAngle = ZERO_LR;
        thetaLR = &angleStandin;
    }
    if (thetaSw == NULL) {
        if (zeroAngle != ZERO_NONE) {
            //TF_CODING_ERROR("Need three angles to correctly decompose rotation");
            return;
        }
        zeroAngle = ZERO_SW;
        thetaSw = &angleStandin;
    }

    if (swShift && zeroAngle != ZERO_NONE) {
        //TF_WARN("A swing shift was provided but we're not decomposing into"
        //        " four angles.  The swing shift will be ignored.");
    }

    // Update hint values if we're using them as hints.
    if (useHint)
    {
        if (thetaTw) hintTw = *thetaTw ;
        if (thetaFB) hintFB = *thetaFB ;
        if (thetaLR) hintLR = *thetaLR ;
        if (thetaSw) hintSw = *thetaSw ;
    }

    // Apply the matrix to the axes.
    Vector3 FBAxisR = FBAxis.transformDir(rot);
    Vector3 TwAxisR = TwAxis.transformDir(rot);

    // do three rotates about the euler axes, in reverse order, that bring
    // the transformed axes back onto the originals.  The resulting rotation 
    // is the inverse of rot, and the angles are the negatives of the euler 
    // angles.
    Matrix4 r;

    // The angles used and what order we rotate axes is determined by which
    // angle we're not decomposing into.
    switch (zeroAngle)
    {
    case ZERO_SW:
    case ZERO_NONE:
        r = r * rotateOntoProjected(TwAxisR.transformDir(r), TwAxis, LRAxis, thetaLR);
        r = r * rotateOntoProjected(TwAxisR.transformDir(r), TwAxis, FBAxis, thetaFB);
        r = r * rotateOntoProjected(FBAxisR.transformDir(r), FBAxis, TwAxis, thetaTw);
            
        // negate the angles
        *thetaFB *= -handedness;
        *thetaLR *= -handedness;
        *thetaTw *= -handedness;

        // Set Sw to swShift if there is a swing shift, otherwise Sw is 
        // zeroed out.
        *thetaSw = swShift ? *swShift : 0.0;
        break;

    case ZERO_TW:
        r = r * rotateOntoProjected(FBAxisR.transformDir(r), FBAxis, TwAxis, thetaSw);
        r = r * rotateOntoProjected(FBAxisR.transformDir(r), FBAxis, LRAxis, thetaLR);
        r = r * rotateOntoProjected(TwAxisR.transformDir(r), TwAxis, FBAxis, thetaFB);
        // negate the angles
        *thetaSw *= -handedness;
        *thetaFB *= -handedness;
        *thetaLR *= -handedness;
        break;

    case ZERO_FB:
        r = r * rotateOntoProjected(TwAxisR.transformDir(r), FBAxis, TwAxis, thetaSw);
        r = r * rotateOntoProjected(TwAxisR.transformDir(r), TwAxis, LRAxis, thetaLR);
        r = r * rotateOntoProjected(FBAxisR.transformDir(r), FBAxis, TwAxis, thetaTw);
        // negate the angles
        *thetaSw *= -handedness;
        *thetaLR *= -handedness;
        *thetaTw *= -handedness;
        break;

    case ZERO_LR:
        r = r * rotateOntoProjected(TwAxisR.transformDir(r), LRAxis, TwAxis, thetaSw);
        r = r * rotateOntoProjected(TwAxisR.transformDir(r), TwAxis, FBAxis, thetaFB);
        r = r * rotateOntoProjected(FBAxisR.transformDir(r), FBAxis, TwAxis, thetaTw);
        // negate the angles
        *thetaSw *= -handedness;
        *thetaFB *= -handedness;
        *thetaTw *= -handedness;
        break;
    };

    // The decomposition isn't unique.  Obviously, adding multiples of
    // 2pi is a no-op, but we've already coerced each angle onto the
    // interval [-pi, pi].  With 3 angles, you can also add an odd
    // multiple of pi to each angle, and negate the middle one.
    //
    // To understand this: Rotating by pi around 1 axis flips the
    // other 2.  To get back where you started, you've got to flip
    // each axis by pi with even parity.  angles are negated if
    // there've been odd flips at the time that their rotation is
    // applied.
    //  
    // Since we've got a 4th axis, we can apply the identity to the
    // 1st three angles, or the last 3, or the 1st 3 then the last 3
    // (or vice versa - they commute.)  That, plus leaving the angles
    // alone, gives us 4 distinct choices.
    //
    // We want to choose the one that minimizes sum of abs of the
    // angles.  We do the miniscule combinatorial optimization
    // exhaustively.

    //  Each angle flipped by pi in the min abs direction.
    float thetaLRp = *thetaLR + ( (*thetaLR > 0)? -M_PI : M_PI);
    float thetaFBp = *thetaFB + ( (*thetaFB > 0)? -M_PI : M_PI);
    float thetaTwp = *thetaTw + ( (*thetaTw > 0)? -M_PI : M_PI);
    float thetaSwp = *thetaSw + ( (*thetaSw > 0)? -M_PI : M_PI);

    // fill up a table with the possible transformations:
    //  0 - do nothing
    //  1 - transform 1st 3
    //  2 - 1 & 3 composed
    //  3 - transform last 3
    Vector4 vals[4];
    vals[0] = Vector4( *thetaTw,  *thetaFB,     *thetaLR,     *thetaSw );
    vals[1] = Vector4( thetaTwp,  -thetaFBp,    thetaLRp,     *thetaSw );
    vals[2] = Vector4( thetaTwp,  -(*thetaFB),  -(*thetaLR),  thetaSwp );
    vals[3] = Vector4( *thetaTw,  thetaFBp,     -thetaLRp,    thetaSwp );

    // All four transforms are valid if we're not forcing any of the angles
    // to zero, but if we are zeroing an angle, then we only have to valid
    // options, the ones that don't flip the zeroed angle by pi.
    int numVals = zeroAngle == ZERO_NONE ? 4 : 2;
    switch (zeroAngle)
    {
    case ZERO_TW:
        vals[1] = vals[3];
        break;
    case ZERO_FB:
    case ZERO_LR:
        vals[1] = vals[2];
        break;
    default:
        break;
    };

    // Store the hint angles in a Tw,FB,LR,Sw ordered array to use for :
    //  1) 2*pi-Shifting
    //  2) calculating sum of absolute differences in order to select 
    //          final angle solution from candidates.
    Vector4 hintAngles(hintTw, hintFB, hintLR, hintSw);

    // If using hint, then alter our 4 euler angle component values
    // to get them into a per angle mult 2*M_PI that is as close as 
    // possible to the hint angles.
    if (useHint ) {
        for (size_t i=0; i<4;i++) {
            vals[i] =  piShift(hintAngles, vals[i]) ;
        }
    }

    // find the min of weighted sum of abs.  The weight on the 2nd
    // angle is to ensure that it stays on [-pi/2, pi/2] If swing
    // isn't wired up, we leave out the 4-axis identity unless the client
    // has passed a swing shift value representing the target sw, with a
    // hint in the sw.
    //
    // if using hint angles, then we sum the differences between the
    // original angle hints and our candidates and select the min
    //
    float min = 0;
    int  i, j, mini = -1;

    for (i = 0; i < numVals; i++) {
        float sum = 0;
        Vector4 hintDiff = vals[i]-hintAngles;
        for(j = 0;  j < 4; j++)
            sum += fabs(hintDiff[j]);
        if( (i == 0) || (sum < min) ) {
            min = sum;
            mini = i;
        }
    }

    // install the answer.
    *thetaTw = vals[mini][0];
    *thetaFB = vals[mini][1];
    *thetaLR = vals[mini][2];
    *thetaSw = vals[mini][3];

    // Oh, but there's more: Take the example of when we're decomposing
    // into tw, fb, and lr. When the middle angle, (fb) is PI/2, then
    // only (tw - lr) is significant, and at fb = -PI/2, only tw+lr is
    // significant, i.e. adding the same constant to both angles is an
    // identity.  Once again, we apply the min sum of abs rule.  This
    // happens because the PI/2 rotation collapses axis 1 onto axis 3.
    // That's what gimbal lock is.
    // 
    // This applies no matter which three angles we're decomposing into
    // except that in the case where we're solving tw, fb, sw or tw, lr, sw
    // we get this gimbal lock situation when the respective fb or lr are 
    // 0, PI, and -PI.  We can account for all these cases in the same 
    // function by shift fb and lr by PI/2 or -PI/2 when they are the middle 
    // angles.  Whether the shift is PI/2 or -PI/2 is dependent on the 
    // handedness of the basis matrix of the three axes as it flips the 
    // direction needed to the get the positive Tw or FB axis to align with 
    // the positive LR or Sw axis.
    Matrix3 basis;
    basis.setRow(0, TwAxis);
    basis.setRow(1, FBAxis);
    basis.setRow(2, LRAxis);
    switch (zeroAngle)
    {
    case ZERO_NONE:
    case ZERO_SW:
        shiftGimbalLock(*thetaFB + M_PI/2 * basis.getHandedness(), thetaTw, thetaLR);
        break;
    case ZERO_TW:
        shiftGimbalLock(*thetaLR + M_PI/2 * basis.getHandedness(), thetaFB, thetaSw);
        break;
    case ZERO_FB:
        shiftGimbalLock(*thetaLR, thetaTw, thetaSw);
        break;
    case ZERO_LR:
        shiftGimbalLock(*thetaFB, thetaTw, thetaSw);
        break;
    };
}


#if 0
// XXX: I ported this code over to presto, but it is not
// yet being used. 

void 
Rotation::composeRotation(float         tw,
                         float         fb,
                         float         lr,
                         float         sw,
                         Matrix4     *rot,
                         Vector3        *TwAxis,
                         Vector3        *FBAxis,
                         Vector3        *LRAxis)
{
    Vector3             twAxis(0,0,1),
                        fbAxis(1,0,0),
                        lrAxis(0,1,0);

    std::vector<Matrix4> matVec;
    matVec.resize(4);
    matVec[0].setRotate(Rotation(twAxis,tw));
    matVec[1].setRotate(Rotation(fbAxis,fb));
    matVec[2].setRotate(Rotation(lrAxis,lr));
    matVec[3].setRotate(Rotation(twAxis,sw));

    Matrix4 mat(1) ;
    for (size_t i=0; i< 4; i++) mat*=matVec[i];

    *rot = mat;
    *TwAxis = twAxis ;
    *FBAxis = fbAxis ;
    *LRAxis = lrAxis ;
}

#endif

Vector3 Rotation::transformDir( const Vector3 &vec ) const
{
    Matrix4 r;
    r.setRotate(_axis.x,_axis.y,_axis.z,_angle);
    return vec.transformDir(r);
}

Rotation & Rotation::operator *=(const Rotation &r)
{
    // Express both rotations as quaternions and multiply them
    Quaternion q = r.getQuaternion();
/*
    // We don't want to just call
    //          SetQuaternion(q);
    // here, because that could change the axis if the angle is a
    // multiple of 360 degrees. Duplicate the math here, preferring
    // the current axis for an identity rotation:
    float len = q.GetImaginary().GetLength();
    if (len > GF_MIN_VECTOR_LENGTH) {
        _axis  = q.GetImaginary() / len;
        _angle = 2.0 * GfRadiansToDegrees(acos(q.GetReal()));
    }
    else {
        // Leave the axis as is; just set the angle to 0.
        _angle = 0.0;
    }
*/
    return *this;
}

}// end namespace BOB
