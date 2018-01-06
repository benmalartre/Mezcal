//-------------------------------------------------------
// FRUSTRUM IMPLEMENTATION
//-------------------------------------------------------
#include "frustrum.h"
#include "matrix4.h"
#include "vector2.h"
#include "vector3.h"
#include "rotation.h"
#include "range1.h"
#include "range2.h"
#include "ray.h"

#include <algorithm>
namespace BOB{
    
    Frustum::Frustum() :
    _position(0.0f,0.0f,0.0f),
    _window(Vector2(-1.0, -1.0), Vector2(1.0, 1.0)),
    _nearFar(1.0, 10.0),
    _viewDistance(5.0),
    _projectionType(Frustum::PERSPECTIVE)
    {
        _rotation.setIdentity();
    }
    
    Frustum::Frustum(const Vector3 &position, const Rotation &rotation,
                    const Range2 &window, const Range1 &nearFar,
                    Frustum::ProjectionType projectionType,
                    float viewDistance) :
    _position(position),
    _rotation(rotation),
    _window(window),
    _nearFar(nearFar),
    _viewDistance(viewDistance),
    _projectionType(projectionType)
    {
    }
    
    Frustum::Frustum(const Matrix4 &camToWorldXf,
                    const Range2 &window, const Range1 &nearFar,
                    Frustum::ProjectionType projectionType,
                    float viewDistance) :
    _window(window),
    _nearFar(nearFar),
    _viewDistance(viewDistance),
    _projectionType(projectionType)
    {
        setPositionAndRotationFromMatrix(camToWorldXf);
    }
    
    Frustum::~Frustum()
    {
    }
    
    void Frustum::setPerspective(float fieldOfViewHeight, float aspectRatio,
                              	float nearDistance, float farDistance)
    {
        setPerspective(fieldOfViewHeight, true, aspectRatio,
                       nearDistance, farDistance);
    }
    
    void Frustum::setPerspective(float fieldOfView, bool isFovVertical,
                              	float aspectRatio,
                              	float nearDistance, float farDistance)
    {
        _projectionType = Frustum::PERSPECTIVE;
        
        float yDist = 1.0f;
        float xDist = 1.0f;
        
        // Check for 0, use 1 in that case
        if (aspectRatio == 0.0f) {
            aspectRatio = 1.0f;
        }
        
        if (isFovVertical) {
            // vertical is taken from the given field of view
            yDist = tanf((fieldOfView / 2.0)*DEGREE_TO_RADIAN) * getReferencePlaneDepth();
            // horizontal is determined by aspect ratio
            xDist = yDist * aspectRatio;
        } else {
            // horizontal is taken from the given field of view
            xDist = tanf((fieldOfView / 2.0)*DEGREE_TO_RADIAN) * getReferencePlaneDepth();
            // vertical is determined by aspect ratio
            yDist = xDist / aspectRatio;
        }
        
        _window.setMin(Vector2(-xDist, -yDist));
        _window.setMax(Vector2(xDist, yDist));
        _nearFar.setMin(nearDistance);
        _nearFar.setMax(farDistance);
        
        dirtyFrustumPlanes();
    }
    
    bool Frustum::getPerspective(float *fieldOfViewHeight, float *aspectRatio,
                              	float *nearDistance, float *farDistance) const
    {
        return getPerspective(true, fieldOfViewHeight, aspectRatio,
                              nearDistance, farDistance);
    }
    
    bool Frustum::getPerspective(bool isFovVertical,
                              	float *fieldOfView, float *aspectRatio,
                              	float *nearDistance, float *farDistance) const
    {
        if (_projectionType != Frustum::PERSPECTIVE)
            return false;
        
        Vector2 winSize = _window.getSize();
        
        if (isFovVertical) {
            *fieldOfView = 2.0 * (( atan(winSize[1] / 2.0)*RADIAN_TO_DEGREE)/ getReferencePlaneDepth() );
        } else {
            *fieldOfView = 2.0 * (( atan(winSize[0] / 2.0)*RADIAN_TO_DEGREE)/ GetReferencePlaneDepth() );
        }
        *aspectRatio       = winSize[0] / winSize[1];
        
        *nearDistance = _nearFar.GetMin();
        *farDistance  = _nearFar.GetMax();
        
        return true;
    }
    
    float Frustum::getFOV(bool isFovVertical)
    {
        float result = 0.0;
        
        if (getProjectionType() == Frustum::PERSPECTIVE) {
            float aspectRatio;
            float nearDistance;
            float farDistance;
            
            getPerspective(isFovVertical,
                           &result,
                           &aspectRatio,
                           &nearDistance,
                           &farDistance);
        }
        
        return result;
    }
    
    void Frustum::setOrthographic(float left, float right,
                               		float bottom, float top,
                               		float nearPlane, float farPlane)
    {
        _projectionType = Frustum::Orthographic;
        
        _window.setMin(Vector2(left, bottom));
        _window.setMax(Vector2(right, top));
        _nearFar.setMin(nearPlane);
        _nearFar.setMax(farPlane);
        
        dirtyFrustumPlanes();
    }
    
    bool Frustum::getOrthographic(float *left, float *right,
                               	float *bottom, float *top,
                               	float *nearPlane, float *farPlane) const
    {
        if (_projectionType != Frustum::Orthographic)
            return false;
        
        *left   = _window.GetMin()[0];
        *right  = _window.GetMax()[0];
        *bottom = _window.GetMin()[1];
        *top    = _window.GetMax()[1];
        
        *nearPlane	= _nearFar.GetMin();
        *farPlane   = _nearFar.GetMax();
        
        return true;
    }
    
    void Frustum::fitToSphere(const Vector3 &center, float radius, float slack)
    {
        //
        // The first part of this computes a good value for
        // _viewDistance and modifies the side (left, right, bottom,
        // and top) coordinates of the frustum as necessary.
        //
        
        if (_projectionType == Frustum::Orthographic) {
            // Set the distance so the viewpoint is outside the sphere.
            _viewDistance = radius + slack;
            // Set the camera window to enclose the sphere.
            _window = Range2(Vector2(-radius, -radius),
                                Vector2(radius, radius));
        }
        else {
            // Find the plane coordinate to use to compute the view.
            // Assuming symmetry, it should be the half-size of the
            // smaller of the two viewing angles. If asymmetric in a
            // dimension, use the larger size in that dimension.
            size_t whichDim = computeAspectRatio() > 1.0 ? 1 : 0;
            
            // XXX-doc
            float min = _window.getMin()[whichDim];
            float max = _window.getMax()[whichDim];
            float halfSize;
            if (min > 0.0) {
                halfSize = max;
            } else if (max < 0.0) {
                // CODE_COVERAGE_OFF_GCOV_BUG - seems to be hit but gcov disagrees
                halfSize = min;
                // CODE_COVERAGE_ON_GCOV_BUG
            } else if (-min > max) {
                halfSize = min;
            } else {
                halfSize = max;
            }
            
            if (halfSize < 0.0) {
                halfSize = -halfSize;
            } else if (halfSize == 0.0) {
                halfSize = 1.0;     // Why not?
            }
            
            // Determine the distance of the viewpoint from the center of
            // the sphere to make the frustum tangent to the sphere. Use
            // similar triangles: the right triangle formed by the
            // viewpoint and the half-size on the plane is similar to the
            // right triangle formed by the viewpoint and the radius of
            // the sphere at the point of tangency.
            _viewDistance = radius * (1.0/halfSize) * sqrtf(halfSize*halfSize + _nearFar.getMin()*_nearFar.getMin()));
            
            // XXX.
            // Hmmm. This is not really used anywhere but in tests, so
            // not gonna fix right now but it seems to me the above equation is
            // off.
            // In the diagram below, similar triangles yield the following
            // equal ratios:
            //    halfSize / referencePlaneDepth = radius / tanDist
            // So tanDist = (radius * referencePlaneDepth) / halfSize
            // Then, because it's a right triangle:
            // viewDistance = sqrt( GfSqr(radius) + GfSqr(tanDist))
            
            /*
             
             -----    |\                  /
             ^      |  \ ra            /
             |      |    \ di         /
             |      |      \ us      /
             |      |        \      /
             |      |          \   /
             |      |            \/      <---- make believe this is a right angle
             |      |------------/ ------
             |      |           /     ^
             |      |          /      |
             viewDistance |         /       |
             |      |        /        |
             |      |       /t        |
             |      |      /s        referencePlaneDepth
             |      |     /i          |
             |      |    /d           |
             |      |   /n            |
             |      |  /a             |
             |      | /t              v
             v      |/            ------
             ------
             |            |
             |<-halfSize->|
             |            |
             |            |
             */
        }
        
        // Adjust the camera so the near plane touches the sphere and the
        // far plane encloses the sphere.
        _nearFar.setMin(_viewDistance - (radius + slack));
        _nearFar.setMax(_nearFar.getMin() + 2.0 * (radius + slack));
        
        // Set the camera to use the new position. The view direction
        // should not have changed.
        _position = center - _viewDistance * computeViewDirection();
    }
    
    Frustum & Frustum::transform(const Matrix4 &matrix)
    {
        // We'll need the old parameters as we build up the new ones, so, work
        // on a newly instantiated frustum. We'll replace the contents of
        // this frustum with it once we are done. Note that _dirty is true
        // by default, so, there is no need to initialize it here.
        Frustum frustum;
        
        // Copy the projection type
        frustum._projectionType = _projectionType;
        
        // Transform the position of the frustum
        frustum._position = _position.transform(matrix);
        
        // Transform the rotation as follows:
        //   1. build view and direction vectors
        //   2. transform them with the given matrix
        //   3. normalize the vectors and cross them to build an orthonormal frame
        //   4. construct a rotation matrix
        //   5. extract the new rotation from the matrix
        
        // Generate view direction and up vector
        Vector3 viewDir = computeViewDirection();
        Vector3 upVec   = computeUpVector();
        
        // Transform by matrix
        Vector3 viewDirPrime = viewDir.transformDir(matrix);
        GfVec3d upVecPrime = upVec.transformDir(matrix);
        
        // Normalize. Save the vec size since it will be used to scale near/far.
        float scale = viewDirPrime.normalize();
        upVecPrime.normalizeInPlace();
        
        // Cross them to get the third axis. Voila. We have an orthonormal frame.
        Vector3 viewRightPrime = viewDirPrime.cross(upVecPrime);
        viewRightPrime.normalizeInPlace();
        
        // Construct a rotation matrix using the axes.
        //
        //  [ right     0 ]
        //  [ up        1 ]
        //  [ -viewDir  0 ]
        //  [ 0  0   0  1 ]
        Matrix4 rotMatrix;
        rotMatrix.identity();
        // first row
        rotMatrix[0] = viewRightPrime[0];
        rotMatrix[1] = viewRightPrime[1];
        rotMatrix[2] = viewRightPrime[2];
        
        // second row
        rotMatrix[3] = upVecPrime[0];
        rotMatrix[4] = upVecPrime[1];
        rotMatrix[5] = upVecPrime[2];
        
        // third row
        rotMatrix[6] = -viewDirPrime[0];
        rotMatrix[7] = -viewDirPrime[1];
        rotMatrix[8] = -viewDirPrime[2];
        
        // Extract rotation
        frustum._rotation = rotMatrix.extractRotation();
        
        // Since we applied the matrix to the direction vector, we can use
        // its length to find out the scaling that needs to applied to the
        // near and far plane.
        frustum._nearFar = _nearFar * scale;
        
        // Use the same length to scale the view distance
        frustum._viewDistance = _viewDistance * scale;
        
        // Transform the reference plane as follows:
        //
        //   - construct two 3D points that are on the reference plane
        //     (left/bottom and right/top corner of the reference window)
        //   - transform the points with the given matrix
        //   - move the window back to one unit from the viewpoint and
        //     extract the 2D coordinates that would form the new reference
        //     window
        //
        //     A note on how we do the last "move" of the reference window:
        //     Using similar triangles and the fact that the reference window
        //     is one unit away from the viewpoint, one can show that it's
        //     sufficient to divide the x and y components of the transformed
        //     corners by the length of the transformed direction vector.
        //
        //     A 2D diagram helps:
        //
        //                            |
        //                            |
        //               |            |
        //       * ------+------------+
        //      vp       |y1          |
        //                            |
        //       \--d1--/             |y2
        //
        //       \-------d2----------/
        //
        //     So, y1/y2 = d1/d2 ==> y1 = y2 * d1/d2
        //     Since d1 = 1 ==> y1 = y2 / d2
        //     The same argument applies to the x coordinate.
        //
        // NOTE: In an orthographic projection, the last step (division by
        // the length of the vector) is skipped.
        //
        // XXX NOTE2:  The above derivation relies on the
        // fact that GetReferecePlaneDepth() is 1.0.
        // If we ever allow this to NOT be 1, we'll need to fix this up.
        
        const Vector2 &min = _window.getMin();
        const Vector2 &max = _window.getMax();
        
        // Construct the corner points in 3D as follows: construct a starting
        // point by using the x and y coordinates of the reference plane and
        // -1 as the z coordinate. Add the position of the frustum to generate
        // the actual points in world-space coordinates.
        Vector3 leftBottom = _position + (GfVec3d(min[0], min[1], -1.0)).transformDir(_rotation);
        Vector3 rightTop = _position + (GfVec3d(max[0], max[1], -1.0)).transfomDir(_rotation);
        
        // Now, transform the corner points by the given matrix
        leftBottom = leftBottom.transform(matrix);
        rightTop   = rightTop.transform(matrix);
        
        // Subtract the transformed frustum position from the transformed
        // corner points. Then, rotate the points using the rotation that would
        // transform the view direction vector back to (0, 0, -1). This brings
        // the corner points from the woorld coordinate system into the local
        // frustum one.
        leftBottom -= frustum._position;
        rightTop   -= frustum._position;
        Matrix4 inv_rotation = frustum._rotation.inverse();
        leftBottom = leftBottom.transformDir(inv_rotation);
        rightTop = rightTop.transformDir(inv_rotation);
        //leftBottom = frustum._rotation.GetInverse().TransformDir(leftBottom);
        //rightTop   = frustum._rotation.GetInverse().TransformDir(rightTop);
        
        // Finally, use the similar triangles trick to bring the corner
        // points back at one unit away from the point. These scaled x and
        // y coordinates can be directly used to construct the new
        // transformed reference plane.  Skip the scaling step for an
        // orthographic projection, though.
        if (_projectionType == Frustum::PERSPECTIVE) {
            leftBottom /= scale;
            rightTop   /= scale;
        }
        
        frustum._window.setMin(Vector2(leftBottom[0], leftBottom[1]));
        frustum._window.setMax(Vector2(rightTop[0],   rightTop[1]));
        
        // Note that negative scales in the transform have the potential
        // to flip the window.  Fix it if necessary.
        Vector2 wMin = frustum._window.getMin();
        Vector2 wMax = frustum._window.getMax();
        // Make sure left < right
        if ( wMin[0] > wMax[0] ) {
            std::swap( wMin[0], wMax[0] );
        }
        // Make sure bottom < top
        if ( wMin[1] > wMax[1] ) {
            std::swap( wMin[1], wMax[1] );
        }
        frustum._window.setMin( wMin );
        frustum._window.setMax( wMax );
        
        *this = frustum;
        
        return *this;
    }
    
    Vector3 Frustum::computeViewDirection() const
    {
        return _rotation.transformDir(Vector3(0.0f,0.0f,-1.0f));
    }
    
    Vector3 Frustum::computeUpVector() const
    {
        return _rotation.transformDir(Vector3(0.0f,1.0f,0.0f));
    }
    
    void Frustum::computeViewFrame(Vector3 *side,
                                   Vector3 *up,
                                   Vector3 *view) const
    {
        *up   = computeUpVector();
        *view = computeViewDirection();
        *side = *view.cross(*up);
    }
    
    Vector3 Frustum::computeLookAtPoint() const
    {
        return _position + _viewDistance * computeViewDirection();
    }
    
    Matrix4 Frustum::computeViewMatrix() const
    {
        Matrix4 m;
        m.setLookAt(_position, _rotation);
        return m;
    }
    
    Matrix4
    Frustum::computeViewInverse() const
    {
        return computeViewMatrix().inverse();
    }
    
    Matrix4 Frustum::computeProjectionMatrix() const
    {
        // Build the projection matrix per Section 2.11 of
        // The OpenGL Specification: Coordinate Transforms.
        Matrix4 matrix;
        matrix.identity();
        
        const float l = _window.getMin()[0];
        const float r = _window.getMax()[0];
        const float b = _window.getMin()[1];
        const float t = _window.getMax()[1];
        const float n = _nearFar.getMin();
        const float f = _nearFar.getMax();
        
        const float rl = r - l;
        const float tb = t - b;
        const float fn = f - n;
        
        if (_projectionType == Frustum::Orthographic) {
            matrix[0] =  2.0f / rl;
            matrix[5] =  2.0f / tb;
            matrix[10] = -2.0f / fn;
            matrix[12] = -(r + l) / rl;
            matrix[13] = -(t + b) / tb;
            matrix[14] = -(f + n) / fn;
        }
        else {
            // Perspective:
            // The window coordinates are specified with respect to the
            // reference plane (near == 1).
            // XXX Note: If we ever allow reference plane depth to be other
            // than 1.0, we'll need to revisit this.
            matrix[0] = 2.0 / rl;
            matrix[5] = 2.0 / tb;
            matrix[10] = -(f + n) / fn;
            matrix[8] =  (r + l) / rl;
            matrix[9] =  (t + b) / tb;
            matrix[14] = -2.0 * n * f / fn;
            matrix[11] = -1.0;
            matrix[15] =  0.0;
        }
        
        return matrix;
    }
    
    float Frustum::computeAspectRatio() const
    {
        Vector2 winSize = _window.getSize();
        float aspectRatio = 0.0f;
        
        if (winSize[1] != 0.0f)
            // Negative winsize is used for envcubes, believe it or not.
            aspectRatio = fabsf(winSize[0] / winSize[1]);
        
        return aspectRatio;
    }
    
    vector<Vector3> Frustum::ComputeCorners() const
    {
        const Vector2 &winMin = _window.getMin();
        const Vector2 &winMax = _window.getMax();
        float near           = _nearFar.getMin();
        float far            = _nearFar.getMax();
        
        vector<Vector3> corners;
        corners.reserve(8);
        
        if (_projectionType == Frustum::PERSPECTIVE) {
            // Compute the eye-space corners of the near-plane and
            // far-plane frustum rectangles using similar triangles. The
            // reference plane in which the window rectangle is defined is
            // a distance of 1 from the eyepoint. By similar triangles,
            // just multiply the window points by near and far to get the
            // near and far rectangles.
            // XXX Note: If we ever allow reference plane depth to be other
            // than 1.0, we'll need to revisit this.
            corners.push_back(Vector3(near * winMin[0], near * winMin[1], -near));
            corners.push_back(Vector3(near * winMax[0], near * winMin[1], -near));
            corners.push_back(Vector3(near * winMin[0], near * winMax[1], -near));
            corners.push_back(Vector3(near * winMax[0], near * winMax[1], -near));
            corners.push_back(Vector3(far  * winMin[0], far  * winMin[1], -far));
            corners.push_back(Vector3(far  * winMax[0], far  * winMin[1], -far));
            corners.push_back(Vector3(far  * winMin[0], far  * winMax[1], -far));
            corners.push_back(Vector3(far  * winMax[0], far  * winMax[1], -far));
        }
        else {
            // Just use the reference plane rectangle as is, translated to
            // the near and far planes.
            corners.push_back(Vector3(winMin[0], winMin[1], -near));
            corners.push_back(Vector3(winMax[0], winMin[1], -near));
            corners.push_back(Vector3(winMin[0], winMax[1], -near));
            corners.push_back(Vector3(winMax[0], winMax[1], -near));
            corners.push_back(Vector3(winMin[0], winMin[1], -far));
            corners.push_back(Vector3(winMax[0], winMin[1], -far));
            corners.push_back(Vector3(winMin[0], winMax[1], -far));
            corners.push_back(Vector3(winMax[0], winMax[1], -far));
        }
        
        // Each corner is then transformed into world space by the inverse
        // of the view matrix.
        Matrix4 m = computeViewInverse();
        for (int i = 0; i < 8; i++)
            corners[i] = corners[i].transform(m);
        
        return corners;
    }
    
    vector<Vector3> Frustum::computeCornersAtDistance(float d) const
    {
        const Vector2 &winMin = _window.getMin();
        const Vector2 &winMax = _window.getMax();
        
        vector<Vector3> corners;
        corners.reserve(4);
        
        if (_projectionType == Frustum::PERSPECTIVE) {
            // Similar to ComputeCorners
            corners.push_back(Vector3(d * winMin[0], d * winMin[1], -d));
            corners.push_back(Vector3(d * winMax[0], d * winMin[1], -d));
            corners.push_back(Vector3(d * winMin[0], d * winMax[1], -d));
            corners.push_back(Vector3(d * winMax[0], d * winMax[1], -d));
        }
        else {
            corners.push_back(Vector3(winMin[0], winMin[1], -d));
            corners.push_back(Vector3(winMax[0], winMin[1], -d));
            corners.push_back(Vector3(winMin[0], winMax[1], -d));
            corners.push_back(Vector3(winMax[0], winMax[1], -d));
        }
        
        // Each corner is then transformed into world space by the inverse
        // of the view matrix.
        const Matrix4 m = computeViewInverse();
        for (int i = 0; i < 4; i++)
            corners[i] = corners[i].transform(m);
        
        return corners;
    }
    
    Frustum Frustum::computeNarrowedFrustum(const Vector2 &point,
                                      const Vector2 &halfSize) const
    {
        // Map the point from normalized space (-1 to 1) onto the frustum's
        // window. First, convert the point into the range from 0 to 1,
        // then interpolate in the window rectangle.
        Vector2 scaledPoint = .5 * (Vector2(1.0, 1.0) + point);
        Vector2 windowPoint = _window.getMin() + GfCompMult(scaledPoint,
                                                            _window.GetSize());
        
        return computeNarrowedFrustumSub(windowPoint, halfSize);
    }
    
    Frustum Frustum::computeNarrowedFrustum(const Vector3 &worldPoint,
                                      const Vector2 &halfSize) const
    {
        // Map the point from worldspace onto the frustum's window
        Matrix4 vm = computeViewMatrix();
        Vector3 lclPt = worldPoint.transform(vm);
        if (lclPt[2] >= 0.0f) {
            //TF_WARN("Given worldPoint is behind or at the eye");
            // Start with this frustum
            return *this;
        }
        float scaleFactor = _nearFar.getMin() / -lclPt[2];
        Vector2 windowPoint(lclPt[0] * scaleFactor, lclPt[1] * scaleFactor);
        
        return computeNarrowedFrustumSub(windowPoint, halfSize);
    }
    
    Frustum Frustum::computeNarrowedFrustumSub(const Vector2 windowPoint,
                                          const Vector2 &halfSize) const
    {
        // Start with this frustum
        Frustum narrowedFrustum = *this;
        
        // Also convert the sizes.
        Vector2 halfSizeOnRefPlane = .5 * GfCompMult(halfSize, _window.GetSize());
        
        // Shrink the narrowed frustum's window to surround the point.
        Vector2 min = windowPoint - halfSizeOnRefPlane;
        Vector2 max = windowPoint + halfSizeOnRefPlane;
        
        // Make sure the new bounds are within the old window.
        if (min[0] < _window.getMin()[0])
            min[0] = _window.getMin()[0];
        if (min[1] < _window.getMin()[1])
            min[1] = _window.getMin()[1];
        if (max[0] > _window.getMax()[0])
            max[0] = _window.getMax()[0];
        if (max[1] > _window.getMax()[1])
            max[1] = _window.getMax()[1];
        
        // Set the window to the result.
        narrowedFrustum.setWindow(Range2(min, max));
        
        return narrowedFrustum;
    }
    
    // Utility function for mapping an input value from
    // one range to another.
    static float
    rescale(float in,
             float inA, float inB,
             float outA, float outB )
    {
        float factor = (inA==inB) ? 0.0 : ((inA-in) / (inA-inB));
        return outA + ((outB-outA)*factor);
    }
    
    static Ray computeUntransformedRay(Frustum::ProjectionType projectionType,
                                          const Range2 &window,
                                          const Vector2 &windowPos)
    {
        // Compute position on window, from provided normalized
        // (-1 to 1) coordinates.
        float winX = rescale(windowPos[0], -1.0, 1.0,
                               window.getMin()[0], window.getMax()[0]);
        float winY = rescale(windowPos[1], -1.0, 1.0,
                               window.getMin()[1], window.getMax()[1]);
        
        // Compute the camera-space starting point (the viewpoint) and
        // direction (toward the point on the window).
        Vector3 pos;
        Vector3 dir;
        if (projectionType == Frustum::PERSPECTIVE) {
            pos = Vector3(0);
            dir = Vector3(winX, winY, -1.0).normalize();
        }
        else {
            pos.set(winX, winY, 0.0);
            dir = Vector3(0.0f,0.0f,-1.0f);
        }
        
        // Build and return the ray
        return Ray(pos, dir);
    }
    
    Ray Frustum::computeRay(const Vector2 &windowPos) const
    {
        Ray ray = computeUntransformedRay(_projectionType, _window, windowPos);
        
        // Transform these by the inverse of the view matrix.
        const Matrix4 &viewInverse = computeViewInverse();
        Vector3 rayFrom = ray.getOrigin().transform(viewInverse);
        Vector3 rayDir = ray.getDirection().transformDir(viewInverse);
        
        // Build and return the ray
        return Ray(rayFrom, rayDir);
    }
    
    Ray Frustum::computePickRay(const Vector2 &windowPos) const
    {
        Ray ray = computeUntransformedRay(_projectionType, _window, windowPos);
        return computePickRayOffsetToNearPlane(ray.getOrigin(), ray.getDirection());
    }
    
    Ray Frustum::computeRay(const Vector3 &worldSpacePos) const
    {
        Vector3 camSpaceToPos = worldSpacePos.transform(computeViewMatrix());
        
        // Compute the camera-space starting point (the viewpoint) and
        // direction (toward the point camSpaceToPos).
        Vector3 pos;
        Vector3 dir;
        if (_projectionType == PERSPECTIVE) {
            pos = Vector3(0);
            dir = camSpaceToPos.normalie();
        }
        else {
            pos.set(camSpaceToPos[0], camSpaceToPos[1], 0.0);
            dir = Vector3(0.0f,0.0f,-1.0f);
        }
        
        // Transform these by the inverse of the view matrix.
        const Matrix4 &viewInverse = computeViewInverse();
        Vector3 rayFrom = pos.transform(viewInverse);
        Vector3 rayDir = dir.transformDir(viewInverse);
        
        // Build and return the ray
        return Ray(rayFrom, rayDir);
    }
    
    Ray Frustum::computePickRay(const Vector3 &worldSpacePos) const
    {
        Vector3 camSpaceToPos = worldSpacePos.transform(computeViewMatrix());
        
        // Compute the camera-space starting point (the viewpoint) and
        // direction (toward the point camSpaceToPos).
        Vector3 pos;
        Vector3 dir;
        if (_projectionType == PERSPECTIVE) {
            pos = Vector3(0);
            dir = camSpaceToPos.normalize();
        }
        else {
            pos.set(camSpaceToPos[0], camSpaceToPos[1], 0.0);
            dir = Vector3(0.0f,0.0f,-1.0f);
        }
        
        return computePickRayOffsetToNearPlane(pos, dir);
    }
    
    Ray Frustum::computePickRayOffsetToNearPlane(const Vector3 &camSpaceFrom,
                                                const Vector3 &camSpaceDir) const
    {
        // Move the starting point to the near plane so we don't pick
        // anything that's clipped out of view.
        Vector3 rayFrom = camSpaceFrom + _nearFar.getMin() * camSpaceDir;
        
        // Transform these by the inverse of the view matrix.
        const Matrix4 &viewInverse = computeViewInverse();
        rayFrom = rayFrom.transform(viewInverse);
        Vector3 rayDir = camSpaceDir.transformDir(viewInverse);
        
        // Build and return the ray
        return Ray(rayFrom, rayDir);
    }
    
    bool Frustum::intersects(const BBox &bbox) const
    {
        
        // Recalculate frustum planes if necessary
        calculateFrustumPlanes();
        
        // Get the bbox in its local space and the matrix that converts
        // world space to that local space.
        const Range3  &localBBox    = bbox.getRange();
        const Matrix4 &worldToLocal = bbox.getInverseMatrix();
        
        // Test the bbox against each of the frustum planes, transforming
        // the plane by the inverse of the matrix to bring it into the
        // bbox's local space.
        for (size_t i = 0; i < _planes.size(); i++) {
            
            Plane localPlane = _planes[i];
            localPlane.transform(worldToLocal);
            
            if (! localPlane.intersectsPositiveHalfSpace(localBBox))
                return false;
        }
        
        return true;
    }
    
    bool Frustum::intersects(const Vector3 &point) const
    {
        // Recalculate frustum planes if necessary
        calculateFrustumPlanes();
        
        // Determine if the point is inside/intersecting the frustum. Quit early
        // if the point is outside of any of the frustum planes.
        for (size_t i = 0; i < _planes.size(); i++) {
            if (!_planes[i].intersectsPositiveHalfSpace(point)) {
                return false;
            }
        }
        
        return true;
    }
    
    inline static uint32_t
    calcIntersectionBitMask( const std::vector<Plane> & planes,
                             Vector3 const &p)
    {
        return
        (1 << 0) * planes[0].intersectsPositiveHalfSpace(p) +
        (1 << 1) * planes[1].intersectsPositiveHalfSpace(p) +
        (1 << 2) * planes[2].intersectsPositiveHalfSpace(p) +
        (1 << 3) * planes[3].intersectsPositiveHalfSpace(p) +
        (1 << 4) * planes[4].intersectsPositiveHalfSpace(p) +
        (1 << 5) * planes[5].intersectsPositiveHalfSpace(p);
    }
    
    bool Frustum::segmentIntersects(Vector3 const &p0, uint32_t p0Mask,
                                  Vector3 const &p1, uint32_t p1Mask) const
    {
        // If any of the 6 bits is 0 in both masks, then both points are
        // on the bad side of the corresponding plane. This means that
        // there can't be any intersection.
        if ((p0Mask | p1Mask) != 0x3F)
            return false;
        
        // If either of the masks has all 6 planes set, the point is
        // inside the frustum, so there's an intersection.
        if ((p0Mask == 0x3F) || (p1Mask == 0x3F))
            return true;
        
        // If we get here, the 2 points of the segment are both outside
        // the frustum, but not both on the outside of any single plane.
        
        // Now we can clip the segment against each plane that it
        // straddles to see if the resulting segment has any length.
        // Perform the clipping using parametric coordinates, where t=0
        // represents p0 and t=1 represents p1. Use v = the vector from p0
        // to p1.
        float t0 = 0.0;
        float t1 = 1.0;
        Vector3 v = p1 - p0;
        
        for (size_t i=0; i < _planes.size(); ++i) {
            const Plane & plane = _planes[i];
            const uint32_t planeBit = 1 << i;
            
            uint32_t p0Bit = p0Mask & planeBit;
            uint32_t p1Bit = p1Mask & planeBit;
            
            // Do this only if the points straddle the plane, meaning they
            // have different values for the bit.
            if (p0Bit == p1Bit)
                continue;
            
            // To find the parametric distance t at the intersection of a
            // plane and the line defined by (p0 + t * v):
            //
            //   Substitute the intersection point (p0 + t * v) into the
            //   plane equation to get   n . (p0 + t * v) - d = 0
            //
            //   Solve for t:  t = - (n . p0 - d) / (n . v)
            //      But (n . p0 - d) is the distance of p0 from the plane.
            float t = -plane.getDistance(p0) / (plane.getNormal() * v);
            
            // If p0 is inside and p1 is outside, replace t1. Otherwise,
            // replace t0.
            if (p0Bit) {
                if (t < t1)
                    t1 = t;
            } else {
                if (t > t0)
                    t0 = t;
            }
            
            // If there is no line segment left, there's no intersection.
            if (t0 > t1)
                return false;
        }
        
        // If we get here, there's an intersection
        return true;
    }
    
    bool Frustum::intersects(const Vector3 &p0, const Vector3 &p1) const
    {
        // Recalculate frustum planes if necessary
        calculateFrustumPlanes();
        
        // Compute the intersection masks for each point. There is one bit
        // in each mask for each of the 6 planes.
        return segmentIntersects(p0, calcIntersectionBitMask(_planes, p0),
                                  p1, calcIntersectionBitMask(_planes, p1));
        
    }
    
    bool Frustum::intersects(const Vector3 &p0,
                          const Vector3 &p1,
                          const Vector3 &p2) const
    {
        // Recalculate frustum planes if necessary
        calculateFrustumPlanes();
        
        // Compute the intersection masks for each point. There is one bit
        // in each mask for each of the 6 planes.
        uint32_t p0Mask = calcIntersectionBitMask(_planes, p0);
        uint32_t p1Mask = calcIntersectionBitMask(_planes, p1);
        uint32_t p2Mask = calcIntersectionBitMask(_planes, p2);
        
        // If any of the 6 bits is 0 in all masks, then all 3 points are
        // on the bad side of the corresponding plane. This means that
        // there can't be any intersection.
        if ((p0Mask | p1Mask | p2Mask) != 0x3F)
            return false;
        
        // If any of the masks has all 6 planes set, the point is inside
        // the frustum, so there's an intersection.
        if ((p0Mask == 0x3F) || (p1Mask == 0x3F) || (p2Mask == 0x3F))
            return true;
        
        // If we get here, the 3 points of the triangle are all outside
        // the frustum, but not all on the outside of any single plane.
        // There are now 3 remaining possibilities:
        //
        //  (1) At least one edge of the triangle intersects the frustum.
        //  (2) The triangle completely encloses the frustum.
        //  (3) Neither of the above is true, so there is no intersection.
        
        // Test case (1) by intersecting all three edges with the
        // frustum.
        if (segmentIntersects(p0, p0Mask, p1, p1Mask) ||
            segmentIntersects(p1, p1Mask, p2, p2Mask) ||
            segmentIntersects(p2, p2Mask, p0, p0Mask))
            return true;
        
        
        // That leaves cases (2) and (3).
        
        // Test for case 2 by computing rays from the viewpoint
        // to the far corners, and doing a ray-triangle
        // intersection test.
        // If all 3 points of the triangle lie between the near/far planes,
        // then we only need to test intersection of 1 corner's ray.
        // Otherwise, we test all 4 corners and if any hit, the frustum is inside
        // the triangle.  If all miss, then the frustum is outside.
        // If the points don't lie between near/far, then  we have to test all
        // 4 corners to catch the case when the triangle is being partially
        // clipped by the near/far plane.
        size_t numCornersToCheck = 4;
        // XXX Note: 4 & 5 below are highly dependent on
        // _CalculateFrustumPlanes implementation
        uint32_t nearBit = (1 << 4);
        uint32_t farBit  = (1 << 5);
        if ( (p0Mask & nearBit) && (p1Mask & nearBit) && (p2Mask & nearBit) &&
            (p0Mask & farBit)  && (p1Mask & farBit)  && (p2Mask & farBit) ) {
            numCornersToCheck = 1;
        }
        
        for (size_t i=0; i<numCornersToCheck; ++i) {
            Vector2 pickPoint = (i==0) ? Vector2(-1.0, -1.0) :
            (i==1) ? Vector2(-1.0,  1.0) :
            (i==2) ? Vector2( 1.0,  1.0) :
            Vector2( 1.0, -1.0);
            Ray pickRay = computePickRay(pickPoint);
            float distance;
            if ( pickRay.intersect(p0, p1, p2, &distance, NULL, NULL) ) {
                return true;
            }
        }
        
        
        // Must be case 3.
        return false;
    }
    
    void Frustum::dirtyFrustumPlanes()
    {
        _planes.clear();
    }
    
    void Frustum::calculateFrustumPlanes() const
    {
        if (!_planes.empty())
            return;
        
        _planes.reserve(6);
        
        // These are values we need to construct the planes.
        const Vector2 &winMin = _window.getMin();
        const Vector2 &winMax = _window.getMax();
        float near           = _nearFar.getMin();
        float far            = _nearFar.getMax();
        Matrix4 m          = computeViewInverse();
        
        // For a perspective frustum, we use the viewpoint and four
        // corners of the near-plane frustum rectangle to define the 4
        // planes forming the left, right, top, and bottom sides of the
        // frustum.
        if (_projectionType == Frustum::PERSPECTIVE) {
            
            //
            // Get the eye-space viewpoint (the origin) and the four corners
            // of the near-plane frustum rectangle using similar triangles.
            //
            // This picture may help:
            //
            //                  top of near plane
            //                  frustum rectangle
            //
            //                  + --
            //                / |  |
            //              /   |  |
            //            /     |  | h
            //          /       |  |
            //        /         |  |
            //   vp +-----------+ --
            //                    center of near plane frustum rectangle
            //      |___________|
            //           near
            //
            // The height (h) of this triangle is found by the following
            // equation, based on the definition of the _window member
            // variable, which is the size of the image rectangle in the
            // reference plane (a distance of 1 from the viewpoint):
            //
            //      h       _window.GetMax()[1]
            //    ------ = --------------------
            //     near             1
            //
            // Solving for h gets the height of the triangle. Doing the
            // similar math for the other 3 sizes of the near-plane
            // rectangle is left as an exercise for the reader.
            //
            // XXX Note: If we ever allow reference plane depth to be other
            // than 1.0, we'll need to revisit this.
            
            Vector3 vp(0.0, 0.0, 0.0);
            Vector3 lb(near * winMin[0], near * winMin[1], -near);
            Vector3 rb(near * winMax[0], near * winMin[1], -near);
            Vector3 lt(near * winMin[0], near * winMax[1], -near);
            Vector3 rt(near * winMax[0], near * winMax[1], -near);
            
            // Transform all 5 points into world space by the inverse of the
            // view matrix (which converts from world space to eye space).
            vp = vp.transform(m);
            lb = lb.transform(m);
            rb = rb.transform(m);
            lt = lt.transform(m);
            rt = rt.transform(m);
            
            // Construct the 6 planes. The three points defining each plane
            // should obey the right-hand-rule; they should be in counter-clockwise
            // order on the inside of the frustum. This makes the intersection of
            // the half-spaces defined by the planes the contents of the frustum.
            _planes.push_back( Plane(vp, lb, lt) );     // Left
            _planes.push_back( Plane(vp, rt, rb) );     // Right
            _planes.push_back( Plane(vp, rb, lb) );     // Bottom
            _planes.push_back( Plane(vp, lt, rt) );     // Top
            _planes.push_back( Plane(rb, lb, lt) );     // Near
        }
        
        // For an orthographic projection, we need only the four corners
        // of the near-plane frustum rectangle and the view direction to
        // define the 4 planes forming the left, right, top, and bottom
        // sides of the frustum.
        else {
            
            //
            // The math here is much easier than in the perspective case,
            // because we have parallel lines instead of triangles. Just
            // use the size of the image rectangle in the reference plane,
            // which is the same in the near plane.
            //
            Vector3 lb(winMin[0], winMin[1], -near);
            Vector3 rb(winMax[0], winMin[1], -near);
            Vector3 lt(winMin[0], winMax[1], -near);
            Vector3 rt(winMax[0], winMax[1], -near);
            
            // Transform the 4 points into world space by the inverse of
            // the view matrix (which converts from world space to eye
            // space).
            lb = lb.transform(m);
            rb = rb.transform(m);
            lt = lt.transform(m);
            rt = rt.transform(m);
            
            // Transform the canonical view direction (-z axis) into world
            // space.
            Vector3 dir = Vector3(0.0f,0.0f,-1.0f).transfomDir(m);
            
            // Construct the 5 planes from these 4 points and the
            // eye-space view direction.
            _planes.push_back( Plane(lt + dir, lt, lb) );       // Left
            _planes.push_back( Plane(rb + dir, rb, rt) );       // Right
            _planes.push_back( Plane(lb + dir, lb, rb) );       // Bottom
            _planes.push_back( Plane(rt + dir, rt, lt) );       // Top
            _planes.push_back( Plane(rb, lb, lt) );             // Near
        }
        
        // The far plane is the opposite to the near plane. To compute the 
        // distance from the origin for the far plane, we take the distance 
        // for the near plane, add the difference between the far and the near 
        // and then negate that. We do the negation since the far plane
        // faces the opposite direction. A small drawing would help:
        //
        //                               far - near
        //                     /---------------------------\ *
        //
        //        |           |                             |
        //        |           |                             |
        //        |           |                             |
        //   <----|---->      |                             |
        // fnormal|nnormal    |                             |
        //        |           |                             |
        //                near plane                     far plane
        //
        //         \---------/ *
        //          ndistance
        //         
        //         \---------------------------------------/ *
        //                         fdistance
        //
        // So, fdistance = - (ndistance + (far - near))
        _planes.push_back(
                          Plane(-_planes[4].getNormal(),
                                  -(_planes[4].getDistanceFromOrigin() + (far - near))) );
    }
    
    bool Frustum::intersectsViewVolume(const BBox3 &bbox,
                                    const Matrix4 &viewProjMat)
    {
        // This implementation is a standard technique employed in frustum
        // culling during rendering.  It correctly culls the box even from
        // view volumes that are not representable by a GfFrustum because of
        // skewed near/far planes, such as the ones produced by
        // presto shadowmap cameras.
        //
        // Its principle of operation:  If all 8 points of
        // the box, when transformed into clip coordinates,
        // are on one side or the other of each dimension's
        // clipping interval, then the entire
        // box volume must lie outside the view volume.
        
        // Compute the 8 points of the bbox in
        // bbox local space.
        Vector4 points[8];
        const Vector3 &localMin = bbox.getRange().getMin();
        const Vector3 &localMax = bbox.getRange().getMax();
        points[0] = Vector4(localMin[0], localMin[1], localMin[2], 1);
        points[1] = Vector4(localMin[0], localMin[1], localMax[2], 1);
        points[2] = Vector4(localMin[0], localMax[1], localMin[2], 1);
        points[3] = Vector4(localMin[0], localMax[1], localMax[2], 1);
        points[4] = Vector4(localMax[0], localMin[1], localMin[2], 1);
        points[5] = Vector4(localMax[0], localMin[1], localMax[2], 1);
        points[6] = Vector4(localMax[0], localMax[1], localMin[2], 1);
        points[7] = Vector4(localMax[0], localMax[1], localMax[2], 1);
        
        // Transform bbox local space points into clip space
        for (int i = 0; i < 8; ++i) {
            points[i] = points[i] * bbox.getMatrix() * viewProjMat;
        }
        
        // clipFlags is a 6-bit field with one bit per +/- per x,y,z,
        // or one per frustum plane.  If the points overlap the
        // clip volume in any axis, then clipFlags will be 0x3f (0b111111).
        int clipFlags = 0;
        for (int i = 0; i < 8; ++i) {
            Vector4 clipPos = points[i];
            
            // flag is used as a 6-bit shift register, as we append
            // results of plane-side testing.  OR-ing all the flags
            // combines all the records of what plane-side the points
            // have been on.
            int flag = 0;
            for (int j = 0; j < 3; ++j) {
                // We use +/-clipPos[3] as the interval bound instead of 
                // 1,-1 because these coordinates are not normalized.
                flag = (flag << 1) | (clipPos[j] <  clipPos[3]);
                flag = (flag << 1) | (clipPos[j] > -clipPos[3]);
            }
            clipFlags |= flag;
        }
        
        return clipFlags == 0x3f;
    }
    
    void Frustum::setPositionAndRotationFromMatrix(
                                                const Matrix4 &camToWorldXf)
    {
        // First conform matrix to be...
        Matrix4 conformedXf = camToWorldXf;
        // ... right handed
        if (!conformedXf.isRightHanded()) {
            static Matrix4 flip(Vector4(-1.0, 1.0, 1.0, 1.0));
            conformedXf = flip * conformedXf;
        }
        
        // ... and orthonormal
        conformedXf.orthonormalize();
        
        setRotation(conformedXf.extractRotation());
        setPosition(conformedXf.extractTranslation());
    }

}// end namespace BOB

