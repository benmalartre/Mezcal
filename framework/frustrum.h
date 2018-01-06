//--------------------------------------------------------
// FRUSTRUM
//--------------------------------------------------------
#ifndef _FRUSTUM_H_
#define _FRUSTUM_H_

#include <float.h>
#include "vector3.h"
#include "range1.h"
#include "range2.h"
#include "point.h"
#include "plane.h"
#include "rotation.h"
#include "tetrahedron.h"
#include "ray.h"

namespace BOB{
    

class Frustum{
public:
    enum ProjectionType {
        ORTHOGRAPHIC,
        PERSPECTIVE,
        UNDEFINED_PROJECTION_TYPE
    };

    Frustum();
    Frustum(const Vector3 &position,
            const Rotation &rotation,
            const Range2 &window,
            const Range1 &nearFar,
            Frustum::ProjectionType projectionType,
            float viewDistance = 5.0);
    
    Frustum(const Matrix4 &camToWorldXf,
            const Range2 &window,
            const Range1 &nearFar,
            Frustum::ProjectionType projectionType,
            float viewDistance = 5.0);

    
    // Equality operator. true iff all parts match.
    bool operator ==(const Frustum& f) const {
        if (_position       != f._position)        return false;
        if (_rotation       != f._rotation)        return false;
        if (_window         != f._window)          return false;
        if (_nearFar        != f._nearFar)         return false;
        if (_viewDistance   != f._viewDistance)    return false;
        if (_projectionType != f._projectionType)  return false;
        
        return true;
    }
    
    // Inequality operator. true iff not equality.
    bool operator !=(const Frustum& f) const {
        return !(*this == f);
    }
    
    /// Destructor.
    ~Frustum();
    
    /// \name Value setting and access
    /// The methods in this group set and access the values that are used to
    /// define a frustum.
    ///@{
    
    /// Sets the position of the frustum in world space.
    void                setPosition(const Vector3 &position) {
        _position = position;
        dirtyFrustumPlanes();
    }
    
    /// Returns the position of the frustum in world space.
    const Vector3 &     getPosition() const {
        return _position;
    }
    
    /// Sets the orientation of the frustum in world space as a rotation to
    /// apply to the default frame: looking along the -z axis with the +y axis
    /// as "up".
    void                setRotation(const Rotation &rotation) {
        _rotation = rotation;
        dirtyFrustumPlanes();
    }
    
    /// Returns the orientation of the frustum in world space as a rotation to
    /// apply to the -z axis.
    const Rotation &  getRotation() const {
        return _rotation;
    }
    
    /// Sets the position and rotation of the frustum from a camera matrix
    /// (always from a y-Up camera). The resulting frustum's transform will
    /// always represent a right-handed and orthonormal coordinate sytem
    /// (scale, shear, and projection are removed from the given \p
    /// camToWorldXf).
    void setPositionAndRotationFromMatrix(const Matrix4 &camToWorldXf);
    
    /// Sets the window rectangle in the reference plane that defines the
    /// left, right, top, and bottom planes of the frustum.
    void                setWindow(const Range2 &window)  {
        _window = window;
        dirtyFrustumPlanes();
    }
    
    /// Returns the window rectangle in the reference plane.
    const Range2 &   getWindow() const {
        return _window;
    }
    
    /// Returns the depth of the reference plane.
    static float getReferencePlaneDepth() {
        return 1.0;
    }
    
    /// Sets the near/far interval.
    void                setNearFar(const Range1 &nearFar) {
        _nearFar = nearFar;
        dirtyFrustumPlanes();
    }
    
    /// Returns the near/far interval.
    const Range1 &   getNearFar() const {
        return _nearFar;
    }
    
    /// Sets the view distance.
    void                setViewDistance(float viewDistance) {
        _viewDistance = viewDistance;
    }
    
    /// Returns the view distance.
    float              getViewDistance() const {
        return _viewDistance;
    }
    
    /// Sets the projection type.
    void        setProjectionType(Frustum::ProjectionType projectionType) {
        _projectionType = projectionType;
        dirtyFrustumPlanes();
    }
    
    /// Returns the projection type.
    Frustum::ProjectionType   getProjectionType() const {
        return _projectionType;
    }
    
    ///@}
    
    /// \name Convenience methods
    ///
    /// The methods in this group allow the frustum's data to be accessed and
    /// modified in terms of different representations that may be more
    /// convenient for certain applications.
    ///
    ///@{
    
    /// Sets up the frustum in a manner similar to \c gluPerspective().
    ///
    /// It sets the projection type to \c GfFrustum::PERSPECTIVE and sets the
    /// window specification so that the resulting symmetric frustum encloses
    /// an angle of \p fieldOfViewHeight degrees in the vertical direction,
    /// with \p aspectRatio used to figure the angle in the horizontal
    /// direction. The near and far distances are specified as well. The
    /// window coordinates are computed as:
    /// \code
    ///     top    = tan(fieldOfViewHeight / 2)
    ///     bottom = -top
    ///     right  = top * aspectRatio
    ///     left   = -right
    ///     near   = nearDistance
    ///     far    = farDistance
    /// \endcode
    ///
    void         setPerspective(float fieldOfViewHeight,
                                float aspectRatio,
                                float nearDistance,
                                float farDistance);
    
    /// Sets up the frustum in a manner similar to gluPerspective().
    ///
    /// It sets the projection type to \c GfFrustum::Perspective and
    /// sets the window specification so that:
    ///
    /// If \a isFovVertical is true, the resulting symmetric frustum encloses
    /// an angle of \p fieldOfView degrees in the vertical direction, with \p
    /// aspectRatio used to figure the angle in the horizontal direction.
    ///
    /// If \a isFovVertical is false, the resulting symmetric frustum encloses
    /// an angle of \p fieldOfView degrees in the horizontal direction, with
    /// \p aspectRatio used to figure the angle in the vertical direction.
    ///
    /// The near and far distances are specified as well. The window
    /// coordinates are computed as follows:
    ///
    /// \li if isFovVertical:
    ///     \li top    = tan(fieldOfView / 2)
    ///     \li right  = top * aspectRatio
    /// \li if NOT isFovVertical:
    ///     \li right    = tan(fieldOfView / 2)
    ///     \li top  = right / aspectRation
    /// \li bottom = -top
    /// \li left   = -right
    /// \li near   = nearDistance
    /// \li far    = farDistance
    ///
    void         setPerspective(float fieldOfView,
                                bool   isFovVertical,
                            	float aspectRatio,
                            	float nearDistance,
                                float farDistance);
    
    /// Returns the current frustum in the format used by \c SetPerspective().
    /// If the current frustum is not a perspective projection, this returns
    /// \c false and leaves the parameters untouched.
    bool         getPerspective(float *fieldOfViewHeight,
                                float *aspectRatio,
                                float *nearDistance,
                                float *farDistance) const;
    
    /// Returns the current frustum in the format used by \c SetPerspective().
    /// If the current frustum is not a perspective projection, this returns
    /// \c false and leaves the parameters untouched.
    bool         getPerspective(bool   isFovVertical,
                                float *fieldOfView,
                                float *aspectRatio,
                                float *nearDistance,
                                float *farDistance) const;
    
    /// Returns the horizontal or vertical fov of the frustum. The fov of the
    /// frustum is not necessarily the same value as displayed in the viewer.
    /// The displayed fov is a function of the focal length or FOV avar. The
    /// frustum's fov may be different due to things like lens breathing.
    ///
    /// If the frustum is not of type \c GfFrustum::Perspective, the returned
    /// FOV will be 0.0.
    ///
    /// \note The default value for \c isFovVertical is false so calling \c
    /// GetFOV without an argument will return the horizontal field of view
    /// which is compatible with menv2x's old GfFrustum::GetFOV routine.
    float       getFOV(bool isFovVertical = false);
    
    /// Sets up the frustum in a manner similar to \c glOrtho().
    ///
    /// Sets the projection to \c GfFrustum::Orthographic and sets the window
    /// and near/far specifications based on the given values.
    void         setOrthographic(float left, float right,
                                 float bottom, float top,
                                 float nearPlane, float farPlane);
    
    /// Returns the current frustum in the format used by \c
    /// SetOrthographic(). If the current frustum is not an orthographic
    /// projection, this returns \c false and leaves the parameters untouched.
    bool         getOrthographic(float *left, float *right,
                                 float *bottom, float *top,
                                 float *nearPlane, float *farPlane)
    const;
    
    /// Modifies the frustum to tightly enclose a sphere with the given center
    /// and radius, using the current view direction. The planes of the
    /// frustum are adjusted as necessary. The given amount of slack is added
    /// to the sphere's radius is used around the sphere to avoid boundary
    /// problems.
    void         fitToSphere(const Vector3 &center,
                             float radius,
                             float slack = 0.0);
    
    /// Transforms the frustum by the given matrix.
    ///
    /// The transformation matrix is applied as follows: the position and the
    /// direction vector are transformed with the given matrix. Then the
    /// length of the new direction vector is used to rescale the near and far
    /// plane and the view distance. Finally, the points that define the
    /// reference plane are transformed by the matrix. This method assures
    /// that the frustum will not be sheared or perspective-projected.
    ///
    /// \note Note that this definition means that the transformed frustum
    /// does not preserve scales very well. Do \em not use this function to
    /// transform a frustum that is to be used for precise operations such as
    /// intersection testing.
    Frustum&   transform(const Matrix4 &matrix);
    
    /// Returns the normalized world-space view direction vector, which is
    /// computed by rotating the -z axis by the frustum's rotation.
    Vector3      computeViewDirection() const;
    
    /// Returns the normalized world-space up vector, which is computed by
    /// rotating the y axis by the frustum's rotation.
    Vector3      computeUpVector() const;
    
    /// Computes the view frame defined by this frustum. The frame consists of
    /// the view direction, up vector and side vector, as shown in this
    /// diagram.
    ///
    /// \code
    ///            up
    ///            ^   ^
    ///            |  /
    ///            | / view
    ///            |/
    ///            +- - - - > side
    /// \endcode
    ///
    void         computeViewFrame(Vector3 *side,
                                  Vector3 *up,
                                  Vector3 *view) const;
    
    /// Computes and returns the world-space look-at point from the eye point
    /// (position), view direction (rotation), and view distance.
    Vector3      computeLookAtPoint() const;
    
    /// Returns a matrix that represents the viewing transformation for this
    /// frustum.  That is, it returns the matrix that converts points from
    /// world space to eye (frustum) space.
    Matrix4   computeViewMatrix() const;
    
    /// Returns a matrix that represents the inverse viewing transformation
    /// for this frustum.  That is, it returns the matrix that converts points
    /// from eye (frustum) space to world space.
    Matrix4   computeViewInverse() const;
    
    /// Returns a GL-style projection matrix corresponding to the frustum's
    /// projection.
    Matrix4   computeProjectionMatrix() const;
    
    /// Returns the aspect ratio of the frustum, defined as the width of the
    /// window divided by the height. If the height is zero or negative, this
    /// returns 0.
    float       computeAspectRatio() const;
    
    /// Returns the world-space corners of the frustum as a vector of 8
    /// points, ordered as:
    /// \li Left bottom near
    /// \li Right bottom near
    /// \li Left top near
    /// \li Right top near
    /// \li Left bottom far
    /// \li Right bottom far
    /// \li Left top far
    /// \li Right top far
    std::vector<Vector3> computeCorners() const;
    
    /// Returns the world-space corners of the intersection of the frustum
    /// with a plane parallel to the near/far plane at distance d from the
    /// apex, ordered as:
    /// \li Left bottom
    /// \li Right bottom
    /// \li Left top
    /// \li Right top
    /// In particular, it gives the partial result of ComputeCorners when given
    /// near or far distance.
    std::vector<Vector3> computeCornersAtDistance(float d) const;
    
    /// Returns a frustum that is a narrowed-down version of this frustum,
    /// such that the frustum rectangle on the near plane encloses \p point
    /// with at most \p halfSize[0] distance on the left and right and at most
    /// \p halfSize[1] distance on the top and bottom. (If \p point is closer
    /// than the half size to a side of the frustum, that side is left alone.
    /// The point and sizes are in normalized 2D coordinates; they range from
    /// (-1, -1) at the lower left corner of the near-plane window rectangle
    /// to (1,1) at the upper right corner.
    ///
    /// \p point is a 2d point expressed as a normalized window position.
    ///
    /// This method is useful for computing a volume to use for interactive
    /// picking.
    Frustum    computeNarrowedFrustum(const Vector2 &point,
                                      const Vector2 &halfSize) const;
    
    /// Returns a frustum that is a narrowed-down version of this frustum,
    /// such that the frustum rectangle on the near plane encloses \p point
    /// with at most \p halfSize[0] distance on the left and right and at most
    /// \p halfSize[1] distance on the top and bottom. (If \p point is closer
    /// than the half size to a side of the frustum, that side is left alone.
    /// The point and sizes are in normalized 2D coordinates; they range from
    /// (-1, -1) at the lower left corner of the near-plane window rectangle
    /// to (1,1) at the upper right corner.
    ///
    /// \p point is a 3d point expressed in world coordinates
    ///
    /// This method is useful for computing a volume to use for interactive
    /// picking.
    Frustum    computeNarrowedFrustum(const Vector3 &worldPoint,
                                      const Vector2 &halfSize) const;
    
    /// Builds and returns a \c GfRay that starts at the viewpoint and extends
    /// through the given \a windowPos given in normalized coords (-1 to +1 in
    /// both dimensions) window position.
    ///
    /// Contrasted with ComputePickRay(), this method returns a ray whose
    /// origin is the eyepoint, while that method returns a ray whose origin
    /// is on the near plane.
    Ray        computeRay(const Vector2 &windowPos) const;
    
    /// Builds and returns a \c GfRay that connects the viewpoint to the given
    /// 3d point in worldspace.
    ///
    /// Contrasted with ComputePickRay(), this method returns a ray whose
    /// origin is the eyepoint, while that method returns a ray whose origin
    /// is on the near plane.
    Ray        computeRay(const Vector3 &worldSpacePos) const;
    
    /// Builds and returns a \c GfRay that can be used for picking at the
    /// given normalized (-1 to +1 in both dimensions) window position.
    ///
    /// Contrasted with ComputeRay(), that method returns a ray whose origin
    /// is the eyepoint, while this method returns a ray whose origin is on
    /// the near plane.
    Ray        computePickRay(const Vector2 &windowPos) const;
    
    /// Builds and returns a \c GfRay that can be used for picking that
    /// connects the viewpoint to the given 3d point in worldspace.
    Ray       computePickRay(const Vector3 &worldSpacePos) const;

    /// \name Intersection methods
    
    /// Returns true if the given axis-aligned bbox is inside or intersecting
    /// the frustum. Otherwise, it returns false. Useful when doing picking or
    /// frustum culling.
    bool         intersects(const BBox3 &bbox) const;
    
    /// Returns true if the given point is inside or intersecting the frustum.
    /// Otherwise, it returns false.
    bool         intersects(const Vector3 &point) const;
    
    /// Returns \c true if the line segment formed by the given points is
    /// inside or intersecting the frustum.  Otherwise, it returns false.
    bool         intersects(const Vector3 &p0,
                            const Vector3 &p1) const;
    
    /// Returns \c true if the triangle formed by the given points is inside
    /// or intersecting the frustum.  Otherwise, it returns false.
    bool         intersects(const Vector3 &p0,
                            const Vector3 &p1,
                            const Vector3 &p2) const;
    
    /// Returns \c true if the bbox volume intersects the view volume given by
    /// the view-projection matrix, erring on the side of false positives for
    /// efficiency.
    ///
    /// This method is intended for cases where a GfFrustum is not available
    /// or when the view-projection matrix yields a view volume that is not
    /// expressable as a GfFrustum.
    ///
    /// Because it errs on the side of false positives, it is suitable for
    /// early-out tests such as draw or intersection culling.
    ///
    static bool  intersectsViewVolume(const BBox3 &bbox, const Matrix4 &vpMat);
private:
    void dirtyFrustumPlanes();
    void calculateFrustumPlanes() const;
    
    Ray computePickRayOffsetToNearPlane(const Vector3 &camSpaceFrom,
                                        const Vector3 &camSpaceDir) const;
    
    // Returns a frustum that is a narrowed-down version of this frustum, such
    // that the frustum rectangle on the near plane encloses \p point with at
    // most \p halfSize[0] distance on the left and right and at most \p
    // halfSize[1] distance on the top and bottom. (If \p point is closer than
    // the half size to a side of the frustum, that side is left alone. The
    // point and sizes are in normalized 2D coordinates; they range from (-1,
    // -1) at the lower left corner of the near-plane window rectangle to
    // (1,1) at the upper right corner.
    //
    // \p windowPoint is expressed in window coordinates
    //
    // This method is useful for computing a volume to use for interactive
    // picking.
    Frustum           computeNarrowedFrustumSub(const Vector2 windowPoint,
                                                   const Vector2 &halfSize) const;
    
    bool segmentIntersects(Vector3 const &p0, uint32_t p0Mask,
                        	Vector3 const &p1, uint32_t p1Mask) const;
    
    // Position of the frustum in world space.
    Vector3                     _position;
    
    // Orientation of the frustum in world space as a rotation to apply to the
    // -z axis.
    Rotation                  _rotation;
    
    // Window rectangle in the image plane.
    Range2                   _window;
    
    // Near/far interval.
    Range1                   _nearFar;
    
    // View distance.
    float                      _viewDistance;
    
    // Projection type.
    ProjectionType              _projectionType;
    
    // Cached planes.
    // If empty, the planes have not been calculated.
    mutable std::vector<Plane> _planes;

    

};

}// end namespace BOB
#endif /* _FRUSTUM_H_ */
