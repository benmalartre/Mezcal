//--------------------------------------------------------
// FRUSTRUM
//--------------------------------------------------------
#ifndef _FRUSTRUM_H_
#define _FRUSTRUM_H_

#include <float.h>
#include "vector3.h"
#include "range1.h"
#include "range2.h"
#include "point.h"
#include "plane.h"
#include "tetrahedron.h"

namespace BOB{
    

class Frustrum{
public:
    enum ProjectionType {
        ORTHOGRAPHIC,
        PERSPECTIVE,
        UNDEFINED_PROJECTION_TYPE
    };

    Frustrum();
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
#endif /* _GRID_H_ */
