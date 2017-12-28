//-------------------------------------------------------
// CAMERA IMPLEMENTATION
//-------------------------------------------------------
#include "camera.h"
#include "common.h"
#include "utils.h"

#include <iostream>
namespace BOB{
	Camera::Camera()
    {
        cameratype = PERSPECTIVE;
    };

	Camera::~Camera()
	{
	}

    //----------------------------------------------------------------------------
    // Look At
    //----------------------------------------------------------------------------
    void Camera::lookAt()
    {
        Vector3 dir = lookat - pos;
        Vector3 scl(1,1,1);
        
        Quaternion q = quaternionLookAt(dir,up);
        setLocalRotation(q);
        updateTransform();
        getViewMatrix();
    }

    void Camera::lookAt(const Vector3& p, const Vector3& l, const Vector3& u)
    {
        pos = p;
        lookat = l;
        up = u;
        lookAt();
    }
    
    //----------------------------------------------------------------------------
    // Pan
    //----------------------------------------------------------------------------
    void Camera::pan(float deltax, float deltay, float width, float height)
    {
        Vector3 delta, dist;
        dist = pos - lookat;
        float d = dist.length();
        
        delta.x = - deltax/(width*0.5f)*d;
        delta.y = deltay/(height*0.5f)*d;
        delta.z = 0.f;
        
        Quaternion q = view.getQuaternion();
        rotateVector(delta, q);
        
        pos += delta;
        lookat += delta;
        
        // update transform
        lookAt();
        
    }
    
    //----------------------------------------------------------------------------
    // Dolly
    //----------------------------------------------------------------------------
    void Camera::dolly(float deltax, float deltay, float width, float height)
    {
        float delta = deltay/height;
        pos = linearInterpolate(pos, lookat, delta);
        // update transform
        lookAt();
    }
    
    //----------------------------------------------------------------------------
    // Orbit
    //----------------------------------------------------------------------------
    void Camera::orbit(float deltax, float deltay, float width, float height)
    {
        Vector3 dist = pos - lookat;
        float d = dist.length();
        
        Vector3 r(0.0f,0.0f,d);
        Quaternion q;
        
        polar -= deltay;
        azimuth -= deltax;
        Vector3 xaxis(1.0f,0.0f,0.0f);
        q = quaternionFromAxisAngle(xaxis, polar * DEGREE_TO_RADIAN);
        rotateVector(r, q);
        
        Vector3 yaxis(0.0f,1.0f,0.0f);
        q = quaternionFromAxisAngle(yaxis, azimuth * DEGREE_TO_RADIAN);
        rotateVector(r, q);
        
        // flip upvector if necessary
        float p = fabs(fmodf(polar, 360.f));
        if(p<90.f || p>=270.f)
        {
            up.x = 0.0f;
            up.y = 1.0f;
            up.z = 0.0f;
        }
        else
        {
            up.x = 0.0f;
            up.y = -1.0f;
            up.z = 0.0f;
        }
        
        // update transform
        lookAt();
    }

    //----------------------------------------------------------------------------
    // Update Projection
    //----------------------------------------------------------------------------
    void Camera::updateProjection()
    {
        switch(cameratype)
        {
            case ORTHOGRAPHIC:
                getOrthoMatrix();
                getViewMatrix();
                break;
            case PERSPECTIVE:
                getProjectionMatrix();
                getViewMatrix();
                break;
        }
    }
    
    //----------------------------------------------------------------------------
    // Set Description
    //----------------------------------------------------------------------------
    void Camera::setDescription(float _fov, float _aspect, float _znear, float _zfar)
    {
        fov = _fov;
        aspect = _aspect;
        nearplane = _znear;
        farplane = _zfar;
    }
    
    //----------------------------------------------------------------------------
    // Get Perspective Projection Matrix
    //----------------------------------------------------------------------------
    void Camera::getProjectionMatrix()
    {
        float f = 1.f / tanf(DEGREE_TO_RADIAN*fov*0.5f);
        nearplane = MAXIMUM(nearplane,0.000001f);
        projection.identity();
        
        projection.v[0] = f/aspect;
        projection.v[5] = f;
        projection.v[10] = (farplane+nearplane)/(nearplane-farplane);
        projection.v[14] = (2.f*farplane*nearplane)/(nearplane-farplane);
        projection.v[11] = -1.f;
        projection.v[15] = 0.0f;
    }
    
    
    //----------------------------------------------------------------------------
    // Get Orthographic Projection Matrix
    //----------------------------------------------------------------------------
    void Camera::getOrthoMatrix()
    {
        projection.identity();
        projection.v[0] = 2.f/(right-left);
        projection.v[5] = 2.f/(top-bottom);
        projection.v[10] = -2.f/(farplane-nearplane);
        projection.v[12] = -(right+left)/(right-left);
        projection.v[13] = -(top+bottom)/(top-bottom);
        projection.v[14] = -(farplane+nearplane)/(farplane-nearplane);
    }
    
    //----------------------------------------------------------------------------
    // Get View Matrix
    //----------------------------------------------------------------------------
    void Camera::getViewMatrix()
    {
        // calculate orientation
        Vector3 dir = pos - lookat;
        dir.normalizeInPlace();
        Vector3 side = up ^ dir;
        side.normalizeInPlace();
        Vector3 up2 = dir ^ side;
        up2.normalizeInPlace();
        
        float d1, d2, d3;
        d1 = -(side.dot(pos));
        d2 = -(up2.dot(pos));
        d3 = -(dir.dot(pos));
        
        view.v[0]  = side.x; view.v[1]  = up2.x; view.v[2]  = dir.x;  view.v[3] = 0.0f;
        view.v[4]  = side.y; view.v[5]  = up2.y; view.v[6]  = dir.y;  view.v[7] = 0.0f;
        view.v[8]  = side.z; view.v[9]  = up2.z; view.v[10] = dir.z;  view.v[11] = 0.0f;
        view.v[12] = d1;     view.v[13] = d2;    view.v[14] = d3;     view.v[15] = 1.0f;
    }
        
}
