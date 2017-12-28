//--------------------------------------------------------
// Camera
//--------------------------------------------------------
#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "common.h"
#include "object.h"
#include <iostream>
namespace BOB{
enum CameraType{
	ORTHOGRAPHIC,
    PERSPECTIVE
};
class Camera: public Object
{
public:
    Camera();
    ~Camera();
    
    void lookAt();
    void lookAt(const Vector3& p, const Vector3& l, const Vector3& u);
    void updateProjection();
    void setDescription(float _fov, float _aspect, float _znear, float _zfar);
    
    void getOrthoMatrix();
    void getProjectionMatrix();
    void getViewMatrix();
    
    Matrix4& viewMatrix(){return view;};
    Matrix4& projectionMatrix(){return projection;};
    
    void setPosition(Vector3& p){pos=p;};
    void setLookAt(Vector3& l){lookat=l;};
    void setUp(Vector3& u){up=u;};
    
    void pan(float deltax, float deltay, float width, float height);
    void dolly(float deltax, float deltay, float width, float height);
    void orbit(float deltax, float deltay, float width, float height);
    
private:
    CameraType cameratype;
    float fov;
    float aspect;
    float nearplane;
    float farplane;
    float left;
    float right;
    float top;
    float bottom;
    
    Vector3 lookat;
    Vector3 up;
    Vector3 pos;
    
    float polar;
    float azimuth;
    
    Matrix4 view;
    Matrix4 projection;
};

}// end namespace BOB
#endif /* _CAMERA_H_ */
