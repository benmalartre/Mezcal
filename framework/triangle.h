//======================================================
// TRIANGLE DECLARATION
//======================================================


#ifndef _TRIANGLE_H_
#define _TRIANGLE_H_

#include <algorithm>
#include <math.h>
#include <stdio.h>
#include "common.h"
#include "Vector3.h"

namespace BOB{
    class Triangle{
    public:
        int ID;
        int mapID;
        int vertices[3];
    public:
        void getCenter(const float* positions, Vector3& center);
        void getNormal(const float* positions, Vector3& normal);
        void closestPoint( const float* positions, const Vector3 &point , Vector3& closest, float& u, float& v, float& w);
        bool touch(const float* positions, const Vector3& center, const Vector3& boxhalfsize);
        bool planeBoxTest(const Vector3& normal, const Vector3& vert, const Vector3& maxbox);
    };
	
#define FINDMINMAX(x0,x1,x2,min,max) \
min = max = x0;   \
if(x1<min) min=x1;\
if(x1>max) max=x1;\
if(x2<min) min=x2;\
if(x2>max) max=x2;


/*======================== X-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb)			   \
p0 = a*v0[1] - b*v0[2];			       	   \
p2 = a*v2[1] - b*v2[2];			       	   \
if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
rad = fa * boxhalfsize[1] + fb * boxhalfsize[2];   \
if(min>rad || max<-rad) return 0;



#define AXISTEST_X2(a, b, fa, fb)			   \
p0 = a*v0[1] - b*v0[2];			           \
p1 = a*v1[1] - b*v1[2];			       	   \
if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
rad = fa * boxhalfsize[1] + fb * boxhalfsize[2];   \
if(min>rad || max<-rad) return 0;

/*======================== Y-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb)			   \
p0 = -a*v0[0] + b*v0[2];		      	   \
p2 = -a*v2[0] + b*v2[2];	       	       	   \
if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
rad = fa * boxhalfsize[0] + fb * boxhalfsize[2];   \
if(min>rad || max<-rad) return 0;



#define AXISTEST_Y1(a, b, fa, fb)			   \
p0 = -a*v0[0] + b*v0[2];		      	   \
p1 = -a*v1[0] + b*v1[2];	     	       	   \
if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
rad = fa * boxhalfsize[0] + fb * boxhalfsize[2];   \
if(min>rad || max<-rad) return 0;

/*======================== Z-tests ========================*/
#define AXISTEST_Z12(a, b, fa, fb)			   \
p1 = a*v1[0] - b*v1[1];			           \
p2 = a*v2[0] - b*v2[1];			       	   \
if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} \
rad = fa * boxhalfsize[0] + fb * boxhalfsize[1];   \
if(min>rad || max<-rad) return 0;

#define AXISTEST_Z0(a, b, fa, fb)			   \
p0 = a*v0[0] - b*v0[1];				   \
p1 = a*v1[0] - b*v1[1];			           \
if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
rad = fa * boxhalfsize[0] + fb * boxhalfsize[1];   \
if(min>rad || max<-rad) return 0;

	
}// end napespace BOB
#endif /* _TRIANGLE_H_ */
