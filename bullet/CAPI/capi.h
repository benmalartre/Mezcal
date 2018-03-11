/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
	Draft high-level generic physics C-API. For low-level access, use the physics SDK native API's.
	Work in progress, functionality will be added on demand.

	If possible, use the richer Bullet C++ API, by including "btBulletDynamicsCommon.h"
*/
#ifndef BULLET_C_API_H
#define BULLET_C_API_H

//#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"

#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"	
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btConvexHull.h"
#include "LinearMath/btConvexHullComputer.h"

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"

#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodySolvers.h"
#include "BulletSoftBody/btDefaultSoftBodySolver.h"


#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"//picking
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"//picking

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"

#include "LinearMath/btQuickprof.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btSerializer.h"
#include "LinearMath/btIDebugDraw.h"


#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"


#define CUBE_HALF_EXTENTS 1.5
#define EXTRA_HEIGHT -10.f

const int maxProxies = 32766;
const int maxOverlap = 65535;

const float TRIANGLE_SIZE=8.f;
static float waveheight = 5.f;

#define BT_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

#ifdef BT_USE_DOUBLE_PRECISION
typedef double	btReal;
#else
typedef float	btReal;
#endif

typedef btReal BTVector3[3];
typedef btReal BTQuaternion[4];
typedef btReal BTMatrix4[16];

struct BTXForm{
	BTVector3 pos;
	BTQuaternion rot;
	BTVector3 scl;
};

struct btConvexHullDescription{
	int num_points;
	int num_planes;
	int num_edges;
	btReal* vertices;
	int* indices;
	
};

struct	btPhysicsSdk
{
	btVector3	m_worldAabbMin;
	btVector3 	m_worldAabbMax;
	btSoftBodyWorldInfo m_softBodyWorldInfo;

  	btBroadphaseInterface * m_broadphase;
	btCollisionDispatcher * m_dispatcher;
	btConstraintSolver * m_solver;
	btSoftBodySolver * m_softBodySolver;
	btCollisionAlgorithmCreateFunc*	m_boxBoxCF;
	btDefaultCollisionConfiguration* m_collisionConfiguration;

	btDynamicsWorld * m_world;
	btDynamicsWorld * m_pick;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>		m_collisionShapes;

	//todo: version, hardware/optimization settings etc?
	btPhysicsSdk()
		:m_worldAabbMin(-1000,-1000,-1000),
		m_worldAabbMax(1000,1000,1000),
		m_broadphase(NULL),
		m_dispatcher(NULL),
		m_solver(NULL),
		m_collisionConfiguration(NULL),
		m_boxBoxCF(NULL),
		m_world(NULL),
		m_pick(NULL)
	{

	}

};

#ifdef __cplusplus
extern "C" { 
#endif

/**	Particular physics SDK (C-API) */
	BT_DECLARE_HANDLE(btPhysicsSdkHandle);

/** 	Dynamics world, belonging to some physics SDK (C-API)*/
	BT_DECLARE_HANDLE(btDynamicsWorldHandle);

/** Rigid Body that can be part of a Dynamics World (C-API)*/	
	BT_DECLARE_HANDLE(btRigidBodyHandle);

/** Soft Body that can be part of a Dynamics World (C-API)*/	
	BT_DECLARE_HANDLE(btSoftBodyHandle);

/** 	Collision Shape/Geometry, property of a Rigid Body (C-API)*/
	BT_DECLARE_HANDLE(btCollisionShapeHandle);

/** Constraint for Rigid Bodies (C-API)*/
	BT_DECLARE_HANDLE(btConstraintHandle);
	BT_DECLARE_HANDLE(btConstraintSolverHandle);

/** Triangle Mesh interface (C-API)*/
	BT_DECLARE_HANDLE(btMeshInterfaceHandle);

/** Broadphase Scene/Proxy Handles (C-API)*/
	BT_DECLARE_HANDLE(btCollisionBroadphaseHandle);
	BT_DECLARE_HANDLE(btBroadphaseProxyHandle);
	BT_DECLARE_HANDLE(btCollisionWorldHandle);

	typedef void * C3DObjectPointer ;

	btDynamicsWorldHandle			BTTestWorld(bool useMCLPSolver);
	void							BTTestXForm(BTXForm& T);
	btDynamicsWorldHandle			BTSoftBodyWorld();
	int								BTCheckSoftBodyWorldInfo(btPhysicsSdkHandle sdkHandle);
/** Create and Delete a Physics SDK	*/
	btPhysicsSdkHandle				BTCreateDynamicsSdk(void); //this could be also another sdk, like ODE, PhysX etc.
	void							BTDeleteDynamicsSdk(btPhysicsSdkHandle	physicsSdk);
	btDynamicsWorldHandle			BTGetDynamicsWorld(btPhysicsSdkHandle physicsSdk);

/** Collision World, not strictly necessary, you can also just create a Dynamics World with Rigid Bodies which internally manages the Collision World with Collision Objects */

	typedef void(*btBroadphaseCallback)(void* clientData, void* object1,void* object2);
	btCollisionBroadphaseHandle		BTCreateSapBroadphase(btBroadphaseCallback beginCallback,btBroadphaseCallback endCallback);
	void							BTDestroyBroadphase(btCollisionBroadphaseHandle bp);
	btBroadphaseProxyHandle			BTCreateProxy(btCollisionBroadphaseHandle bp, void* clientData, btReal minX,btReal minY,btReal minZ, btReal maxX,btReal maxY, btReal maxZ);
	void							BTDestroyProxy(btCollisionBroadphaseHandle bp, btBroadphaseProxyHandle proxyHandle);
	void							BTSetBoundingBox(btBroadphaseProxyHandle proxyHandle, btReal minX,btReal minY,btReal minZ, btReal maxX,btReal maxY, btReal maxZ);

/* todo: add pair cache support with queries like add/remove/find pair */
	

/* todo: add/remove objects */
	

/* Dynamics World */
	btDynamicsWorldHandle			BTCreateDynamicsWorld(btPhysicsSdkHandle physicsSdkHandle,bool useMCLPSolver=false);
	btDynamicsWorldHandle			BTCreateSoftRigidDynamicsWorld(btPhysicsSdkHandle physicsSdk);
	void							BTDeleteDynamicsWorld(btDynamicsWorldHandle world);
	void							BTDeleteSoftRigidDynamicsWorld(btDynamicsWorldHandle world);
	int								BTStepSimulation(btDynamicsWorldHandle,	btReal	timeStep);
	void							BTAddRigidBody(btDynamicsWorldHandle world, btRigidBodyHandle object);
	void							BTRemoveRigidBody(btDynamicsWorldHandle world, btRigidBodyHandle object);
	void							BTAddSoftBody(btDynamicsWorldHandle world, btSoftBodyHandle object);
	void							BTRemoveSoftBody(btDynamicsWorldHandle world, btSoftBodyHandle object);
	int								BTGetNumCollideObjects(btDynamicsWorldHandle world);
	void							BTSetGravity(btDynamicsWorldHandle world, BTVector3 gravity);
	void							BTSetCollisionProcessedCallback( ContactProcessedCallback fn );
	void							BTGetWorldBoundingBox(btPhysicsSdkHandle sdk,btVector3& bb_min,btVector3& bb_max);
	
	//btSoftBodyWorldInfoHandle		BTGetSoftBodyWorldInfo(btPhysicsSdkHandle physicsSdk);

	int								BTCheckSoftBodySolver(btPhysicsSdkHandle sdkHandle);
	void							BTResetSparseSDF(btPhysicsSdkHandle sdkHandle);
/* Rigid Body  */
	btRigidBodyHandle				BTCreateRigidBody(	void* user_data,  float mass, btCollisionShapeHandle cshape );
	void							BTDeleteRigidBody(btRigidBodyHandle body);
	btRigidBodyHandle				BTGetRigidBodyByID(btDynamicsWorldHandle world, int id);
	C3DObjectPointer				BTGetUserData(btRigidBodyHandle body);
	void							BTSetActivationState(btRigidBodyHandle cbody, int state);
	void							BTRigidBodySetCollisionFlags(btRigidBodyHandle cbody, int flags);
	int								BTRigidBodyGetCollisionFlags(btRigidBodyHandle cbody);
	void							BTSetLinearFactor(btRigidBodyHandle cbody,BTVector3& factor);
	void							BTSetAngularFactorF(btRigidBodyHandle cbody,float factor);
	void							BTSetAngularFactor(btRigidBodyHandle cbody,BTVector3& factor);
	void							BTSetLinearVelocity(btRigidBodyHandle cbody,BTVector3& velocity);
	void							BTSetAngularVelocity(btRigidBodyHandle cbody,BTVector3& velocity);

/* Soft Body */
	btSoftBodyHandle				BTCreateSoftBodyFromConvexHull(void* user_data,btPhysicsSdkHandle hsdk, btReal* vertices,int* indices, int nb_triangles);
	btSoftBodyHandle				BTCreateSoftBodyFromTriMesh(void* user_data,btPhysicsSdkHandle hsdk, btReal* vertices,int* indices, int nb_triangles);
	btSoftBodyHandle				BTCreateClusterSoftBodyFromTriMesh(void* user_data,btPhysicsSdkHandle hsdk, btReal* vertices,int* indices, int nb_triangles,int nb_clusters);
	btSoftBodyHandle				BTGetSoftBodyByID(btDynamicsWorldHandle world, int id);
	int 							BTUpdatePointPosition(btSoftBodyHandle sbdh,btReal* vertices);
	btSoftBodyHandle				BTSoftBox(btPhysicsSdkHandle sdkHandle,BTVector3 ip,BTVector3 is);
	int								BTGetNumSoftBodies(btDynamicsWorldHandle hWorld);
	btSoftBodyHandle				BTSoftBoulder(btPhysicsSdkHandle sdkHandle,BTVector3 p,BTVector3 s,int np,int id);
	void							BTUpdateSoftBodyGeometry(btSoftBodyHandle hBody,btReal* vertices);
	int								BTGetSoftBodyNbVertices(btSoftBodyHandle hBody);
	int								BTGetSoftBodyNbFaces(btSoftBodyHandle hBody);
	int								BTGetSoftBodyNbNodes(btSoftBodyHandle hBody);
	//btSoftBodyHandle 			BTCreateSoftBody(	void* user_data, float mass, btCollisionShapeHandle cshape);
	//void						BTDeleteSoftBody

/* Collision Shape definition */
	btCollisionShapeHandle			BTNewGroundPlaneShape(BTVector3 pos, btReal size);
	btCollisionShapeHandle			BTNewSphereShape(btReal radius);
	btCollisionShapeHandle			BTNewBoxShape(btReal x, btReal y, btReal z);
	btCollisionShapeHandle			BTNewCapsuleShape(btReal radius, btReal height);	
	btCollisionShapeHandle			BTNewConeShape(btReal radius, btReal height);
	btCollisionShapeHandle			BTNewCylinderShape(btReal radius, btReal height);
	btCollisionShapeHandle			BTNewCompoundShape(void);
	void							BTAddChildShape(btCollisionShapeHandle compoundShape,btCollisionShapeHandle childShape, BTVector3 childPos,BTQuaternion childOrn);
	btCollisionShapeHandle			BTNewGImpactShape(int num_tri,int* indices,int num_vertices,btReal* vertices); // not working properly
	btCollisionShapeHandle			BTNewBvhTriangleMeshShape(int num_tri,int* indices,int num_vertices,btReal* vertices); // not working properly
	void							BTDeleteShape(btCollisionShapeHandle shape);
	void							BTSetCollisionMargin(btRigidBodyHandle hBody,btReal margin);
	btReal							BTGetCollisionMargin(btRigidBodyHandle hBody);
	void							BTSetFriction(btRigidBodyHandle hBody,btReal friction);

/* Convex Meshes */
	btCollisionShapeHandle			BTNewEmptyConvexHullShape(void);
	btCollisionShapeHandle			BTNewConvexHullShape(int num_tri,int* indices,int num_vertices,btReal* vertices);
	void							BTAddVertex(btCollisionShapeHandle convexHull, btReal x,btReal y,btReal z);
	void							BTGetConvexHullShapeDescription(btCollisionShapeHandle handle,btConvexHullDescription* desc);
	void							BTFillConvexHullShapeDescription(btCollisionShapeHandle handle,btConvexHullDescription* desc);
	int								BTDrawConvexHullShape(btCollisionShapeHandle handle, BTMatrix4& model, BTMatrix4& view, BTMatrix4& projection);
	void							BTDrawCollisionShape(btCollisionShapeHandle handle, BTMatrix4& modelview, BTMatrix4& projection);

/* Concave static triangle meshes */
	 btMeshInterfaceHandle			BTNewMeshInterface(void);
	 void							BTAddTriangle(btMeshInterfaceHandle meshHandle, BTVector3& v0,BTVector3& v1,BTVector3& v2);
	 btCollisionShapeHandle			BTNewTriangleMeshShape(int nbt, int nbv, float* vertices, int* indices);

	 void							BTSetScaling(btRigidBodyHandle handle, BTVector3 scale);

/* Constraints */
	//btConstraintHandle				BTNewHingeConstraint(btRigidBodyHandle body,BTVector3 pivot,BTVector3 axis,bool usereferenceframe);
	btConstraintHandle				BTNewHingeConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTVector3& pivotA,BTVector3& pivotB,BTVector3& axisA,BTVector3& axisB,bool useReferenceFrameA);
	btConstraintHandle				BTNewHingeConstraint2(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB);
    void							BTSetHingeConstraintLimits(btConstraintHandle constraint,btReal low, btReal high, btReal softness,btReal biasFactor, btReal relaxationFactor);
	btConstraintHandle				BTNewGearConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTVector3& axisA,BTVector3& axisB,float ratio);
	btConstraintHandle				BTNewPoint2PointConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTVector3& pivotA,BTVector3& pivotB);
	btConstraintHandle				BTNewSliderConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTXForm& frameA,BTXForm& frameB,bool useReferenceFrameA);
	btConstraintHandle				BTNewGeneric6DofConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTXForm& frameA,BTXForm& frameB,bool usereferenceframeA);
	void							BTSetGeneric6DofConstraintLimit(btConstraintHandle constraint,int axis, btReal lo, btReal hi);
	void							BTSetGeneric6DofConstraintLinearLowerLimit(btConstraintHandle constraint,BTVector3& limit);
	void							BTSetGeneric6DofConstraintLinearUpperLimit(btConstraintHandle constraint,BTVector3& limit);
	void							BTSetGeneric6DofConstraintAngularLowerLimit(btConstraintHandle constraintHandle,BTVector3& limit);
	void							BTSetGeneric6DofConstraintAngularUpperLimit(btConstraintHandle constraintHandle,BTVector3& limit);
	void							BTAddConstraint(btDynamicsWorldHandle world, btConstraintHandle constraint,bool disableCollisionBetweenLinkedBodies);	 
/* SOLID has Response Callback/Table/Management */
/* PhysX has Triggers, User Callbacks and filtering */
/* ODE has the typedef void dNearCallback (void *data, dGeomID o1, dGeomID o2); */

/*	typedef void plUpdatedPositionCallback(void* userData, plRigidBodyHandle	rbHandle, plVector3 pos); */
/*	typedef void plUpdatedOrientationCallback(void* userData, plRigidBodyHandle	rbHandle, plQuaternion orientation); */

/* get world transform */
	void							BTGetMatrix(btRigidBodyHandle object, BTMatrix4 matrix);
	void							BTGetPosition(btRigidBodyHandle object,BTVector3 position);
	void							BTGetOrientation(btRigidBodyHandle object,BTQuaternion orientation);

/* set world transform (position/orientation) */
	void							BTSetPosition(btRigidBodyHandle object, BTVector3& position);
	void							BTSetOrientation(btRigidBodyHandle object, BTQuaternion& orientation);
	void							BTSetEuler(btReal yaw,btReal pitch,btReal roll, BTQuaternion& orient);
	void							BTSetMatrix(btRigidBodyHandle object, BTMatrix4 matrix);

	void							BTTestVector3(BTVector3& io,float x, float y, float z);
	void							BTTestQuaternion(BTQuaternion& io,float x, float y, float z,float w);

	void							BTTranslate(btRigidBodyHandle handle,btReal x, btReal y, btReal z);
	void							BTTransform(btRigidBodyHandle object, BTQuaternion q, BTVector3 p);

	typedef struct btRayCastResult {
		btRigidBodyHandle		m_body;  
		btCollisionShapeHandle	m_shape; 		
		BTVector3				m_pointWorld; 		
		BTVector3				m_normalWorld;
		long					m_triangleIndex;

	} btRayCastResult;

	int			BTRayCast(btDynamicsWorldHandle world, const BTVector3 rayStart, const BTVector3 rayEnd, btRayCastResult& res);
	int			BTTest(btPhysicsSdkHandle hsdk);
	int					BTRayCastHit(btDynamicsWorldHandle world);

	/* Sweep API */

	/*  plRigidBodyHandle plObjectCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal); */

	/* Continuous Collision Detection API */
	
	// needed for source/blender/blenkernel/intern/collision.c
	double BTNearestPoints(float p1[3], float p2[3], float p3[3], float q1[3], float q2[3], float q3[3], float *pa, float *pb, float normal[3]);

#ifdef __cplusplus
}
#endif


#endif //BULLET_C_API_H

