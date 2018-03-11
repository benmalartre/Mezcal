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

	If possible, use the richer Bullet C++ API, by including <src/btBulletDynamicsCommon.h>
*/

#include "capi.h"

//
// Random
//

static inline btScalar	UnitRand()
{
	return(rand()/(btScalar)RAND_MAX);
}

static inline btScalar	SignedUnitRand()
{
	return(UnitRand()*2-1);
}

static inline btVector3	Vector3Rand()
{
	const btVector3	p=btVector3(SignedUnitRand(),SignedUnitRand(),SignedUnitRand());
	return(p.normalized());
}

btSoftRigidDynamicsWorld* getSoftDynamicsWorld(btDynamicsWorld* world){
	return (btSoftRigidDynamicsWorld*)world;
}
/*
//#define BULLET_TRIANGLE_COLLISION 1
#define BULLET_GIMPACT 1
//#define BULLET_GIMPACT_CONVEX_DECOMPOSITION 1

#define TEST_GIMPACT_TORUS

#ifdef BULLET_GIMPACT
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
	#ifdef BULLET_GIMPACT_CONVEX_DECOMPOSITION
	#include "../Extras/GIMPACTUtils/btGImpactConvexDecompositionShape.h"
	#endif


#else

#include "BulletCollision/Gimpact/btConcaveConcaveCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGIMPACTMeshShape.h"

#endif
	*/
//#include "GlutStuff.h"
//#include "ConvexDecomposition.h"
//#include "GL_ShapeDrawer.h"
///for mouse picking
/*
void pickingPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{

}*/

int	BTCheckSoftBodySolver(btPhysicsSdkHandle sdkHandle){
	btPhysicsSdk* sdk = reinterpret_cast<btPhysicsSdk*>(sdkHandle);
	if(sdk->m_softBodySolver == NULL)return 2;
	return sdk->m_softBodySolver->checkInitialized();
}

static void Init_Volume(btPhysicsSdk * sdk)
{
	/*
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateEllipsoid((*)sdk->m_softBodyWorldInfo,btVector3(35,25,0),
		btVector3(1,1,1)*3,
		512);
	psb->m_materials[0]->m_kLST	=	0.45;
	psb->m_cfg.kVC				=	20;
	psb->setTotalMass(50,true);
	psb->setPose(true,false);

	sdk->m_world->addSoftBody(psb);
	*/
}

int	BTTest(btPhysicsSdkHandle physicsSdkHandle){

	
	return 7;
}

btDynamicsWorldHandle BTTestWorld(bool useMCLPSolver)
{
		///-----includes_end-----

	int i;
	///-----initialization_start-----

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0,-10,0));

	///-----initialization_end-----

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;


	///create a few basic rigid bodies

	//the ground is a cube of side 100 at position y = -56.
	//the sphere will hit it at y = -6, with center at -5
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));

		collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-56,0));

		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		dynamicsWorld->addRigidBody(body);
	}


	{
		//create a dynamic rigidbody

		//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

			startTransform.setOrigin(btVector3(2,10,0));
		
			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
			btRigidBody* body = new btRigidBody(rbInfo);

			dynamicsWorld->addRigidBody(body);
	}



/// Do some simulation


	///-----stepsimulation_start-----
	for (i=0;i<150;i++)
	{
		dynamicsWorld->stepSimulation(1.f/60.f,10);
		/*
		//print positions of all objects
		for (int j=dynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--)
		{
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			btTransform trans;
			if (body && body->getMotionState())
			{
				body->getMotionState()->getWorldTransform(trans);

			} else
			{
				trans = obj->getWorldTransform();
			}
			printf("world pos object %d = %f,%f,%f\n",j,float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()));
		}
		*/
	}
/*
	///-----stepsimulation_end-----

	//cleanup in the reverse order of creation/initialization
	
	///-----cleanup_start-----

	//remove the rigidbodies from the dynamics world and delete them
	for (i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<collisionShapes.size();j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete dynamicsWorld;

	//delete solver
	delete solver;

	//delete broadphase
	delete overlappingPairCache;

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();
	*/
	return (btDynamicsWorldHandle)dynamicsWorld;
};
/*
	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	btBroadphaseInterface* broadphase = new btAxisSweep3(worldMin,worldMax);
	btConstraintSolver* constraintSolver;
	if (useMCLPSolver)
	{
		btDantzigSolver* mlcp = new btDantzigSolver();
		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
		btMLCPSolver* sol = new btMLCPSolver(mlcp);
		constraintSolver = sol;
	} else
	{
		constraintSolver = new btSequentialImpulseConstraintSolver();
	}
	btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(dispatcher,broadphase,constraintSolver,collisionConfiguration);
	if (useMCLPSolver)
	{
		world ->getSolverInfo().m_minimumSolverBatchSize = 1;//for direct solver it is better to have a small A matrix
	} else
	{
		world ->getSolverInfo().m_minimumSolverBatchSize = 128;//for direct solver, it is better to solve multiple objects together, small batches have high overhead
	}
	world->getSolverInfo().m_numIterations = 100;
	return(btDynamicsWorldHandle)world;
*/
void BTTestXForm(BTXForm& T)
{
	T.pos[0] += 1;
	T.pos[1] += 1;
	T.pos[2] += 1;

	T.rot[0] += 45;
}

/*-------------------------------------------------------------------
	BTBulletSDK
-------------------------------------------------------------------*/
btPhysicsSdkHandle	BTCreateDynamicsSdk()
{
	void* mem = btAlignedAlloc(sizeof(btPhysicsSdk),16);
	return (btPhysicsSdkHandle)new (mem)btPhysicsSdk;
}

void BTDeleteDynamicsSdk(btPhysicsSdkHandle	physicsSdk)
{
	btPhysicsSdk* phys = reinterpret_cast<btPhysicsSdk*>(physicsSdk);
	btAssert(phys);
	/*
	if(phys->m_softBodySolver)delete phys->m_softBodySolver;
	if(phys->m_broadphase)delete phys->m_broadphase;
	if(phys->m_dispatcher)delete phys->m_dispatcher;
	if(phys->m_solver)delete phys->m_solver;
	*/
	//if(phys->m_collisionConfiguration)delete phys->m_collisionConfiguration;
	//if (phys)delete phys;	
}

/*
btSoftBodyWorldInfoHandle BTGetSoftBodyWorldInfo(btPhysicsSdkHandle physicsSdkHandle)
{
	btPhysicsSdk* physicsSdk = reinterpret_cast<btPhysicsSdk*>(physicsSdkHandle);
	
	return (btSoftBodyWorldInfoHandle) &physicsSdk->m_softBodyWorldInfo;
}*/

/* Dynamics World */
btDynamicsWorldHandle BTCreateDynamicsWorld(btPhysicsSdkHandle physicsSdkHandle,bool useMCLPSolver)
{
	btPhysicsSdk* physicsSdk = reinterpret_cast<btPhysicsSdk*>(physicsSdkHandle);
	btAssert(physicsSdk);
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btBroadphaseInterface*pairCache = new btAxisSweep3(physicsSdk->m_worldAabbMin,physicsSdk->m_worldAabbMax);//new btDbvtBroadphase();//
	btConstraintSolver* constraintSolver = new btSequentialImpulseConstraintSolver();
	btDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration);
	physicsSdk->m_pick = dynamicsWorld;
	return (btDynamicsWorldHandle)dynamicsWorld;
/*
	btPhysicsSdk* physicsSdk = reinterpret_cast<btPhysicsSdk*>(physicsSdkHandle);
	btAssert(physicsSdk);
	
	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	
	physicsSdk->m_collisionConfiguration = new btDefaultCollisionConfiguration();

	physicsSdk->m_dispatcher = new btCollisionDispatcher(physicsSdk->m_collisionConfiguration);
	physicsSdk->m_worldAabbMin.setValue(-1000,-1000,-1000);
	physicsSdk->m_worldAabbMax.setValue(1000,1000,1000);
	
	physicsSdk->m_broadphase = new btAxisSweep3(physicsSdk->m_worldAabbMin,physicsSdk->m_worldAabbMax);
	if (useMCLPSolver)
	{
		btDantzigSolver* mlcp = new btDantzigSolver();
		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
		btMLCPSolver* sol = new btMLCPSolver(mlcp);
		physicsSdk->m_solver = sol;
	} else
	{
		physicsSdk->m_solver = new btSequentialImpulseConstraintSolver();
	}
	physicsSdk->m_world = new btDiscreteDynamicsWorld(physicsSdk->m_dispatcher,physicsSdk->m_broadphase,physicsSdk->m_solver,physicsSdk->m_collisionConfiguration);
	if (useMCLPSolver)
	{
		physicsSdk->m_world ->getSolverInfo().m_minimumSolverBatchSize = 1;//for direct solver it is better to have a small A matrix
	} else
	{
		physicsSdk->m_world ->getSolverInfo().m_minimumSolverBatchSize = 128;//for direct solver, it is better to solve multiple objects together, small batches have high overhead
	}
	physicsSdk->m_world->getSolverInfo().m_numIterations = 100;

	return (btDynamicsWorldHandle) physicsSdk->m_world;
	*/
	return NULL;

}

void BTDeleteDynamicsWorld(btDynamicsWorldHandle world)
{
	//todo: also clean up the other allocations, axisSweep, pairCache,dispatcher,constraintSolver,collisionConfiguration
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	for(int i=0;i<dynamicsWorld->getNumCollisionObjects();i++){

	}
	delete dynamicsWorld;
}

/* Soft Rigid Dynamics World */
/*
struct btSoftRigidDynamicsWorld{
	btSoftBodyWorldInfo m_softBodyWorldInfo;

};
*/


btDynamicsWorldHandle			BTCreateSoftRigidDynamicsWorld(btPhysicsSdkHandle physicsSdkHandle)
{
	
	btPhysicsSdk* physicsSdk = reinterpret_cast<btPhysicsSdk*>(physicsSdkHandle);
	physicsSdk->m_collisionConfiguration = 0;
	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	physicsSdk->m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	physicsSdk->m_dispatcher = new	btCollisionDispatcher(physicsSdk->m_collisionConfiguration);
	physicsSdk->m_softBodyWorldInfo.m_dispatcher = physicsSdk->m_dispatcher;
/*
	///register GIMPACT algorithm
	#ifdef BULLET_GIMPACT
		btGImpactCollisionAlgorithm::registerAlgorithm(physicsSdk->m_dispatcher);
	#else
		btConcaveConcaveCollisionAlgorithm::registerAlgorithm(physicsSdk->m_dispatcher);
	#endif
*/
	////////////////////////////
	///Register softbody versus softbody collision algorithm
	

	///Register softbody versus rigidbody collision algorithm


	////////////////////////////
/*
	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	physicsSdk->m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	physicsSdk->m_softBodyWorldInfo.m_broadphase = physicsSdk->m_broadphase;

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	physicsSdk->m_solver = solver;

	btSoftBodySolver* softBodySolver = 0;

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(physicsSdk->m_dispatcher,physicsSdk->m_broadphase,physicsSdk->m_solver,physicsSdk->m_collisionConfiguration,softBodySolver);
	physicsSdk->m_world = (btSoftRigidDynamicsWorld*)world;
	


	world->getDispatchInfo().m_enableSPU = true;
	world->setGravity(btVector3(0,-10,0));
	physicsSdk->m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

	//	clientResetScene();

	physicsSdk->m_softBodyWorldInfo.m_sparsesdf.Initialize();
	*/
	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	physicsSdk->m_broadphase = new btDbvtBroadphase();//new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	physicsSdk->m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
	physicsSdk->m_softBodyWorldInfo.water_density	=	0;
	physicsSdk->m_softBodyWorldInfo.water_offset		=	0;
	physicsSdk->m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);

	//physicsSdk->m_softBodySolver = new btDefaultSoftBodySolver();
	physicsSdk->m_softBodyWorldInfo.m_broadphase = physicsSdk->m_broadphase;

	physicsSdk->m_solver = new btSequentialImpulseConstraintSolver();
	physicsSdk->m_softBodySolver = 0;//new btDefaultSoftBodySolver();

	btSoftRigidDynamicsWorld* world = new btSoftRigidDynamicsWorld(physicsSdk->m_dispatcher,
							physicsSdk->m_broadphase,
							physicsSdk->m_solver,
							physicsSdk->m_collisionConfiguration,
							physicsSdk->m_softBodySolver );
	

	//world->setInternalTickCallback(pickingPreTickCallback,NULL,true);
	world->getDispatchInfo().m_enableSPU = false;
	world->setGravity(btVector3(0,-10,0));
	physicsSdk->m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);
	physicsSdk->m_softBodyWorldInfo.m_sparsesdf.Initialize();
	world->setWorldUserInfo((void * )physicsSdkHandle);

	physicsSdk->m_world = world;

	btRigidBody* ground = new btRigidBody(1.0,0,new btBoxShape(btVector3(100,1,100)));

	return (btDynamicsWorldHandle) world;
	
}

void BTDeleteSoftRigidDynamicsWorld(btDynamicsWorldHandle world)
{
	//todo: also clean up the other allocations, axisSweep, pairCache,dispatcher,constraintSolver,collisionConfiguration
	btSoftRigidDynamicsWorld* dynamicsWorld = reinterpret_cast< btSoftRigidDynamicsWorld* >(world);

	/* Clean up	*/ 
	for(int i=dynamicsWorld->getNumCollisionObjects()-1;i>=0;i--)
	{
		btCollisionObject*	obj=dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
		while(dynamicsWorld->getNumConstraints())
		{
			btTypedConstraint*	pc=dynamicsWorld->getConstraint(0);
			dynamicsWorld->removeConstraint(pc);
			delete pc;
		}
		btSoftBody* softBody = btSoftBody::upcast(obj);
		if (softBody)
		{
			getSoftDynamicsWorld(dynamicsWorld)->removeSoftBody(softBody);
		} else
		{
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body)
				dynamicsWorld->removeRigidBody(body);
			else
				dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}
	delete dynamicsWorld;
}

btDynamicsWorldHandle	BTGetDynamicsWorld(btPhysicsSdkHandle physicsSdk)
{
	btPhysicsSdk* sdk = reinterpret_cast< btPhysicsSdk* >(physicsSdk);
	btAssert(sdk);

	return (btDynamicsWorldHandle)sdk->m_world;
}

void BTGetWorldBoundingBox(btPhysicsSdkHandle sdk,btVector3& bb_min,btVector3& bb_max)
{
	btPhysicsSdk* physicsSdk = reinterpret_cast<btPhysicsSdk*>(sdk);
	bb_min = physicsSdk->m_worldAabbMin;
	bb_max = physicsSdk->m_worldAabbMax;
	//btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	
}

int	BTStepSimulation(btDynamicsWorldHandle hWorld,	btReal	timeStep)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast<btDynamicsWorld*>(hWorld);
	btAssert(dynamicsWorld);
	dynamicsWorld->stepSimulation(timeStep);

	//btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)dynamicsWorld;
	//btAssert(softWorld);
	//dynamicsWorld->stepSimulation(timeStep,1/60.0);
	//softWorld->getWorldInfo().m_sparsesdf.GarbageCollect();
	//m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();
	return 1;
	/*
	btDynamicsWorld* dynamicsWorld = (btDynamicsWorld*)hWorld;
	btAssert(dynamicsWorld);

	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)dynamicsWorld;
	btAssert(softWorld);

	btPhysicsSdk * sdk = (btPhysicsSdk *)softWorld->getWorldUserInfo();
	softWorld->stepSimulation(timeStep);
	sdk->m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();
	return (int)(timeStep*100);
	*/

}

void BTResetSparseSDF(btPhysicsSdkHandle sdkHandle){
	btPhysicsSdk* sdk = reinterpret_cast<btPhysicsSdk*>(sdkHandle);
	sdk->m_softBodyWorldInfo.m_sparsesdf.Reset();
}

int BTGetNumSoftBodies(btDynamicsWorldHandle hWorld){
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(hWorld);
	btAssert(dynamicsWorld);

	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)dynamicsWorld;
	btAssert(softWorld);
	return (int)softWorld->getSoftBodyArray().size();
}

void BTSetGravity(btDynamicsWorldHandle world, BTVector3 gravity)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	dynamicsWorld->setGravity(btVector3(gravity[0],gravity[1],gravity[2]));
}

int	BTGetNumCollideObjects(btDynamicsWorldHandle world)
{
	btSoftRigidDynamicsWorld* dynamicsWorld = reinterpret_cast< btSoftRigidDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	return dynamicsWorld->getNumCollisionObjects();
}

btRigidBodyHandle BTGetRigidBodyByID(btDynamicsWorldHandle world, int id)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	btCollisionObjectArray bodies = dynamicsWorld->getCollisionObjectArray();
	if(id<bodies.size())
		return (btRigidBodyHandle)bodies[id];
	else
		return NULL;
}

void BTAddRigidBody(btDynamicsWorldHandle world, btRigidBodyHandle object)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);

	body->setActivationState(4);
	dynamicsWorld->addRigidBody(body);
}

void BTRemoveRigidBody(btDynamicsWorldHandle world, btRigidBodyHandle object)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	dynamicsWorld->removeRigidBody(body);
}

void BTAddSoftBody(btDynamicsWorldHandle world, btSoftBodyHandle object)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	btSoftBody* soft = reinterpret_cast< btSoftBody* >(object);
	btAssert(soft);

	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)dynamicsWorld;
	softWorld->addSoftBody(soft);
}

void BTRemoveSoftBody(btDynamicsWorldHandle world, btRigidBodyHandle object)
{
	
	btSoftRigidDynamicsWorld* dynamicsWorld = reinterpret_cast< btSoftRigidDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	btSoftBody* soft = reinterpret_cast< btSoftBody* >(object);
	btAssert(soft);

	dynamicsWorld->removeSoftBody(soft);
}

btSoftBodyHandle BTGetSoftBodyByID(btDynamicsWorldHandle world, int id)
{
	btSoftRigidDynamicsWorld* dynamicsWorld = reinterpret_cast< btSoftRigidDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	btSoftBodyArray bodies = dynamicsWorld->getSoftBodyArray();
	if(id<bodies.size())
		return (btSoftBodyHandle)bodies[id];
	else
		return NULL;
}

int BTGetSoftBodyNbVertices(btSoftBodyHandle hBody){
	btSoftBody* psb = reinterpret_cast<btSoftBody*>(hBody);
	btAssert(psb);
	int nbv = 0;
	int i,j,nj;
	for(int i=0;i<psb->m_clusters.size();i++){
		btAlignedObjectArray<btVector3>	vertices;
		vertices.resize(psb->m_clusters[i]->m_nodes.size());
		nbv + vertices.size();
		for(j=0,nj=vertices.size();j<nj;++j)
		{				
			vertices[j]=psb->m_clusters[i]->m_nodes[j]->m_x;
		}
	}
	return nbv;
}

int BTGetSoftBodyNbFaces(btSoftBodyHandle hBody){
	btSoftBody* psb = reinterpret_cast<btSoftBody*>(hBody);
	btAssert(psb);
	const btScalar	scl=(btScalar)0.8;
	const btScalar	alp=(btScalar)1;
	const btVector3	col(0,(btScalar)0.7,0);
	int i;

	for(i=0;i<psb->m_faces.size();++i)
	{
		const btSoftBody::Face&	f=psb->m_faces[i];
		if(0==(f.m_material->m_flags&btSoftBody::fMaterial::DebugDraw)) continue;
		const btVector3			x[]={f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x};
		const btVector3			c=(x[0]+x[1]+x[2])/3;

		/*
		idraw->drawTriangle((x[0]-c)*scl+c,
			(x[1]-c)*scl+c,
			(x[2]-c)*scl+c,
			col,alp);
		*/
	}	

	return psb->m_faces.size();
}

int BTGetSoftBodyNbNodes(btSoftBodyHandle hBody){
	btSoftBody* psb = reinterpret_cast<btSoftBody*>(hBody);
	btAssert(psb);

	return psb->m_nodes.size();
}

btRigidBody*	localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);	
	body->setWorldTransform(startTransform);
#endif//

	//m_dynamicsWorld->addRigidBody(body);

	return body;
}

/* Constraints */



/** NOT Working ---> Crash Pure Basic **/
btConstraintHandle BTNewHingeConstraintWorld(btRigidBodyHandle body,BTVector3& pivot,BTVector3& axis,bool usereferenceframe)
{
	btRigidBody* b = reinterpret_cast<btRigidBody*>(body);
	btAssert(b);

	btTransform T;
	T.setIdentity();
	btCollisionShape* shape = new btBoxShape(btVector3(0.01,0.01,0.01));

	/*
	btRigidBody* b2 = localCreateRigidBody(0.0, T,shape);
	*/
	const btVector3 p(pivot[0],pivot[1],pivot[2] ); 
	btVector3 a( axis[0],axis[1],axis[2]);
	/*
	btTransform tA;
	btTransform tB;
	tB.setIdentity();
	*/
	void* mem = btAlignedAlloc(sizeof(btHingeConstraint),16);
	btHingeConstraint* hinge = new(mem) btHingeConstraint(*b,p,a,usereferenceframe);
	hinge->enableAngularMotor(true,12,1);
	hinge->enableMotor(true);
	return (btConstraintHandle)hinge;

}

/* Hinge Constraint the ONE working*/
btConstraintHandle BTNewHingeConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTVector3& pivotA,BTVector3& pivotB,BTVector3& axisA,BTVector3& axisB,bool usereferenceframe)
{

	btRigidBody* bA = reinterpret_cast<btRigidBody*>(bodyA);
	btAssert(bA);
	btRigidBody* bB = reinterpret_cast<btRigidBody*>(bodyB);
	btAssert(bB);

	const btVector3 pA(pivotA[0],pivotA[1],pivotA[2] ); 
	const btVector3 pB(pivotB[0],pivotB[1],pivotB[2] ); 
	btVector3 aA( axisA[0],axisA[1],axisA[2]);
	btVector3 aB( axisB[0],axisB[1],axisB[2]);

	/*
	bA->setLinearFactor(btVector3(0,0,0));
	bB->setLinearFactor(btVector3(0,0,0));
	*/

	void* mem = btAlignedAlloc(sizeof(btHingeConstraint),16);
	btHingeConstraint* hinge = new(mem) btHingeConstraint(*bA,*bB, pA,pB,aA,aB,usereferenceframe);

	//hinge->setAngularOnly(true);
	hinge->enableAngularMotor(true,12,1);
	hinge->enableMotor(true);
	return (btConstraintHandle)hinge;
}
/* Hinge Constraint NOT Tested in PureBasic */
btConstraintHandle BTNewHingeConstraint2(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB)
{
	btRigidBody* bA = reinterpret_cast<btRigidBody*>(bodyA);
	btAssert(bA);
	btRigidBody* bB = reinterpret_cast<btRigidBody*>(bodyB);
	btAssert(bB);

	btTransform liftTrans;
	btVector3 m_liftStartPos = btVector3(0.0f, 2.5f, 3.05f);
	liftTrans.setIdentity();
	liftTrans.setOrigin(m_liftStartPos);

	btTransform localA, localB;
	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 3.14/2, 0);
	localA.setOrigin(btVector3(0.0, 1.0, 3.05));
	localB.getBasis().setEulerZYX(0, 3.14/2, 0);
	localB.setOrigin(btVector3(0.0, -1.5, -0.05));
	btHingeConstraint* hinge = new btHingeConstraint(*bA,*bB, localA, localB);
//		m_liftHinge->setLimit(-LIFT_EPS, LIFT_EPS);
	hinge->setLimit(0.0f, 0.0f);

	return (btConstraintHandle)hinge;
}	

void BTSetHingeConstraintLimits(btConstraintHandle constraint,btReal low, btReal high, btReal softness=0.9f,btReal biasFactor=0.3f, btReal relaxationFactor=1.0f)
{
	btHingeConstraint* cns = reinterpret_cast<btHingeConstraint*>(constraint);
	btAssert(cns);

	cns->setLimit(low,high,softness,biasFactor,relaxationFactor);
}


/* Point to Point Constraint */
btConstraintHandle BTNewPoint2PointConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTVector3& pivotA,BTVector3& pivotB)
{
	btRigidBody* A = reinterpret_cast<btRigidBody*>(bodyA);
	btAssert(A);
	btRigidBody* B = reinterpret_cast<btRigidBody*>(bodyB);
	btAssert(B);

	void* mem = btAlignedAlloc(sizeof(btPoint2PointConstraint),16);
	btPoint2PointConstraint* gear = new(mem) btPoint2PointConstraint(*A,*B,btVector3(pivotA[0],pivotA[1],pivotA[2]),btVector3(pivotB[0],pivotB[1],pivotB[2]));

	return (btConstraintHandle)gear;
}

/* Gear Constraint */
btConstraintHandle BTNewGearConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTVector3& axisA,BTVector3& axisB,float ratio)
{
	btRigidBody* A = reinterpret_cast<btRigidBody*>(bodyA);
	btAssert(A);
	btRigidBody* B = reinterpret_cast<btRigidBody*>(bodyB);
	btAssert(B);

	void* mem = btAlignedAlloc(sizeof(btGearConstraint),16);
	btGearConstraint* gear = new(mem) btGearConstraint(*A,*B,btVector3(axisA[0],axisA[1],axisA[2]),btVector3(axisB[0],axisB[1],axisB[2]),ratio);

	return (btConstraintHandle)gear;
}

/* Slider Constraint */
btConstraintHandle BTNewSliderConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTXForm& frameA,BTXForm& frameB,bool usereferenceframeA)
{
	btRigidBody* A = reinterpret_cast<btRigidBody*>(bodyA);
	btAssert(A);
	btRigidBody* B = reinterpret_cast<btRigidBody*>(bodyB);
	btAssert(B);

	btTransform fA;
	btTransform fB;
	fA.setRotation(btQuaternion(frameA.rot[0],frameA.rot[1],frameA.rot[2],frameA.rot[3]));
	fA.setOrigin(btVector3(frameA.pos[0],frameA.pos[1],frameA.pos[2]));
	fB.setRotation(btQuaternion(frameB.rot[0],frameB.rot[1],frameB.rot[2],frameB.rot[3]));
	fB.setOrigin(btVector3(frameB.pos[0],frameB.pos[1],frameB.pos[2]));

	void* mem = btAlignedAlloc(sizeof(btSliderConstraint),16);
	btSliderConstraint* slider = new(mem) btSliderConstraint(*A,*B,fA,fB,usereferenceframeA);

	return (btConstraintHandle)slider;
}

/* Generic6Dof Constraint */
btConstraintHandle BTNewGeneric6DofConstraint(btRigidBodyHandle bodyA,btRigidBodyHandle bodyB,BTXForm& frameA,BTXForm& frameB,bool usereferenceframeA)
{
	btRigidBody* A = reinterpret_cast<btRigidBody*>(bodyA);
	btAssert(A);
	btRigidBody* B = reinterpret_cast<btRigidBody*>(bodyB);
	btAssert(B);

	btTransform fA;
	btTransform fB;
	fA.setRotation(btQuaternion(frameA.rot[0],frameA.rot[1],frameA.rot[2],frameA.rot[3]));
	fA.setOrigin(btVector3(frameA.pos[0],frameA.pos[1],frameA.pos[2]));
	fB.setRotation(btQuaternion(frameB.rot[0],frameB.rot[1],frameB.rot[2],frameB.rot[3]));
	fB.setOrigin(btVector3(frameB.pos[0],frameB.pos[1],frameB.pos[2]));

	void* mem = btAlignedAlloc(sizeof(btGeneric6DofConstraint),16);
	btGeneric6DofConstraint* generic6dof = new(mem) btGeneric6DofConstraint(*A,*B,fA,fB,usereferenceframeA);
	
	return (btConstraintHandle)generic6dof;
}

void BTSetGeneric6DofConstraintLimit(btConstraintHandle constraintHandle,int axis,btReal lo, btReal hi)
{
	btGeneric6DofConstraint* cns = reinterpret_cast<btGeneric6DofConstraint*>(constraintHandle);
	btAssert(cns);
	cns->setLimit(axis,lo,hi);
}

void BTSetGeneric6DofConstraintLinearLowerLimit(btConstraintHandle constraintHandle,BTVector3& limit)
{
	btGeneric6DofConstraint* cns = reinterpret_cast<btGeneric6DofConstraint*>(constraintHandle);
	btAssert(cns);
	cns->setLinearLowerLimit(btVector3(limit [0],limit [1],limit [2]));
}

void BTSetGeneric6DofConstraintLinearUpperLimit(btConstraintHandle constraintHandle,BTVector3& limit)
{
	btGeneric6DofConstraint* cns = reinterpret_cast<btGeneric6DofConstraint*>(constraintHandle);
	btAssert(cns);
	cns->setLinearUpperLimit(btVector3(limit [0],limit [1],limit [2]));
}

void BTSetGeneric6DofConstraintAngularLowerLimit(btConstraintHandle constraintHandle,BTVector3& limit)
{
	btGeneric6DofConstraint* cns = reinterpret_cast<btGeneric6DofConstraint*>(constraintHandle);
	btAssert(cns);
	cns->setAngularLowerLimit(btVector3(limit [0],limit [1],limit [2]));
}

void BTSetGeneric6DofConstraintAngularUpperLimit(btConstraintHandle constraintHandle,BTVector3& limit)
{
	btGeneric6DofConstraint* cns = reinterpret_cast<btGeneric6DofConstraint*>(constraintHandle);
	btAssert(cns);
	cns->setLinearUpperLimit(btVector3(limit [0],limit [1],limit [2]));
}

/* Add Constraint to the world */
void BTAddConstraint(btDynamicsWorldHandle world, btConstraintHandle constraint,bool disableCollisionBetweenLinkedBodies)
{
	btDynamicsWorld* w = reinterpret_cast<btDynamicsWorld*>(world);
	btAssert(w);
	btTypedConstraint* c = reinterpret_cast<btTypedConstraint*>(constraint);
	btAssert(c);
	w->addConstraint(c,disableCollisionBetweenLinkedBodies);
}

/* Rigid Body  */
btRigidBodyHandle BTCreateRigidBody(	void* user_data,  float mass, btCollisionShapeHandle cshape )
{
	btTransform trans;
	trans.setIdentity();
	btVector3 localInertia(0,0,0);
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( cshape);
	btAssert(shape);
	if (mass)
	{
		shape->calculateLocalInertia(mass,localInertia);
	}
	// create motion state
    btDefaultMotionState* defaultMotionState = new btDefaultMotionState( trans );
	btRigidBody::btRigidBodyConstructionInfo rbci(mass, defaultMotionState,shape,localInertia);

	void* mem = btAlignedAlloc(sizeof(btRigidBody),16);
	btRigidBody* body = new(mem) btRigidBody(rbci);
	body->setWorldTransform(trans);
	body->setInterpolationWorldTransform(trans);

	body->setUserPointer(user_data);
	return (btRigidBodyHandle) body;
}

C3DObjectPointer	BTGetUserData(btRigidBodyHandle hbody)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(hbody);
	btAssert(body);
	return body->getUserPointer();
}

void BTDeleteRigidBody(btRigidBodyHandle cbody)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(cbody);
	btAssert(body);
	btAlignedFree( body);
}

void BTSetLinearFactor(btRigidBodyHandle cbody, BTVector3& factor)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(cbody);
	btAssert(body);
	body->setLinearFactor(btVector3(factor[0],factor[1],factor[2]));
}

void BTSetAngularFactorF(btRigidBodyHandle cbody, float factor)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(cbody);
	btAssert(body);
	body->setAngularFactor(factor);
}

void BTSetAngularFactor(btRigidBodyHandle cbody, BTVector3& factor)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(cbody);
	btAssert(body);
	body->setAngularFactor(btVector3(factor[0],factor[1],factor[2]));
}

void BTSetLinearVelocity(btRigidBodyHandle cbody, BTVector3& velocity)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(cbody);
	btAssert(body);
	body->setLinearVelocity(btVector3(velocity[0],velocity[1],velocity[2]));
}

void BTSetAngularVelocity(btRigidBodyHandle cbody, BTVector3& velocity)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(cbody);
	btAssert(body);
	body->setAngularVelocity(btVector3(velocity[0],velocity[1],velocity[2]));
}

/* Soft Body */
btSoftBodyHandle BTSoftBox(btPhysicsSdkHandle sdkHandle,BTVector3 ip,BTVector3 is)
{
	btPhysicsSdk* sdk = reinterpret_cast< btPhysicsSdk* >(sdkHandle);
	btAssert(sdk);

	const btVector3	h= btVector3(is[0],is[1],is[2])*0.5;
	const btVector3 p(ip[0],ip[1],ip[2]);

	const btVector3	c[]={	p+h*btVector3(-1,-1,-1),
		p+h*btVector3(+1,-1,-1),
		p+h*btVector3(-1,+1,-1),
		p+h*btVector3(+1,+1,-1),
		p+h*btVector3(-1,-1,+1),
		p+h*btVector3(+1,-1,+1),
		p+h*btVector3(-1,+1,+1),
		p+h*btVector3(+1,+1,+1)};

	btSoftBody*	psb=btSoftBodyHelpers::CreateFromConvexHull(sdk->m_softBodyWorldInfo,c,8);

	psb->generateBendingConstraints(2);

	return(btSoftBodyHandle)psb;

}

btSoftBodyHandle BTSoftBoulder(btPhysicsSdkHandle sdkHandle,BTVector3 ip,BTVector3 is,int np,int id)
{
	btPhysicsSdk* sdk = reinterpret_cast< btPhysicsSdk* >(sdkHandle);
	btVector3 p(ip[0],ip[1],ip[2]);
	btVector3 s(is[0],is[1],is[2]);
	btAssert(sdk);
	btAlignedObjectArray<btVector3>	pts;
	if(id) srand(id);
	for(int i=0;i<np;++i)
	{
		pts.push_back(Vector3Rand()*s+p);
	}
	btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(sdk->m_softBodyWorldInfo,&pts[0],pts.size());
	
	psb->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.4;
	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2,pm);
	psb->setTotalMass(150);

	return(btSoftBodyHandle)psb;
}

void BTUpdateSoftBodyGeometry(btSoftBodyHandle hBody, btReal* vertices){
	btSoftBody* psb = reinterpret_cast<btSoftBody*>(hBody);
	btAssert(psb);

	for(int i=0;i<psb->m_nodes.size();i++){
		vertices[i*3] = psb->m_nodes[i].m_x.x();
		vertices[i*3+1] = psb->m_nodes[i].m_x.y();
		vertices[i*3+2] = psb->m_nodes[i].m_x.z();
	}
}
int BTCheckSoftBodyWorldInfo(btPhysicsSdkHandle sdkHandle){
	btPhysicsSdk* sdk = reinterpret_cast< btPhysicsSdk* >(sdkHandle);
	btAssert(sdk);
	
	return sdk->m_softBodyWorldInfo.m_sparsesdf.ncells;

}

btSoftBodyHandle BTCreateSoftBodyFromTriMesh( void* user_data,btPhysicsSdkHandle hsdk, btReal* vertices,int* indices, int nb_triangles)
{
	btPhysicsSdk* sdk = reinterpret_cast< btPhysicsSdk* >(hsdk);
	btAssert(sdk);
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(sdk->m_softBodyWorldInfo,vertices,indices,nb_triangles);
	btSoftBody::Material*	pm=psb->appendMaterial();
	psb->generateBendingConstraints(2);
	psb->m_cfg.piterations=2;
	psb->randomizeConstraints();
	psb->setPose(true,false);
	psb->setUserPointer(user_data);
	psb->setTotalMass(50,true);
	return(btSoftBodyHandle)psb;

}

btSoftBodyHandle BTCreateSoftBodyFromConvexHull( void* user_data,btPhysicsSdkHandle hsdk, btReal* vertices,int* indices, int nb_triangles)
{
	btPhysicsSdk* sdk = reinterpret_cast<btPhysicsSdk*>(hsdk);
	btAssert(sdk);

	//MyConvexDecomposition* decomp = new MyConvexDecomposition(NULL);
	btTriangleMesh* trimesh = new btTriangleMesh();

	btVector3 localScaling(1.f,1.f,1.f);
	
	int i;
	for ( i=0;i<nb_triangles;i++)
	{
		int index0 = indices[i*3];
		int index1 = indices[i*3+1];
		int index2 = indices[i*3+2];

		btVector3 vertex0(vertices[index0*3], vertices[index0*3+1],vertices[index0*3+2]);
		btVector3 vertex1(vertices[index1*3], vertices[index1*3+1],vertices[index1*3+2]);
		btVector3 vertex2(vertices[index2*3], vertices[index2*3+1],vertices[index2*3+2]);
		
		vertex0 *= localScaling;
		vertex1 *= localScaling;
		vertex2 *= localScaling;

		trimesh->addTriangle(vertex0,vertex1,vertex2);
	}

	btConvexShape* tmpConvexShape = new btConvexTriangleMeshShape(trimesh);

	//create a hull approximation
	btShapeHull* hull = new btShapeHull(tmpConvexShape);
	btScalar margin = tmpConvexShape->getMargin();
	hull->buildHull(margin);
	tmpConvexShape->setUserPointer(hull);

	btSoftBody* psb=btSoftBodyHelpers::CreateFromConvexHull(sdk->m_softBodyWorldInfo,&hull->getVertexPointer()[0],hull->numVertices());
	btSoftBody::Material*	pm=psb->appendMaterial();
	pm->m_kLST				=	0.5;
	pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2,pm);
	psb->m_cfg.piterations	=	2;
	psb->m_cfg.kDF			=	0.5;
	psb->m_cfg.collisions	|=	btSoftBody::fCollision::VF_SS;
	psb->randomizeConstraints();
	/*
	btMatrix3x3	m;
	m.setEulerZYX(a.x(),a.y(),a.z());
	psb->transform(btTransform(m,x));
	psb->scale(btVector3(6,6,6));
	*/
	psb->setTotalMass(100,true);

	psb->setUserPointer(user_data);
	delete tmpConvexShape;
	delete hull;	

	return (btSoftBodyHandle) psb;
}

//(SoftDemo* pdemo,const btVector3& x,const btVector3& a,const btVector3& s=btVector3(2,2,2))
btSoftBodyHandle BTCreateClusterSoftBodyFromTriMesh(void* user_data,btPhysicsSdkHandle hsdk, btReal* vertices,int* indices, int nb_triangles,int nb_clusters)
{
	btPhysicsSdk* sdk = reinterpret_cast< btPhysicsSdk* >(hsdk);
	btAssert(sdk);
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(sdk->m_softBodyWorldInfo,vertices,indices,nb_triangles);
	btSoftBody::Material* pm = psb->appendMaterial();
	pm->m_kLST = 1;
	pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2,pm);
	psb->m_cfg.piterations = 2;
	psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS+btSoftBody::fCollision::CL_RS+btSoftBody::fCollision::CL_SELF;
	psb->randomizeConstraints();
	psb->generateClusters(nb_clusters);
	psb->setPose(true,false);
	psb->setUserPointer(user_data);
	return(btSoftBodyHandle)psb;
	/*
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,gVertices,&gIndices[0][0],NUM_TRIANGLES);
	btSoftBody::Material*	pm=psb->appendMaterial();
	pm->m_kLST				=	1;
	pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;			
	psb->generateBendingConstraints(2,pm);
	psb->m_cfg.piterations	=	2;
	psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+
		btSoftBody::fCollision::CL_RS;
	psb->randomizeConstraints();
	psb->scale(s);
	psb->rotate(btQuaternion(a[0],a[1],a[2]));
	psb->translate(x);
	psb->setTotalMass(50,true);
	psb->generateClusters(64);			
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	return(psb);
	*/
}

int BTUpdatePointPosition(btSoftBodyHandle sbdh,btReal* vertices)
{
	btSoftBody* psb = reinterpret_cast<btSoftBody*> (sbdh);
	btAssert(psb);

	if(psb->m_pose.m_bframe)
	{
		static const btScalar	ascl=10;
		static const btScalar	nscl=(btScalar)0.1;
		const btVector3			com=psb->m_pose.m_com;
		const btMatrix3x3		trs=psb->m_pose.m_rot*psb->m_pose.m_scl;
		const btVector3			Xaxis=(trs*btVector3(1,0,0)).normalized();
		const btVector3			Yaxis=(trs*btVector3(0,1,0)).normalized();
		const btVector3			Zaxis=(trs*btVector3(0,0,1)).normalized();
		//idraw->drawLine(com,com+Xaxis*ascl,btVector3(1,0,0));
		//idraw->drawLine(com,com+Yaxis*ascl,btVector3(0,1,0));
		//idraw->drawLine(com,com+Zaxis*ascl,btVector3(0,0,1));
		for(int i=0;i<psb->m_pose.m_pos.size();++i)
		{
			const btVector3	x=com+trs*psb->m_pose.m_pos[i];
			memcpy(vertices+i*3*sizeof(btVector3),psb->m_pose.m_pos[i],sizeof(btVector3));
			//drawVertex(idraw,x,nscl,btVector3(1,0,1));
		}
		return psb->m_pose.m_pos.size();
	}
	return 0;
}

/* Collision Shape definition */
btCollisionShapeHandle BTNewGroundPlaneShape(BTVector3 norm, btReal size)
{
	void* mem = btAlignedAlloc(sizeof(btStaticPlaneShape),16);
	return (btCollisionShapeHandle) new (mem)btStaticPlaneShape(btVector3(norm[0],norm[1],norm[2]),size);
}
	
btCollisionShapeHandle BTNewSphereShape(btReal radius)
{
	void* mem = btAlignedAlloc(sizeof(btSphereShape),16);
	return (btCollisionShapeHandle) new (mem)btSphereShape(radius);
	
}
	
btCollisionShapeHandle BTNewBoxShape(btReal x, btReal y, btReal z)
{
	void* mem = btAlignedAlloc(sizeof(btBoxShape),16);
	return (btCollisionShapeHandle) new (mem)btBoxShape(btVector3(x,y,z));
}

btCollisionShapeHandle BTNewCapsuleShape(btReal radius, btReal height)
{
	//capsule is convex hull of 2 spheres, so use btMultiSphereShape
	
	const int numSpheres = 2;
	btVector3 positions[numSpheres] = {btVector3(0,height,0),btVector3(0,-height,0)};
	btScalar radi[numSpheres] = {radius,radius};
	void* mem = btAlignedAlloc(sizeof(btMultiSphereShape),16);
	return (btCollisionShapeHandle) new (mem)btMultiSphereShape(positions,radi,numSpheres);
}
btCollisionShapeHandle BTNewConeShape(btReal radius, btReal height)
{
	void* mem = btAlignedAlloc(sizeof(btConeShape),16);
	return (btCollisionShapeHandle) new (mem)btConeShape(radius,height);
}

btCollisionShapeHandle BTNewCylinderShape(btReal radius, btReal height)
{
	void* mem = btAlignedAlloc(sizeof(btCylinderShape),16);
	return (btCollisionShapeHandle) new (mem)btCylinderShape(btVector3(radius,height,radius));
}

/* Convex Meshes */
btCollisionShapeHandle BTNewEmptyConvexHullShape()
{
	void* mem = btAlignedAlloc(sizeof(btConvexHullShape),16);
	return (btCollisionShapeHandle) new (mem)btConvexHullShape();
}

btCollisionShapeHandle BTNewConvexHullShape(int num_tri,int* indices,int num_vertices,btReal* vertices)
{
	//MyConvexDecomposition* decomp = new MyConvexDecomposition(NULL);
	btTriangleMesh* trimesh = new btTriangleMesh();

	btVector3 localScaling(1.f,1.f,1.f);
	
	int i;
	for ( i=0;i<num_tri;i++)
	{
		int index0 = indices[i*3];
		int index1 = indices[i*3+1];
		int index2 = indices[i*3+2];

		btVector3 vertex0(vertices[index0*3], vertices[index0*3+1],vertices[index0*3+2]);
		btVector3 vertex1(vertices[index1*3], vertices[index1*3+1],vertices[index1*3+2]);
		btVector3 vertex2(vertices[index2*3], vertices[index2*3+1],vertices[index2*3+2]);
		
		vertex0 *= localScaling;
		vertex1 *= localScaling;
		vertex2 *= localScaling;

		trimesh->addTriangle(vertex0,vertex1,vertex2);
	}

	btConvexShape* tmpConvexShape = new btConvexTriangleMeshShape(trimesh);

	//create a hull approximation
	btShapeHull* hull = new btShapeHull(tmpConvexShape);
	btScalar margin = tmpConvexShape->getMargin();
	hull->buildHull(margin);
	tmpConvexShape->setUserPointer(hull);

	void* mem = btAlignedAlloc(sizeof(btConvexHullShape),16);
	btConvexHullShape* convexShape = new (mem)btConvexHullShape();
	for (i=0;i<hull->numVertices();i++)
	{
		convexShape->addPoint(hull->getVertexPointer()[i]);	
	}

	delete tmpConvexShape;
	delete hull;	

	return (btCollisionShapeHandle) convexShape;
}


/* Concave static triangle meshes */
btMeshInterfaceHandle		   BTNewMeshInterface()
{
	return 0;
}

btCollisionShapeHandle BTNewCompoundShape()
{
	void* mem = btAlignedAlloc(sizeof(btCompoundShape),16);
	return (btCollisionShapeHandle) new (mem)btCompoundShape();
}

void	BTAddChildShape(btCollisionShapeHandle compoundShapeHandle,btCollisionShapeHandle childShapeHandle, BTVector3 childPos,BTQuaternion childOrn)
{
	btCollisionShape* colShape = reinterpret_cast<btCollisionShape*>(compoundShapeHandle);
	btAssert(colShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);
	btCompoundShape* compoundShape = reinterpret_cast<btCompoundShape*>(colShape);
	btCollisionShape* childShape = reinterpret_cast<btCollisionShape*>(childShapeHandle);
	btTransform	localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(childPos[0],childPos[1],childPos[2]));
	localTrans.setRotation(btQuaternion(childOrn[0],childOrn[1],childOrn[2],childOrn[3]));
	compoundShape->addChildShape(localTrans,childShape);
}

btCollisionShapeHandle  BTNewGImpactShape(int num_tri,int* indices,int num_vertices,btReal* vertices)
{
	btTriangleIndexVertexArray* m_indexVertexArrays = new btTriangleIndexVertexArray(num_tri,
											indices,
											3*sizeof(int),
											num_vertices,
											vertices,sizeof(btReal)*3);

	btCollisionShape* m_trimeshShape(NULL);
/*
	#ifdef BULLET_GIMPACT
		#ifdef BULLET_GIMPACT_CONVEX_DECOMPOSITION
			btGImpactConvexDecompositionShape * trimesh  = new
			btGImpactConvexDecompositionShape(
			m_indexVertexArrays, btVector3(1.f,1.f,1.f),btScalar(0.01));
			trimesh->setMargin(0.07);
			trimesh->updateBound();


		#else
			btGImpactMeshShape * trimesh = new btGImpactMeshShape(m_indexVertexArrays);
			trimesh->setLocalScaling(btVector3(1.f,1.f,1.f));
			#ifdef BULLET_TRIANGLE_COLLISION 
			trimesh->setMargin(0.07f); ///?????
			#else
			trimesh->setMargin(0.0f);
			#endif
			trimesh->updateBound();
		#endif

		m_trimeshShape = trimesh;

	#else
		m_trimeshShape = new btGIMPACTMeshData(m_indexVertexArrays);
	#endif*/

	return (btCollisionShapeHandle)m_trimeshShape;
}


void BTSetCollisionMargin(btRigidBodyHandle hBody,btReal margin){
	btRigidBody* body = reinterpret_cast<btRigidBody*>(hBody);
	btAssert(body);
	btCollisionShape* shape = body->getCollisionShape();
	btAssert(shape);
	shape->setMargin(margin);
}

btReal BTSetCollisionMargin(btRigidBodyHandle hBody){
	btRigidBody* body = reinterpret_cast<btRigidBody*>(hBody);
	btAssert(body);
	btCollisionShape* shape = body->getCollisionShape();
	btAssert(shape);
	return(btReal)shape->getMargin();
}

void BTSetFriction(btRigidBodyHandle hBody,btReal friction){
	btRigidBody* body = reinterpret_cast<btRigidBody*>(hBody);
	btAssert(body);
	body->setFriction((btScalar)friction);
}

void BTSetEuler(btReal yaw,btReal pitch,btReal roll, BTQuaternion orient)
{
	btQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	orient[0] = orn.getX();
	orient[1] = orn.getY();
	orient[2] = orn.getZ();
	orient[3] = orn.getW();

}


//	 void		plAddTriangle(plMeshInterfaceHandle meshHandle, plVector3 v0,plVector3 v1,plVector3 v2);
//	 plCollisionShapeHandle plNewStaticTriangleMeshShape(plMeshInterfaceHandle);


void		BTAddVertex(btCollisionShapeHandle cshape, btReal x,btReal y,btReal z)
{
	btCollisionShape* colShape = reinterpret_cast<btCollisionShape*>( cshape);
	(void)colShape;
	btAssert(colShape->getShapeType()==CONVEX_HULL_SHAPE_PROXYTYPE);
	btConvexHullShape* convexHullShape = reinterpret_cast<btConvexHullShape*>( cshape);
	//convexHullShape->addPoint(btVector3(x,y,z));

}

void BTAddTriangle(btMeshInterfaceHandle meshHandle, BTVector3 v0,BTVector3 v1,BTVector3 v2)
{
	
}



void BTDeleteShape(btCollisionShapeHandle cshape)
{
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( cshape);
	btAssert(shape);
	btAlignedFree(shape);
}
void BTSetScaling(btRigidBodyHandle hBody, BTVector3 cscaling)
{
	btRigidBody* body = reinterpret_cast<btRigidBody*>(hBody);
	btAssert(body);
	btCollisionShape* shape = body->getCollisionShape();
	btAssert(shape);

	btVector3 scaling(cscaling[0],cscaling[1],cscaling[2]);
	shape->setLocalScaling(scaling);	
}

btCollisionShapeHandle	BTNewBvhTriangleMeshShape(int num_tri,int* indices,int num_vertices,btReal* vertices)
{
	int vertStride = 3*sizeof(btReal);
	int indexStride = 3*sizeof(int);

	btTriangleIndexVertexArray* m_indexVertexArrays = new btTriangleIndexVertexArray(num_tri,
		indices,
		indexStride,
		num_vertices,vertices,vertStride);

	bool useQuantizedAabbCompression = true;

	btVector3 aabbMin(-1000,-1000,-1000),aabbMax(1000,1000,1000);
	
	void* mem = btAlignedAlloc(sizeof(btBvhTriangleMeshShape),16);

	btBvhTriangleMeshShape* shape  = new (mem)btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression,aabbMin,aabbMax);
	/*
	btTriangleMesh * mesh = new btTriangleMesh();
	mesh->addTriangle()
	*/
	return (btCollisionShapeHandle) shape;

}

void BTSetPosition(btRigidBodyHandle object, BTVector3& position)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	btVector3 pos(position[0],position[1],position[2]);
	btTransform worldTrans = body->getWorldTransform();
	worldTrans.setOrigin(pos);
	body->setWorldTransform(worldTrans);
	btCollisionShape* shape;
}

void BTSetOrientation(btRigidBodyHandle object, BTQuaternion& orientation)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	btQuaternion orn(orientation[0],orientation[1],orientation[2],orientation[3]);
	btTransform worldTrans = body->getWorldTransform();
	worldTrans.setRotation(orn);
	body->setWorldTransform(worldTrans);
}

void	BTSetMatrix(btRigidBodyHandle object, btReal* matrix)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	btTransform& worldTrans = body->getWorldTransform();
	worldTrans.setFromOpenGLMatrix(matrix);
}


void	BTSetOpenGLMatrix(btRigidBodyHandle object, btReal* matrix)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	btTransform& worldTrans = body->getWorldTransform();
	worldTrans.setFromOpenGLMatrix(matrix);
}

void BTTransform(btRigidBodyHandle object, BTQuaternion q, BTVector3 p)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);

	btSoftBody* soft = btSoftBody::upcast(body);
	if(soft){
		btQuaternion orn(q[0],q[1],q[2],q[3]);
		btVector3 pos(p[0],p[1],p[2]);
		btTransform t(orn,pos);
		soft->transform(t);
	}
}

void BTTranslate(btRigidBodyHandle object, btReal x, btReal y, btReal z)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);

	btSoftBody* soft = btSoftBody::upcast(body);
	if(soft)
		soft->translate(btVector3(x,y,z));
	else
		body->translate(btVector3(x,y,z));
}


void	BTGetOpenGLMatrix(btRigidBodyHandle object, btReal* matrix)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	body->getWorldTransform().getOpenGLMatrix(matrix);

}

void	BTGetPosition(btRigidBodyHandle object,BTVector3 position)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	const btVector3& pos = body->getWorldTransform().getOrigin();
	position[0] = pos.getX();
	position[1] = pos.getY();
	position[2] = pos.getZ();
}

void BTGetOrientation(btRigidBodyHandle object,BTQuaternion orientation)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	const btQuaternion& orn = body->getWorldTransform().getRotation();
	orientation[0] = orn.getX();
	orientation[1] = orn.getY();
	orientation[2] = orn.getZ();
	orientation[3] = orn.getW();
}


void BTGetMatrix(btRigidBodyHandle object,BTMatrix4 matrix)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	body->getWorldTransform().getOpenGLMatrix(&matrix[0]);
}

void BTGetConvexHullShapeDescription(btCollisionShapeHandle handle,btConvexHullDescription* desc){
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( handle);
	btAssert(shape);

	btAssert(shape->getShapeType()==CONVEX_HULL_SHAPE_PROXYTYPE);
	btConvexHullShape* convexHullShape = reinterpret_cast<btConvexHullShape*>( shape);
	desc->num_points = convexHullShape->getNumPoints();
	desc->num_planes = convexHullShape->getNumPlanes();
	//convexHullShape->getPoints

}

void BTFillConvexHullShapeDescription(btCollisionShapeHandle handle,btConvexHullDescription* desc){
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( handle);
	btAssert(shape);

	btAssert(shape->getShapeType()==CONVEX_HULL_SHAPE_PROXYTYPE);
	btConvexHullShape* convexHullShape = reinterpret_cast<btConvexHullShape*>( shape);
	btVector3 pnt;
	
	for(int i=0;i<desc->num_points;i++){
		convexHullShape->getVertex(i,pnt);
		/*
		desc->vertices[i*3] = pnt.getX();
		desc->vertices[i*3+1] = pnt.getY();
		desc->vertices[i*3+2] = pnt.getZ();
		*/
	}
	desc->num_edges = sizeof(desc->vertices);
}





/*
void BTDrawCollisionShape(btCollisionShapeHandle handle, BTMatrix4& modelview,  BTMatrix4& projection){
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( handle);
	btAssert(shape);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMultMatrixf(projection);

	//m_drawer.drawOpenGL(modelview, shape, btVector3(1,1,0),0,btVector3(-1000,-1000,-1000),btVector3(1000,1000,1000));

	glPopMatrix();

}*/

/*
int BTDrawConvexHullShape(btCollisionShapeHandle handle, BTMatrix4& model, BTMatrix4& view, BTMatrix4& projection){
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( handle);
	btAssert(shape);

	if(shape->getShapeType()==CONVEX_HULL_SHAPE_PROXYTYPE){
		btConvexHullShape* convexHullShape = reinterpret_cast<btConvexHullShape*>( shape);
		btVector3 pnt,start,end;

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glMultMatrixf(view);
		glMultMatrixf(model);
		

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glMultMatrixf(projection);

		glBegin(GL_POINTS);
		glPointSize(0.1);
		glColor3f(1,1,0);
		for(int i=0;i<convexHullShape->getNumPoints();i++){
			convexHullShape->getVertex(i,pnt);
			glVertex3f(pnt.getX(),pnt.getY(),pnt.getZ());
		}
		glEnd();

		if(convexHullShape->getNumEdges()){
			glBegin(GL_LINE_LOOP);
			for(int i=0;i<convexHullShape->getNumEdges();i++){
				convexHullShape->getEdge(i,start,end);
				//glVertex3f(start.getX(),start.getY(),start.getZ());
				glVertex3f(end.getX(),end.getY(),end.getZ());
			}
			glEnd();
		}
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		return convexHullShape->getNumEdges();
	}
	return 0;
}
*/
//plRigidBodyHandle plRayCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal);
//int BTRayCast(btDynamicsWorldHandle world, const BTVector3 &rayStart, const BTVector3 &rayEnd, btRayCastResult &res)
int	BTRayCast(btDynamicsWorldHandle world, const BTVector3 rayStart, const BTVector3 rayEnd, btRayCastResult& res)
{
	struct	AllRayResultCallback : public btCollisionWorld::RayResultCallback
	{ 
		const btCollisionShape* m_hitTriangleShape;
		int m_hitTriangleIndex;
		int m_hitShapePart;

		AllRayResultCallback(const btVector3&	rayFromWorld,const btVector3&	rayToWorld)
		:m_rayFromWorld(rayFromWorld),
		m_rayToWorld(rayToWorld),
		m_hitTriangleShape(NULL),
		m_hitTriangleIndex(0),
		m_hitShapePart(0){}

		btVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
		btVector3	m_rayToWorld;

		btVector3	m_hitNormalWorld;
		btVector3	m_hitPointWorld;
			
		virtual	btScalar	addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
		{
			//caller already does the filter on the m_closestHitFraction
			btAssert(rayResult.m_hitFraction <= m_closestHitFraction);
			
			m_closestHitFraction = rayResult.m_hitFraction;
			m_collisionObject = rayResult.m_collisionObject;
			if (normalInWorldSpace)
			{
				m_hitNormalWorld = rayResult.m_hitNormalLocal;
			} else
			{
				///need to transform normal into worldspace
				m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
			}
			if(rayResult.m_localShapeInfo){
				m_hitTriangleShape = rayResult.m_collisionObject->getCollisionShape();
				m_hitTriangleIndex = rayResult.m_localShapeInfo->m_triangleIndex;
				m_hitShapePart = rayResult.m_localShapeInfo->m_shapePart;
			}
			else{
				m_hitTriangleShape = NULL;
				m_hitTriangleIndex = 0;
				m_hitShapePart = 0;
			}

			m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
			return rayResult.m_hitFraction;
		}
	};

	btDynamicsWorld *dynamicsWorld = reinterpret_cast<btDynamicsWorld *>(world);
	btVector3 start(rayStart[0],rayStart[1],rayStart[2]);
	btVector3 end(rayEnd[0],rayEnd[1],rayEnd[2]);
	//btCollisionWorld::ClosestRayResultCallback resultCallback(start,end);
	AllRayResultCallback resultCallback(start,end);
	dynamicsWorld->rayTest(start,end,resultCallback);
	if (resultCallback.hasHit())
	{
		const btCollisionObject* object = resultCallback.m_collisionObject;
		const btCollisionShape* shape = object->getCollisionShape();
		res.m_shape = (btCollisionShapeHandle)shape;
		res.m_body = (btRigidBodyHandle)object;

		res.m_triangleIndex = resultCallback.m_hitTriangleIndex;


		memcpy(res.m_pointWorld, &(resultCallback.m_hitPointWorld),  sizeof(btReal) * 3);
		memcpy(res.m_normalWorld,   &(resultCallback.m_hitNormalWorld), sizeof(btReal) * 3);
		return 1;
	}
	
	return 7; // no results
}

//	 plRigidBodyHandle plObjectCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal);

double BTNearestPoints(float p1[3], float p2[3], float p3[3], float q1[3], float q2[3], float q3[3], float *pa, float *pb, float normal[3])
{
	btVector3 vp(p1[0], p1[1], p1[2]);
	btTriangleShape trishapeA(vp, 
				  btVector3(p2[0], p2[1], p2[2]), 
				  btVector3(p3[0], p3[1], p3[2]));
	trishapeA.setMargin(0.000001f);
	btVector3 vq(q1[0], q1[1], q1[2]);
	btTriangleShape trishapeB(vq, 
				  btVector3(q2[0], q2[1], q2[2]), 
				  btVector3(q3[0], q3[1], q3[2]));
	trishapeB.setMargin(0.000001f);
	
	// btVoronoiSimplexSolver sGjkSimplexSolver;
	// btGjkEpaPenetrationDepthSolver penSolverPtr;	
	
	static btSimplexSolverInterface sGjkSimplexSolver;
	sGjkSimplexSolver.reset();
	
	static btGjkEpaPenetrationDepthSolver Solver0;
	static btMinkowskiPenetrationDepthSolver Solver1;
		
	btConvexPenetrationDepthSolver* Solver = NULL;
	
	Solver = &Solver1;	
		
	btGjkPairDetector convexConvex(&trishapeA ,&trishapeB,&sGjkSimplexSolver,Solver);
	
	convexConvex.m_catchDegeneracies = 1;
	
	// btGjkPairDetector convexConvex(&trishapeA ,&trishapeB,&sGjkSimplexSolver,0);
	
	btPointCollector gjkOutput;
	btGjkPairDetector::ClosestPointInput input;
	
		
	btTransform tr;
	tr.setIdentity();
	
	input.m_transformA = tr;
	input.m_transformB = tr;
	
	convexConvex.getClosestPoints(input, gjkOutput, 0);
	
	
	if (gjkOutput.m_hasResult)
	{
		
		pb[0] = pa[0] = gjkOutput.m_pointInWorld[0];
		pb[1] = pa[1] = gjkOutput.m_pointInWorld[1];
		pb[2] = pa[2] = gjkOutput.m_pointInWorld[2];

		pb[0]+= gjkOutput.m_normalOnBInWorld[0] * gjkOutput.m_distance;
		pb[1]+= gjkOutput.m_normalOnBInWorld[1] * gjkOutput.m_distance;
		pb[2]+= gjkOutput.m_normalOnBInWorld[2] * gjkOutput.m_distance;
		
		normal[0] = gjkOutput.m_normalOnBInWorld[0];
		normal[1] = gjkOutput.m_normalOnBInWorld[1];
		normal[2] = gjkOutput.m_normalOnBInWorld[2];

		return gjkOutput.m_distance;
	}
	return -1.0f;	
}

//--------------------------------------------- SOFT BODIES TEST

void ResetScene(){
/*
	m_azi = 0;
	m_cameraDistance = 30.f;
	m_cameraTargetPosition.setValue(0,0,0);

	DemoApplication::clientResetScene();
	// Clean up
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>=0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
		while(m_dynamicsWorld->getNumConstraints())
		{
			btTypedConstraint*	pc=m_dynamicsWorld->getConstraint(0);
			m_dynamicsWorld->removeConstraint(pc);
			delete pc;
		}
		btSoftBody* softBody = btSoftBody::upcast(obj);
		if (softBody)
		{
			getSoftDynamicsWorld()->removeSoftBody(softBody);
		} else
		{
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body)
				m_dynamicsWorld->removeRigidBody(body);
			else
				m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}

	//create ground object
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-12,0));

	btCollisionObject* newOb = new btCollisionObject();
	newOb->setWorldTransform(tr);
	newOb->setInterpolationWorldTransform( tr);
	m_dynamicsWorld->addCollisionObject(newOb);

	m_softBodyWorldInfo.m_sparsesdf.Reset();
	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
	m_softBodyWorldInfo.water_density	=	0;
	m_softBodyWorldInfo.water_offset		=	0;
	m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);
	*/
}

btDynamicsWorldHandle BTSoftBodyWorld(){
	///create concave ground mesh

	
	float m_azi = 0;

	btCollisionShape* groundShape = 0;
	{
		int i;
		int j;

		const int NUM_VERTS_X = 30;
		const int NUM_VERTS_Y = 30;
		const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
		const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

		btVector3* gGroundVertices = new btVector3[totalVerts];
		int* gGroundIndices = new int[totalTriangles*3];

		btScalar offset(-50);

		for ( i=0;i<NUM_VERTS_X;i++)
		{
			for (j=0;j<NUM_VERTS_Y;j++)
			{
				gGroundVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
					//0.f,
					waveheight*sinf((float)i)*cosf((float)j+offset),
					(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
			}
		}

		int vertStride = sizeof(btVector3);
		int indexStride = 3*sizeof(int);

		int index=0;
		for ( i=0;i<NUM_VERTS_X-1;i++)
		{
			for (int j=0;j<NUM_VERTS_Y-1;j++)
			{
				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = j*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i;
			}
		}

		btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
			gGroundIndices,
			indexStride,
			totalVerts,(btScalar*) &gGroundVertices[0].x(),vertStride);

		bool useQuantizedAabbCompression = true;

		groundShape = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression);
		groundShape->setMargin(0.5);
	}
	btAlignedObjectArray<btCollisionShape*>		m_collisionShapes;
	m_collisionShapes.push_back(groundShape);

	btCollisionShape* groundBox = new btBoxShape (btVector3(100,CUBE_HALF_EXTENTS,100));
	m_collisionShapes.push_back(groundBox);

	btCompoundShape* cylinderCompound = new btCompoundShape;
	btCollisionShape* cylinderShape = new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
	btTransform localTransform;
	localTransform.setIdentity();
	cylinderCompound->addChildShape(localTransform,cylinderShape);
	btQuaternion orn(btVector3(0,1,0),SIMD_PI);
	localTransform.setRotation(orn);
	cylinderCompound->addChildShape(localTransform,cylinderShape);

	m_collisionShapes.push_back(cylinderCompound);

	btCollisionDispatcher* m_dispatcher=0;
	btSoftBodyWorldInfo m_softBodyWorldInfo;
	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	btSoftBodyRigidBodyCollisionConfiguration* m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();


	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	btBroadphaseInterface* m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	btSequentialImpulseConstraintSolver* m_solver = new btSequentialImpulseConstraintSolver();

	btSoftBodySolver* softBodySolver = new btDefaultSoftBodySolver();

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration,softBodySolver);
	//world->setInternalTickCallback(pickingPreTickCallback,world,true);


	world->getDispatchInfo().m_enableSPU = false;
	world->setGravity(btVector3(0,-1,0));
	m_softBodyWorldInfo.m_gravity.setValue(0,-1,0);

	m_softBodyWorldInfo.m_sparsesdf.Initialize();

	btVector3 p(0.0,0.0,0.0);
	btVector3 s(1.0,1.0,1.0);

	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-12,0));

	btCollisionObject* newOb = new btCollisionObject();
	newOb->setWorldTransform(tr);
	newOb->setInterpolationWorldTransform( tr);

	newOb->setCollisionShape(m_collisionShapes[0]);

	world->addCollisionObject(newOb);

	m_softBodyWorldInfo.m_sparsesdf.Reset();

	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
	m_softBodyWorldInfo.water_density	=	0;
	m_softBodyWorldInfo.water_offset		=	0;
	m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

	btAlignedObjectArray<btVector3>	pts;
	int id = 666;
	int np = 33;
	if(id) srand(id);
	for(int i=0;i<np;++i)
	{
		pts.push_back(Vector3Rand()*s+p);
	}
	
	btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(m_softBodyWorldInfo,&pts[0],pts.size());
	psb->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.4;
	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2,pm);
	psb->setTotalMass(150);

	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)world;
	
	softWorld->addSoftBody(psb);
	
	return (btDynamicsWorldHandle)softWorld;
}




