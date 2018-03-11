#include "ConvexDecomposition.h"

int BTNewConvexDecompositionShape(int num_tri,int* tri, int num_vertices, btReal* vertices)
{
	//-----------------------------------
	// Bullet Convex Decomposition
	//-----------------------------------
	
	unsigned int depth = 5;
	float cpercent     = 5;
	float ppercent     = 15;
	unsigned int maxv  = 16;
	float skinWidth    = 0.0;

	//printf("WavefrontObj num triangles read %i\n",num_tri);
	ConvexDecomposition::DecompDesc desc;
	desc.mVcount       = num_vertices;
	desc.mVertices     = (float*)vertices;
	desc.mTcount       = num_tri;
	desc.mIndices      = (unsigned int *)tri;
	desc.mDepth        = depth;
	desc.mCpercent     = cpercent;
	desc.mPpercent     = ppercent;
	desc.mMaxVertices  = maxv;
	desc.mSkinWidth    = skinWidth;

	MyConvexDecomposition	convexDecomposition(NULL);
	desc.mCallback = &convexDecomposition;

	//-----------------------------------------------
	// HACD
	//-----------------------------------------------

	std::vector< HACD::Vec3<HACD::Real> > points;
	std::vector< HACD::Vec3<long> > triangles;
	
	for(int i=0; i<num_vertices; i++ ) 
	{
		int index = i*3;
		HACD::Vec3<HACD::Real> vertex(vertices[index],vertices[index+1],vertices[index+2]);
		points.push_back(vertex);
	}

	for(int i=0;i<num_tri;i++)
	{
		int index = i*3;
		HACD::Vec3<long> triangle(tri[index], tri[index+1], tri[index+2]);
		triangles.push_back(triangle);
	}
	
	HACD::HACD myHACD;
	myHACD.SetPoints(&points[0]);
	myHACD.SetNPoints(points.size());
	myHACD.SetTriangles(&triangles[0]);
	myHACD.SetNTriangles(triangles.size());
	myHACD.SetCompacityWeight(0.1);
	myHACD.SetVolumeWeight(0.0);

	// HACD parameters
	// Recommended parameters: 2 100 0 0 0 0
	size_t nClusters = 2;
	double concavity = 100;
	bool invert = false;
	bool addExtraDistPoints = false;
	bool addNeighboursDistPoints = false;
	bool addFacesPoints = false;       

	myHACD.SetNClusters(nClusters);                     // minimum number of clusters
	myHACD.SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
	myHACD.SetConcavity(concavity);                     // maximum concavity
	myHACD.SetAddExtraDistPoints(addExtraDistPoints);   
	myHACD.SetAddNeighboursDistPoints(addNeighboursDistPoints);   
	myHACD.SetAddFacesPoints(addFacesPoints); 
	
	myHACD.Compute();
	nClusters = myHACD.GetNClusters();	
	return nClusters;
	/*
	//myHACD.Save("output.wrl", false);


	//convexDecomposition.performConvexDecomposition(desc);

//		ConvexBuilder cb(desc.mCallback);
//		cb.process(desc);
	//now create some bodies
	
	if (1)
	{
		void* mem = btAlignedAlloc(sizeof(btCompoundShape),16);
		btCompoundShape* compound = new (mem)btCompoundShape();
		//m_collisionShapes.push_back (compound);

		btTransform trans;
		trans.setIdentity();

		for (int c=0;c<nClusters;c++)
		{
			//generate convex result
			size_t nPoints = myHACD.GetNPointsCH(c);
			size_t nTriangles = myHACD.GetNTrianglesCH(c);

			float* vertices = new float[nPoints*3];
			unsigned int* triangles = new unsigned int[nTriangles*3];
			
			HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
			HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
			myHACD.GetCH(c, pointsCH, trianglesCH);

			// points
			for(size_t v = 0; v < nPoints; v++)
			{
				vertices[3*v] = pointsCH[v].X();
				vertices[3*v+1] = pointsCH[v].Y();
				vertices[3*v+2] = pointsCH[v].Z();
			}
			// triangles
			for(size_t f = 0; f < nTriangles; f++)
			{
				triangles[3*f] = trianglesCH[f].X();
				triangles[3*f+1] = trianglesCH[f].Y();
				triangles[3*f+2] = trianglesCH[f].Z();
			}

			delete [] pointsCH;
			delete [] trianglesCH;

			ConvexResult r(nPoints, vertices, nTriangles, triangles);
			convexDecomposition.ConvexDecompResult(r);
		}

		for (int i=0;i<convexDecomposition.m_convexShapes.size();i++)
		{
			btVector3 centroid = convexDecomposition.m_convexCentroids[i];
			trans.setOrigin(centroid);
			btConvexHullShape* convexShape = convexDecomposition.m_convexShapes[i];
			compound->addChildShape(trans,convexShape);

			
			btRigidBody* body;
			body = localCreateRigidBody( 1.0, trans,convexShape);
			
		}
	
		return (btCollisionShapeHandle) compound;
		for (int i=0;i<convexDecomposition.m_convexShapes.size();i++)
		{
			
			btVector3 centroid = convexDecomposition.m_convexCentroids[i];
			trans.setOrigin(centroid);
			btConvexHullShape* convexShape = convexDecomposition.m_convexShapes[i];
			compound->addChildShape(trans,convexShape);

			btRigidBody* body;
			body = localCreateRigidBody( 1.0, trans,convexShape);
		}

#if 1
		btScalar mass=10.f;
		trans.setOrigin(-convexDecompositionObjectOffset);
		btRigidBody* body = localCreateRigidBody( mass, trans,compound);
		body->setCollisionFlags(body->getCollisionFlags() |   btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

		convexDecompositionObjectOffset.setZ(6);
		trans.setOrigin(-convexDecompositionObjectOffset);
		body = localCreateRigidBody( mass, trans,compound);
		body->setCollisionFlags(body->getCollisionFlags() |   btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

		convexDecompositionObjectOffset.setZ(-6);
		trans.setOrigin(-convexDecompositionObjectOffset);
		body = localCreateRigidBody( mass, trans,compound);
		body->setCollisionFlags(body->getCollisionFlags() |   btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
#endif
		*/
	
}