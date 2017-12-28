//--------------------------------------------------
// INTERSECTOR IMPLEMENTATION
//--------------------------------------------------

#include "intersector.h"
namespace BOB{
// Compute barycentric coordinates (u, v, w) for
// point p with respect to triangle (a, b, c)
void Intersector::barycentric(Vector3& p, Vector3& a, Vector3& b, Vector3& c, float &u, float &v, float &w)
{
    Vector3 v0 = b - a;
    Vector3 v1 = c - a;
    Vector3 v2 = p - a;
    float d00 = v0*v0;
    float d01 = v0*v1;
    float d11 = v1*v1;
    float d20 = v2*v0;
    float d21 = v2*v1;
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

bool Intersector::create(const float* positions, unsigned num_points, const unsigned* tris, unsigned num_tris)
{

    octree.clearTree();
    triangles.resize(num_tris);
    for(unsigned k=0;k<num_tris;k++)
    {

        triangles[k].ID = k;
        triangles[k].mapID = k;
        triangles[k].vertices[0] = tris[k*3];
        triangles[k].vertices[1] = tris[k*3+1];
        triangles[k].vertices[2] = tris[k*3+2];
        
    }
    
    int vertices[num_points];
    for( unsigned i = 0; i < num_points; ++i ) vertices[i]=i;
    
    octree.getBoundingBox(positions, vertices, num_points);
    octree.buildTree(triangles, positions);
    return true;
}

bool Intersector::create(const float* positions, unsigned num_points, const unsigned* tris, const unsigned* map_indices, unsigned num_tris)
{
    octree.clearTree();
    triangles.resize(num_tris);
    for(unsigned k=0;k<num_tris;k++)
    {
        
        triangles[k].ID = k;
        triangles[k].mapID = k;
        triangles[k].vertices[0] = tris[k*3];
        triangles[k].vertices[1] = tris[k*3+1];
        triangles[k].vertices[2] = tris[k*3+2];
        
    }
    
    int vertices[num_points];
    for( unsigned i = 0; i < num_points; ++i ) vertices[i]=i;
    
    octree.getBoundingBox(positions, vertices, num_points);
    octree.buildTree(triangles, positions);
    return true;
}

void Intersector::recurseGetClosestCell(const Vector3& point, Octree* cell, float& closestDistance, Octree*& closestCell)
{
    if(cell==NULL)return;
	float distance;
	if(!cell->isLeaf())
	{
		for(unsigned k=0;k<8;k++)
		{
            Octree* current = cell->getCell(k);
            
            if(current!=NULL)
            {
            	distance = current->getDistance(point);
            	if(distance<=closestDistance)
                {
                    recurseGetClosestCell(point, current, closestDistance, closestCell);
                }
            }
		}
	}
	else
	{
		distance = cell->getDistance(point);
        
		if(distance<closestDistance)
		{
			closestDistance = distance;
			closestCell = cell;
		}
	}
}

void Intersector::getClosestCell(const Vector3& point, Octree*& closestCell)
{
	float closestDistance = FLT_MAX;
    
    // the case of low polygon count
    if(octree.isLeaf())
    {
        closestCell = &octree;
    }
    
    // normal case
    else
    {
    	for(unsigned j=0;j<8;j++)
    	{
    		recurseGetClosestCell(point, octree.getCell(j), closestDistance, closestCell);
    	}
    }
}

void Intersector::recurseGetNearbyCells(Octree* cell, const Vector3& center, const float radius, std::vector<Octree*>& cells)
{
    if(cell==NULL)return;
    if(!cell->isLeaf())
    {
        for(unsigned k=0;k<8;k++)
        {
        	recurseGetNearbyCells(cell->getCell(k), center, radius, cells);
        }
    }
    else
    {
        if(cell->intersectSphere(center, radius))
        {
            cells.push_back(cell);
        }
    }
}

void Intersector::getNearbyCells(const Vector3& point, Octree* cell, std::vector<Octree*>& cells)
{
    Vector3 furthestCorner;
    cell->getFurthestCorner(point, furthestCorner);
    
    float radius = (float)((furthestCorner - point).length());
    
    // the case of low polygon count
    if(!octree.isLeaf())
    {
        for(unsigned j=0;j<8;j++)
        {
            recurseGetNearbyCells(octree.getCell(j), point, radius, cells);
        }
    }
}

void Intersector::getClosestPoint(const Vector3& queryPoint, ClosestPoint& closestPoint)
{
    Octree* closestCell = NULL;
    getClosestCell(queryPoint, closestCell);

    std::vector<Octree*> nearbyCells;
    if(!octree.isLeaf())
    {
    	getNearbyCells(queryPoint, closestCell, nearbyCells);
    }
 
    // brute force neighbor cell
    std::vector<Triangle*>::iterator tri = closestCell->getTriangles().begin();
    float closestDistance = FLT_MAX;
    float distance;
    float currentU, currentV, currentW;
    Vector3 closest;
    Vector3 delta;
    Triangle* closestTriangle = NULL;
    for(;tri< closestCell->getTriangles().end();tri++)
    {
        (*tri)->closestPoint( positions, queryPoint , closest, currentU, currentV, currentW);
        delta = queryPoint-closest;
        distance = delta.length();
        
        if(distance<closestDistance)
        {
            closestDistance = distance;
            closestPoint.position = closest;
            closestTriangle = *tri;
            closestPoint.U = currentU;
            closestPoint.V = currentV;
            closestPoint.W = currentW;
        }
    }
    
    // loop nearby cells
    std::vector<Octree*>::iterator nearby = nearbyCells.begin();
    for(;nearby<nearbyCells.end();nearby++)
    {
        tri = (*nearby)->getTriangles().begin();
        for(;tri< (*nearby)->getTriangles().end();tri++)
        {
            (*tri)->closestPoint( positions, queryPoint , closest, currentU, currentV, currentW);
            delta = queryPoint-closest;
            distance = delta.length();
            
            if(distance<closestDistance)
            {
                closestDistance = distance;
                closestPoint.position = closest;
                closestTriangle = *tri;
                closestPoint.U = currentU;
                closestPoint.V = currentV;
                closestPoint.W = currentW;
            }
        }
    }
    
    closestPoint.triangleID = closestTriangle->ID;
    closestPoint.triangleMapID = closestTriangle->mapID;
    closestTriangle->getNormal(positions, closestPoint.normal);
}

void Intersector::getClosestPointBruteForce(const Vector3& queryPoint, ClosestPoint& closestPoint)
{
    std::vector<Triangle>::iterator tri = triangles.begin();
    float closestDistance = FLT_MAX;
    float distance;
    Vector3 closest;
    Vector3 delta;
    float currentU, currentV, currentW;
    Triangle* closestTriangle = NULL;
    for(;tri<triangles.end();tri++)
    {
        (*tri).closestPoint( positions, queryPoint , closest, currentU, currentV, currentW);
        delta = queryPoint-closest;
        distance = delta.length();
        if(distance<=closestDistance)
        {
            closestDistance = distance;
            closestPoint.position = closest;
            closestTriangle = &(*tri);
            closestPoint.U = currentU;
            closestPoint.V = currentV;
            closestPoint.W = currentW;
        }
    }
    
    closestPoint.triangleID = closestTriangle->ID;
    closestTriangle->getNormal(positions, closestPoint.normal);

}
}//end namespace BOB
