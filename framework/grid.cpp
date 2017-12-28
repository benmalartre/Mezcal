//-------------------------------------------------------
// GRID IMPLEMENTATION
//-------------------------------------------------------
#include "grid.h"
#include <iostream>
namespace BOB{
Grid::Grid(){};
Grid::Grid(Vector3& bmin, Vector3& bmax, int x, int y, int z)
{
    min = bmin;
    max = bmax;
    nx = x;
    ny = y;
    nz = z;
}

Grid::~Grid()
{
    points.clear();
    cells.clear();
}

void Grid::setResolution(int x, int y, int z)
{
    nx = (x<2) ? 2 : x;
    ny = (y<2) ? 2 : y;
    nz = (z<2) ? 2 : z;
}

void Grid::setBoundingBox(Vector3& corner1, Vector3& corner2)
{
    min = corner1;
    max = corner2;
}

void Grid::create()
{
    points.clear();
    cells.clear();
    
    numPoints = nx*ny*nz;
    numCells = (nx-1)*(ny-1)*(nz-1);
    points.resize(numPoints);
    cells.resize(numCells);
    
    unsigned pid = 0;
    unsigned cid = 0;
    unsigned base = 0;
    unsigned nRow = nx;
    unsigned nSlice = nx*ny;

    Point* P;
    Cell* C;
    float sx, sy, sz;
    sx = (max.x - min.x)/(nx-1);
    sy = (max.y - min.y)/(ny-1);
    sz = (max.z - min.z)/(nz-1);
    
    // setup points
    for(unsigned iz=0;iz<nz;iz++)
    {
        for(unsigned iy=0;iy<ny;iy++)
        {
            for(unsigned ix=0;ix<nx;ix++)
            {
                P = &points[pid];
                P->ID = pid;
                P->position.x = min.x + sx * ix;
                P->position.y = min.y + sy * iy;
                P->position.z = min.z + sz * iz;
                
                P->up.x = 0.0f;
                P->up.y = 1.0f;
                P->up.z = 0.0f;
                
                P->tangent.x = 1.0f;
                P->tangent.y = 0.0f;
                P->tangent.z = 0.0f;
                
                P->border = 0;
                
                // left
                if(ix==0) BIT_SET(P->border, 0);
                //right
                if(ix==nx-1) BIT_SET(P->border, 1);
                // bottom
                if(iy==0) BIT_SET(P->border, 2);
                // top
                if(iy==ny-1) BIT_SET(P->border, 3);
                // back
                if(iz==0) BIT_SET(P->border, 4);
                // front
                if(iz==nz-1) BIT_SET(P->border, 5);
                pid++;
            }
        }
    }
    
    unsigned flip = 0;
    // setup cells
    for(unsigned iz=0;iz<nz;iz++)
    {
        for(unsigned iy=0;iy<ny;iy++)
        {
            for(unsigned ix=0;ix<nx;ix++)
            {
                if(ix<(nx-1) && iy<(ny-1) && iz<(nz-1))
                {
                    C = &cells[cid];
                    C->ID = cid;
                    base = iz*nSlice+iy*nRow+ix;
    
                    C->corners[0] = &points[base];
                    C->corners[1] = &points[base+1];
                    C->corners[2] = &points[base+nRow];
                    C->corners[3] = &points[base+nRow+1];
                    C->corners[4] = &points[base+nSlice];
                    C->corners[5] = &points[base+nSlice+1];
                    C->corners[6] = &points[base+nSlice+nRow];
                    C->corners[7] = &points[base+nSlice+nRow+1];
                    
                    C->border = 0;
                    for(unsigned kk=0;kk<8;kk++)
                    {
                        for(unsigned jj=0;jj<6;jj++)
                            if(BIT_CHECK(C->corners[kk]->border, jj))BIT_SET(C->border, jj);
                    }
                    //flip = (iz%2)*4 + (iy%2)*2 + ix%2;
                    if(iz%2==0) flip = (iy+ix)%2;
                    else flip = (iy+ix+1)%2;
                    C->flip = flip;
                    cid++;
                }
            }
        }
    }
    
    // setup tets
    for(unsigned ix=0; ix<tets.size();ix++)delete tets[ix];
    unsigned numTets = numCells*5;
    tets.resize(numTets);
    //Tetrahedron* T;
    unsigned ct = 0;
    
    for(unsigned ix=0;ix<numCells;ix++)
    {
        // five tetrahedrons
        C = &cells[ix];
        switch(C->flip%2)
        {
            case 0:
                tets[ct+0] = new Tetrahedron(C->corners[0], C->corners[1], C->corners[2], C->corners[4]);
                tets[ct+1] = new Tetrahedron(C->corners[1], C->corners[3], C->corners[2], C->corners[7]);
                tets[ct+2] = new Tetrahedron(C->corners[2], C->corners[6], C->corners[4], C->corners[7]);
                tets[ct+3] = new Tetrahedron(C->corners[4], C->corners[5], C->corners[7], C->corners[1]);
                tets[ct+4] = new Tetrahedron(C->corners[2], C->corners[4], C->corners[1], C->corners[7]);
                break;
            case 1:
                tets[ct+0] = new Tetrahedron(C->corners[0], C->corners[1], C->corners[3], C->corners[5]);
                tets[ct+1] = new Tetrahedron(C->corners[2], C->corners[0], C->corners[3], C->corners[6]);
                tets[ct+2] = new Tetrahedron(C->corners[7], C->corners[4], C->corners[3], C->corners[6]);
                tets[ct+3] = new Tetrahedron(C->corners[4], C->corners[5], C->corners[6], C->corners[0]);
                tets[ct+4] = new Tetrahedron(C->corners[0], C->corners[3], C->corners[6], C->corners[5]);
                break;
        }
        C->tets[0] = tets[ct+0];
        C->tets[1] = tets[ct+1];
        C->tets[2] = tets[ct+2];
        C->tets[3] = tets[ct+3];
        C->tets[4] = tets[ct+4];
        ct+=5;
    }
}

void Grid::filter()
{
    
}

unsigned Grid::size()
{
    unsigned sizePoints = points.size()*sizeof(Point);
    unsigned sizeCells = cells.size()*sizeof(Cell);
    unsigned sizeTets = tets.size()*sizeof(Tetrahedron);
    std::cout << "SIZE POINTS : "<< sizePoints << std::endl;
    std::cout << "SIZE CELLS : "<< sizeCells << std::endl;
    std::cout << "SIZE TETS : "<< sizeTets << std::endl;
    /*
    MGlobal::displayInfo("Size Points : "+MString()+(sizePoints/1000)+" MBytes");
    MGlobal::displayInfo("Size Cells : "+MString()+(sizeCells/1000)+" MBytes");
    MGlobal::displayInfo("Size Tets : "+MString()+(sizeTets/1000)+" MBytes");
    
    MGlobal::displayInfo("Total Size : "+MString()+((sizePoints+sizeCells+sizeTets)/1000)+" MBytes");
     */
    return 666;
}

#ifdef USE_GLDRAWER
/*Grid::draw()
{
 
}*/
#endif
}// end namespace BOB
/*
//-------------------------------------------------------
// Compute AABB from Mesh
//-------------------------------------------------------
void AABB::compute(const MFnMesh& mesh)
{
	MStatus status;
	_min.x = FLT_MAX;
	_min.y = FLT_MAX;
	_min.z = FLT_MAX;
    _min.w = 1.0f;
	_max.x = -FLT_MAX;
	_max.y = -FLT_MAX;
	_max.z = -FLT_MAX;
    _max.w = 1.0f;

	int nbv = mesh.numVertices(&status);
	const float* pos = mesh.getRawPoints(&status);

	for(int i=0;i<nbv;i++)
	{
		if(pos[i*3]<_min.x)_min.x = pos[i*3];
		if(pos[i*3+1]<_min.y)_min.y = pos[i*3+1];
		if(pos[i*3+2]<_min.z)_min.z = pos[i*3+2];

		if(pos[i*3]>_max.x)_max.x = pos[i*3];
		if(pos[i*3+1]>_max.y)_max.y = pos[i*3+1];
		if(pos[i*3+2]>_max.z)_max.z = pos[i*3+2];
	}
}

//-------------------------------------------------------
// Compute AABB from Mesh and Vertex Indices Array
//-------------------------------------------------------
void AABB::compute( const MFnMesh& mesh, const MIntArray& vertices)
{
	MStatus status;
	_min.x = FLT_MAX;
	_min.y = FLT_MAX;
	_min.z = FLT_MAX;
    _min.w = 1.0;
	_max.x = -FLT_MAX;
	_max.y = -FLT_MAX;
	_max.z = -FLT_MAX;
    _max.w = 1.0f;

	const float* pos = mesh.getRawPoints(&status);

	int j=0;
	for(int i=0;i<vertices.length();i++)
	{
		j = vertices[i];
		if(pos[j*3]<_min.x)_min.x = pos[j*3];
		if(pos[j*3+1]<_min.y)_min.y = pos[j*3+1];
		if(pos[j*3+2]<_min.z)_min.z = pos[j*3+2];

		if(pos[i*3]>_max.x)_max.x = pos[i*3];
		if(pos[i*3+1]>_max.y)_max.y = pos[i*3+1];
		if(pos[i*3+2]>_max.z)_max.z = pos[i*3+2];
	}
}

//-------------------------------------------------------
// Compute AABB from Vertex Array
//-------------------------------------------------------
void AABB::compute(std::vector<Vertex*>& vertices)
{
    MStatus status;
    _min.x = FLT_MAX;
    _min.y = FLT_MAX;
    _min.z = FLT_MAX;
    _min.w = 1.0;
    _max.x = -FLT_MAX;
    _max.y = -FLT_MAX;
    _max.z = -FLT_MAX;
    _max.w = 1.0f;
    
    MVector pos;
    MVector norm;
    std::vector<Vertex*>::iterator it = vertices.begin();
    for(;it<vertices.end();it++)
    {
        pos = (MVector)(*it)->position;
        norm = (*it)->normal;
        
        if(pos.x<_min.x)_min.x = pos.x;
        if(pos.y<_min.y)_min.y = pos.y;
        if(pos.z<_min.z)_min.z = pos.z;
        
        if(pos.x>_max.x)_max.x = pos.x;
        if(pos.y>_max.y)_max.y = pos.y;
        if(pos.z>_max.z)_max.z = pos.z;
    }
}

//-------------------------------------------------------
// Compute AABB from Vertex Array and Vertex Indices Array
//-------------------------------------------------------
void AABB::compute(const std::vector<Vertex*>& vertices, const MIntArray& indices)
{
    MStatus status;
    _min.x = FLT_MAX;
    _min.y = FLT_MAX;
    _min.z = FLT_MAX;
    _min.w = 1.0;
    _max.x = -FLT_MAX;
    _max.y = -FLT_MAX;
    _max.z = -FLT_MAX;
    _max.w = 1.0f;
    
    int j=0;
    MVector pos;
    MVector norm;
    for(int i=0;i<indices.length();i++)
    {
        j = indices[i];

        pos = (MVector)vertices[j]->position;
        norm = vertices[j]->normal;
        
        if(pos.x<_min.x)_min.x = pos.x;
        if(pos.y<_min.y)_min.y = pos.y;
        if(pos.z<_min.z)_min.z = pos.z;
        
        if(pos.x>_max.x)_max.x = pos.x;
        if(pos.y>_max.y)_max.y = pos.y;
        if(pos.z>_max.z)_max.z = pos.z;
    }
}

//-------------------------------------------------------
// MVector Is Inside
//-------------------------------------------------------
bool AABB::isInside(const MVector& pos)
{
	bool inside = pos.x >= _min.x &&
				pos.x <= _max.x &&
				pos.y >= _min.y &&
				pos.y <= _max.y &&
				pos.z >= _min.z &&
				pos.z <= _max.z;
	return inside;
}

//-------------------------------------------------------
// MPoint Is Inside
//-------------------------------------------------------
bool AABB::isInside(const MPoint& pos)
{
	bool inside = pos.x >= _min.x &&
				pos.x <= _max.x &&
				pos.y >= _min.y &&
				pos.y <= _max.y &&
				pos.z >= _min.z &&
				pos.z <= _max.z;
	return inside;
}

//-------------------------------------------------------
// AABB Intersect Other
//-------------------------------------------------------
bool AABB::intersect(const AABB& other)
{
	bool intersect = _min.x < other._max.x &&
					_max.x > other._min.x &&
					_min.y < other._max.y &&
					_max.y > other._min.y &&
					_min.z < other._max.z &&
					_max.z > other._min.z;
	return intersect;
}

//-------------------------------------------------------
// AABB Transform
// Only Transform the 8 corners of the Bounding Box
// Should be faster but not really accurate
//-------------------------------------------------------
void AABB::transform(const MMatrix& M)
{

    float P[8] = {
    			(float)_min.x,
    			(float)_min.y,
				(float)_min.z,
				(float)1.0f,
				(float)_max.x,
				(float)_max.y,
				(float)_max.z,1.0f
    };

    int permutation[24] = {0,1,2,4,1,2,0,1,6,4,1,6,
    						0,5,2,4,5,2,0,5,6,4,5,6};
    
    _min.x = FLT_MAX;
    _min.y = FLT_MAX;
    _min.z = FLT_MAX;
    _min.w = 1.0;
    _max.x = -FLT_MAX;
    _max.y = -FLT_MAX;
    _max.z = -FLT_MAX;
    _max.w = 1.0f;


    for(int i=0;i<8;i++)
    {
        MPoint T(P[permutation[i*3]],P[permutation[i*3+1]],P[permutation[i*3+2]],1.0f);
        T*=M;
        if(T.x<_min.x)_min.x = T.x;
        if(T.y<_min.y)_min.y = T.y;
        if(T.z<_min.z)_min.z = T.z;
        
        if(T.x>_max.x)_max.x = T.x;
        if(T.y>_max.y)_max.y = T.y;
        if(T.z>_max.z)_max.z = T.z;
    }
}

*/

