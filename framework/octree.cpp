//======================================================
// OCTREE IMPLEMENTATION
//======================================================
#include "octree.h"

namespace BOB{
const int Octree::MAX_TRIANGLES_NUMBER = 128;

// destructor
Octree::~Octree()
{
    for (int i=0; i<8; i++)
    {
        if (_child[i])delete _child[i];
        _child[i] = NULL;
    }
    if(_triangles.size())_triangles.clear();
}

// clear tree
void Octree::clearTree()
{
    for (int i=0; i<8; i++)
    {
        if (_child[i])delete _child[i];
        _child[i] = NULL;
    }
    if(_triangles.size())_triangles.clear();
}

// get distance
float Octree::getDistance(const Vector3 &point)
{
    float dx = getDistance1D(point.x, _min.x, _max.x);
    float dy = getDistance1D(point.y, _min.y, _max.y);
    float dz = getDistance1D(point.z, _min.z, _max.z);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// intersect sphere
bool Octree::intersectSphere(const Vector3& center, const float radius)
{
    float r2 = radius * radius;
    float dmin = 0;
    for(int ii = 0; ii < 3; ii++ )
    {
        if( center[ii] < _min[ii] ) dmin += squared( center[ii] - _min[ii] );
        else if( center[ii] > _max[ii] ) dmin += squared( center[ii] - _max[ii] );
    }
    return dmin <= r2;
}

// get bounding box
void Octree::getBoundingBox(const float* positions, const int* vertices, int num_vertices)
{
    // reset bounding box
    _min.x = FLT_MAX;
    _min.y = FLT_MAX;
    _min.z = FLT_MAX;
    _max.x = -FLT_MAX;
    _max.y = -FLT_MAX;
    _max.z = -FLT_MAX;
    
    Vector3 tmp;
    for(int i=0;i<num_vertices;i++)
    {
        tmp.x = positions[vertices[i]*3];
        tmp.y = positions[vertices[i]*3+1];
        tmp.z = positions[vertices[i]*3+2];

        if(tmp.x<_min.x)_min.x = tmp.x;
        if(tmp.y<_min.y)_min.y = tmp.y;
        if(tmp.z<_min.z)_min.z = tmp.z;
        
        if(tmp.x>_max.x)_max.x = tmp.x;
        if(tmp.y>_max.y)_max.y = tmp.y;
        if(tmp.z>_max.z)_max.z = tmp.z;
    }
    
    // extend bounding box
    _min.x -= 0.1f;
    _min.y -= 0.1f;
    _min.z -= 0.1f;
    _max.x += 0.1f;
    _max.y += 0.1f;
    _max.z += 0.1f;
}


// get furthest corner
void Octree::getFurthestCorner(const Vector3& point, Vector3& corner)
{
    Vector3 delta;
    float dist;
    float furthestDist=-1.0f;
    
    float P[6] = {
        (float)_min.x,
        (float)_min.y,
        (float)_min.z,
        (float)_max.x,
        (float)_max.y,
        (float)_max.z
    };
    
    int permutation[24] = {0,1,2,3,1,2,0,1,5,3,1,5,
        0,4,2,3,4,2,0,4,5,3,4,5};
    
    for(unsigned int z=0;z<8;z++)
    {
        Vector3 currentCorner(P[permutation[z*3]],P[permutation[z*3+1]],P[permutation[z*3+2]]);
        delta = point - currentCorner;
        dist = (float)delta.length();
        if(dist>furthestDist)
        {
            furthestDist = dist;
            corner = currentCorner;
        }
    }
}

// build tree
void Octree::buildTree(std::vector<Triangle>& triangles, const float * positions)
{
    clearTree();
    
    // loop over all triangles, insert all leaves to the tree
    std::vector<Triangle>::iterator it;
    for(it = triangles.begin();it<triangles.end();it++)
    {
        insert(&(*it));
    }
    
    split(positions);
}

// split tree
void Octree::split(const float* positions)
{
    int tsz = _triangles.size();

    if (tsz <= MAX_TRIANGLES_NUMBER || (tsz <= 2*MAX_TRIANGLES_NUMBER && _depth > 3) ||(tsz <= 3*MAX_TRIANGLES_NUMBER && _depth > 4) ||_depth > 6 )
    {
        _isLeaf = true;
        return;
    }
    
    _isLeaf = false;
    
    double xx[] = {_min.x, 0.5*(_min.x+_max.x), _max.x};
    double yy[] = {_min.y, 0.5*(_min.y+_max.y), _max.y};
    double zz[] = {_min.z, 0.5*(_min.z+_max.z), _max.z};
    
    Vector3 center, halfSize;

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++)
            {
                int m = 4*i + 2*j + k;
                _child[m] = new Octree( Vector3(xx[i], yy[j], zz[k]),Vector3(xx[i+1], yy[j+1], zz[k+1]),_depth+1);
                /*
#ifdef USE_GLDRAWER
                _child[m]->_r = ((float) rand()) / (float) RAND_MAX;
                _child[m]->_g = ((float) rand()) / (float) RAND_MAX;
                _child[m]->_b = ((float) rand()) / (float) RAND_MAX;
#endif
                 */
                center = _child[m]->getCenter();
                halfSize = _child[m]->getHalfSize();
                int tsz = _triangles.size();
                
                for (int t = 0; t < tsz; t++)
                    if (_triangles[t]->touch(positions, center, halfSize))
                        _child[m]->insert(_triangles[t]);
                
                if (_child[m]->getSize() == 0)
                {
                    delete _child[m];
                    _child[m] = NULL;
                }
                else _child[m]->split(positions);
            }
    
    //if (_depth > 0)_triangles();
}

/*
// draw tree
#ifdef USE_GLDRAWER
void Octree::draw(GLDrawer::Compound* compound)
{
    if(_isLeaf)
    {
        GLDrawer::Box* box = new GLDrawer::Box(_min, _max, _r, _g, _b);
        compound->addShape(box);
    }
    else
    {
        for(int i=0;i<8;i++){
            if(_child[i])_child[i]->draw(compound);
        }
    }
}
#endif
*/
}//end namespace BOB
