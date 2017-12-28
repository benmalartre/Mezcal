#include "mesh.h"
#include <string.h>
#include <iostream>
#include <float.h>

using namespace std;
namespace BOB{
    Mesh::~Mesh(){}
    
    // constructor
    Mesh::Mesh(unsigned num_points, const float* in_positions, unsigned num_desc, int* desc)
    {
        positions.resize(num_points);
        memcpy( &positions[0], in_positions, num_points*3*sizeof(float));
        
        nbVertices = num_points;

        description.resize(num_desc);
        memcpy(&description[0], desc, num_desc*sizeof(int));
        
        getTriangles();
        
    }
    
    // get num faces
    void Mesh::getNumFaces()
    {
        nbFaces = 0;
        for(unsigned i=0;i<description.size();i++)
            if(description[i]<0)nbFaces++;
    }
    
    // get bounding box
    void Mesh::getBoundingBox(bool worldSpace)
    {
        _min.x = FLT_MAX;
        _min.y = FLT_MAX;
        _min.z = FLT_MAX;
        _max.x = -FLT_MAX;
        _max.y = -FLT_MAX;
        _max.z = -FLT_MAX;
        
        Vector3 tmp;
        for(unsigned i=0;i<nbVertices;i++)
        {
            tmp.x = positions[i].x;
            tmp.y = positions[i].y;
            tmp.z = positions[i].z;
            if(worldSpace) tmp *= matrix;
            
            if(tmp.x<_min.x)_min.x = tmp.x;
            if(tmp.y<_min.y)_min.y = tmp.y;
           	if(tmp.z<_min.z)_min.z = tmp.z;
            
            if(tmp.x>_max.x)_max.x = tmp.x;
            if(tmp.y>_max.y)_max.y = tmp.y;
            if(tmp.z>_max.z)_max.z = tmp.z;
        }
    }
    
    // get triangles from polygonal description
    void Mesh::getTriangles()
    {
        nbTriangles = 0;
        nbFaces = 0;
        unsigned vertexCount = 0;
        for(unsigned i=0;i<description.size();i++)
        {
            if(description[i]<0)
            {
                nbFaces++;
                nbTriangles+=vertexCount-2;
                vertexCount=0;
            }
            else vertexCount++;
        }
        
        triangles.resize(nbTriangles*3);
        unsigned triIndex = 0;
        std::vector<unsigned> faceDescription;
        for(unsigned i=0;i<description.size();i++)
        {
            if(description[i]>=0)
            {
                faceDescription.push_back(description[i]);
            }
            else
            {
                if(faceDescription.size() == 3)
                {
                    for(unsigned j=0;j<3;j++)
                    	triangles[triIndex++] = faceDescription[j];
                }
                else
                {
                    unsigned numTri = faceDescription.size()-2;
                    for(unsigned j=0;j<numTri;j++)
                    {
                        triangles[triIndex++] = faceDescription[0];
                        triangles[triIndex++] = faceDescription[j+1];
                        triangles[triIndex++] = faceDescription[j+2];
                    }
                }
                faceDescription.clear();
            }
        }
    }

    // update mesh datas
    void Mesh::set(unsigned num_points, const float * in_positions, const float* in_normals, Matrix4& M)
    {
        if(num_points == positions.size())
        {
            memcpy(&positions[0].x, in_positions, num_points*3*sizeof(float));
            memcpy(&normals[0].x, in_normals, num_points*3*sizeof(float));
            getBoundingBox();
        }
        
    }

}//end namespace BOB
