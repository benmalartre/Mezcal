//--------------------------------------------------------
// Grid
//--------------------------------------------------------
#ifndef _GRID_H_
#define _GRID_H_

#include <float.h>
#include "vector3.h"
#include "point.h"
#include "tetrahedron.h"

namespace BOB{
struct Cell{
    Point* corners[8];
    //Point center;
    Tetrahedron* tets[5];
    char border;
    char flip;
    unsigned ID;
};

class Grid{
public:
    Grid();
    Grid(Vector3& bmin, Vector3& bmax, int x, int y, int z);
    ~Grid();
    void setResolution(int x, int y, int z);
    void setBoundingBox(Vector3& corner1, Vector3& corner2);
    void create();
    void filter();
    unsigned size();
public:
    Vector3 min;
    Vector3 max;
    std::vector<Point> points;
    std::vector<Cell> cells;
    std::vector<Tetrahedron*> tets;
    unsigned numCells;
    unsigned numPoints;
    unsigned nx;
    unsigned ny;
    unsigned nz;
};

}// end namespace BOB
#endif /* _GRID_H_ */
