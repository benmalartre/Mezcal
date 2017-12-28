//--------------------------------------------------------
// KDTREE
//--------------------------------------------------------
#ifndef _KDTREE_H_
#define _KDTREE_H_

#include <vector>
#include <algorithm>
#include <math.h>
#include <stdlib.h>

namespace BOB{

#define KDTREE_DIM 3
#define KD_F32_MAX 3.402823466e+38
#define KD_F32_MIN 1.175494351e-38


struct KDPoint
{
    float v[KDTREE_DIM];
    unsigned ID;
};

struct KDSort
{
    float v;
    int ID;
};

struct KDNode
{
    unsigned ID;
    unsigned level;
    float split_value;
    
    KDNode *left;
    KDNode *right;
    KDNode *parent;

    unsigned leftID;
    unsigned rightID;
    unsigned parentID;
    
    float r;
    float g;
    float b;
    
    bool hit;
    std::vector<unsigned> indices;
};

class KDTree
{
    unsigned nbp;
    KDNode *root;
    unsigned currentaxis;
    unsigned levels;
    unsigned cmps;
    unsigned ID;
    std::vector<KDPoint*> points;
    std::vector<KDSort> closests;
   
public:
    KDTree();
    ~KDTree();
    KDNode* createNode(KDNode* parent, unsigned ID, unsigned level);
    void deleteNode(KDNode* node);
    bool sortPoints(const unsigned& a, const unsigned& b);
    
    void split(KDNode* node,KDNode* left,KDNode* right);
    void build(const float* pnts,unsigned num_points, unsigned max_levels=99,unsigned min_pnts=10);
    float distance(KDPoint* a,KDPoint* b);
    void getBoundingBox(KDNode* node,KDPoint& min,KDPoint& max);
    void searchAtNode(KDNode* node, KDPoint* query, unsigned& retID, float& retDist, KDNode*& retNode);
    void searchAtNodeRange(KDNode* node, KDPoint* query, float range, unsigned& retID, float& retDist);
    
    void searchNAtNodeRange(KDNode* node, KDPoint* query, float range);
    void search(KDPoint* query,unsigned& retID,float& retDist);
    void searchN(KDPoint* query,float max_distance,unsigned max_points);
    void resetHit();
    void resetHitAtNode(KDNode* node);
};
}//end namespace BOB
#endif /* _KDTREE_H_ */
