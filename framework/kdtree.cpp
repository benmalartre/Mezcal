//-------------------------------------------------------
// KDTREE IMPLEMENTATION
//-------------------------------------------------------
#include "kdtree.h"
namespace BOB{
KDTree::KDTree(){};
KDTree::~KDTree()
{
    deleteNode(root);
}

float KDTree::distance(KDPoint* a,KDPoint* b)
{
    float dist,d;
    dist = 0.0f;
    for(unsigned i=0;i<KDTREE_DIM;i++)
    {
        d = a->v[i] - b->v[i];
        dist += d*d;
    }
    return dist;
}

KDNode* KDTree::createNode(KDNode* parent,unsigned ID,unsigned level)
{
	KDNode* node = new KDNode();
	node->parent = parent;
	node->ID = ID;
	node->level = level;
	node->r = ((float) rand() / (RAND_MAX));
	node->g = ((float) rand() / (RAND_MAX));
	node->b = ((float) rand() / (RAND_MAX));
	return node;
}

void KDTree::deleteNode(KDNode* node)
{
    if(node->left)deleteNode(node->left);
    if(node->right)deleteNode(node->right);
    delete node;
}

bool KDTree::sortPoints(const unsigned& a,const unsigned& b)
{
    return (points[a]->v[currentaxis]<points[b]->v[currentaxis]);
}

void KDTree::getBoundingBox(KDNode* node, KDPoint& min, KDPoint& max)
{
    KDPoint* v;
    min.v[0] = KD_F32_MAX;
    min.v[1] = KD_F32_MAX;
    min.v[2] = KD_F32_MAX;

    max.v[0] = KD_F32_MIN;
    max.v[1] = KD_F32_MIN;
    max.v[2] = KD_F32_MIN;

    for(unsigned ii=0;ii<node->indices.size();ii++)
    {
        v = points[node->indices[ii]];
        
        //Vector3_MulByMatrix4InPlace(*v,*srt)
        if(v->v[0] < min.v[0]) min.v[0] = v->v[0];
        if(v->v[1] < min.v[1]) min.v[1] = v->v[1];
        if(v->v[2] < min.v[2]) min.v[2] = v->v[2];
        
        if(v->v[0] > max.v[0]) max.v[0] = v->v[0];
        if(v->v[1] > max.v[1]) max.v[1] = v->v[1];
        if(v->v[2] > max.v[2]) max.v[2] = v->v[2];
    }
}

void KDTree::resetHitAtNode(KDNode* node)
{
    if(node->left)
    {
    	resetHitAtNode(node->left);
        resetHitAtNode(node->right);
    }
    else node->hit = false;
}

void KDTree::resetHit()
{
    resetHitAtNode(root);
}

bool compareValue( const KDSort& lhs, const KDSort& rhs) { return lhs.v < rhs.v; }

void KDTree::split(KDNode* node,KDNode* left,KDNode* right)
{
    currentaxis = node->level % KDTREE_DIM;
    unsigned i,j;

    std::vector<KDSort> v;
    unsigned nbi = node->indices.size();
    v.resize(nbi);

    for(i=0;i<nbi;i++)
    {
        v[i].v = points[node->indices[i]]->v[currentaxis];
        v[i].ID = node->ID;
    }

    std::sort(v.begin(), v.end(), compareValue);
    
    for(i=0;i<node->indices.size();i++)
    {
        node->indices[i] = v[i].ID;
    }

    node->split_value = points[node->indices[nbi/2]]->v[currentaxis];
    
    for(i=0;i<nbi;i++)
    {
        j = node->indices[i];
        if(points[j]->v[currentaxis]<node->split_value)
        {
            left->indices.push_back(j);
        }
        else
        {
            right->indices.push_back(j);
        }
    }
}

void KDTree::searchAtNode(KDNode*node, KDPoint *query	, unsigned int &retID, float &retDist, struct KDNode*& retNode)
{
    unsigned best_idx =0;
    float best_dist = KD_F32_MAX;
    unsigned idx;
    float dist;
    bool search = true;
    unsigned split_axis;
    while(search)
    {
        split_axis = node->level % KDTREE_DIM;
        cmps++;
        if(!node->left)
        {
            retNode = node;
            for(unsigned x=0;x<node->indices.size();x++)
            {
                cmps++;
                idx = node->indices[x];
                dist = distance(query,points[idx]);
                if(dist<best_dist)
                {
                    best_dist = dist;
                    best_idx = idx;
                }
            }
            search=false;
        }
        else if(query->v[split_axis]<node->split_value)
            node = node->left;
        else
            node = node->right;
    }
    retID = best_idx;
    retDist = best_dist;
}


void KDTree::searchAtNodeRange(KDNode* node, KDPoint* query, float range,unsigned& retID, float& retDist)
{
    unsigned best_idx=0;
    float best_dist = KD_F32_MAX;
    unsigned split_axis;
    unsigned idx;
    float dist;
    
    std::vector<KDNode*> to_visit;
    to_visit.push_back(node);
    
    while(to_visit.size())
    {
        std::vector<KDNode*> next_search;

        while(to_visit.size())
        {
            node = *to_visit.rbegin();
            to_visit.pop_back();
            split_axis = node->level %KDTREE_DIM;
            
            if(!node->left)
            {
                for(unsigned ii=0;ii<node->indices.size();ii++)
                {
                    cmps++;
                    idx = node->indices[ii];
                    dist = distance(query, points[idx]);
                    if(dist<best_dist)
                    {
                        best_dist = dist;
                        best_idx = idx;
                        node->hit = true;
                    }
                }
            }
            else
            {
                dist = query->v[split_axis]- node->split_value;
                // there are 3 possible scenarios
                // the hypercircle only intersect the left region
                // the hypercricle only intersect the right region
                // the hypercircle intersects both
                cmps++;
                if(fabs(dist)>range)
                {
                    if(dist<0) next_search.push_back(node->left);
                    else next_search.push_back(node->right);
                }
                else
                {
                    next_search.push_back(node->left);
                    next_search.push_back(node->right);
                }
            }
        }
        std::copy(next_search.begin(),next_search.end(),to_visit.begin());
    }
    retID = best_idx;
    retDist = best_dist;
}

void KDTree::searchNAtNodeRange(KDNode* node, KDPoint* query, float range)
{
    unsigned split_axis;
    unsigned idx;
    float dist;
    
    std::vector<KDNode*> to_visit;
    to_visit.push_back(node);
    
    while(to_visit.size())
    {
        std::vector<KDNode*> next_search;
        
        while(to_visit.size())
        {
            node = *to_visit.rbegin();
            to_visit.pop_back();
            split_axis = node->level %KDTREE_DIM;
            
            if(!node->left)
            {
                for(unsigned ii=0;ii<node->indices.size();ii++)
                {
                    cmps++;
                    idx = node->indices[ii];
                    dist = distance(query, points[idx]);
                    if(dist<range)
                    {
                        node->hit = true;
                        KDSort closest;
                        closest.v = sqrt(dist);
                        closest.ID = idx;
                        closests.push_back(closest);
                    }
                }
            }
            else
            {
                dist = query->v[split_axis]- node->split_value;
                // there are 3 possible scenarios
                // the hypercircle only intersect the left region
                // the hypercricle only intersect the right region
                // the hypercircle intersects both
                cmps++;
                if(fabs(dist)>range)
                {
                    if(dist<0) next_search.push_back(node->left);
                    else next_search.push_back(node->right);
                }
                else
                {
                    next_search.push_back(node->left);
                    next_search.push_back(node->right);
                }
            }
	
        }
        std::copy(next_search.begin(),next_search.end(),to_visit.begin());
    }
}

void KDTree::search(KDPoint* query,unsigned& retID,float& retDist)
{
    KDNode* best_node = NULL;
    unsigned best_idx = 0;
    float best_dist = KD_F32_MAX;
    float radius = 0.0f;
    cmps = 0;
    
    // find the closest Node, this will be the upper bound for the next searches
    searchAtNode(root,query,best_idx,best_dist,best_node);
    radius = sqrt(best_dist);
    
    // now find possible other candidates
    KDNode* node = best_node;
    KDNode* parent;
    unsigned split_axis;
    
    while(node->parent)
    {
        // go up
        parent = node->parent;
        split_axis = parent->level % KDTREE_DIM;
        
        // search theother node
        unsigned tmp_idx;
        float tmp_dist = KD_F32_MAX;
        KDNode* tmp_node;
        KDNode* search_node;
        
        if(fabs(parent->split_value - query->v[split_axis]) <= radius)
        {
        	// search opposite node
        	if(parent->left != node)
            {
                searchAtNodeRange(parent->left,query,radius,tmp_idx,tmp_dist);
            }
            else
            {
                searchAtNodeRange(parent->right,query,radius,tmp_idx,tmp_dist);
            }
        }
        
        if(tmp_dist<best_dist)
        {
            best_dist = tmp_dist;
            best_idx = tmp_idx;
        }
            
        node = parent;
    }
}

void KDTree::searchN(KDPoint* query,float max_distance,unsigned max_points)
{
    //find the closest Node, this will be the upper bound for the next searches
    KDNode* best_node = NULL;
    unsigned best_idx = 0;
    float best_dist = KD_F32_MAX;
    float radius = 0;
    cmps = 0;
    unsigned founds = 0;
    resetHit();
    closests.clear();
    
    searchAtNode(root,query,best_idx,best_dist,best_node);
    radius = sqrt(best_dist);
    if(radius > max_distance) return;
    
    KDSort closest;
    closest.v = radius;
    closest.ID = best_idx;
    closests.push_back(closest);
    
    searchNAtNodeRange(best_node,query,max_distance);
    
    // now find possible other candidates
    KDNode* node = best_node;
    KDNode* parent;
    unsigned split_axis;
    
    while(node->parent)
    {
        // go up
        parent = node->parent;
        split_axis = parent->level % KDTREE_DIM;
        
        // search the other node
        unsigned tmp_idx;
        float tmp_dist = KD_F32_MAX;
        KDNode* tmp_node;
        KDNode* search_node;
		
        if(fabs(parent->split_value - query->v[split_axis]) <= max_distance)
        {
            // search opposite node
            if(parent->left != node)
            {
                searchAtNodeRange(parent->left,query,radius,tmp_idx,tmp_dist);
            }
            else
            {
                searchAtNodeRange(parent->right,query,radius,tmp_idx,tmp_dist);
            }
        }
        node = parent;
    }
    
    std::sort(closests.begin(), closests.end(), compareValue);
    if(max_points>0 and closests.size()>max_points)
    {
        closests.resize(max_points);
    }
}

void KDTree::build(const float* pnts,unsigned num_points, unsigned max_levels,unsigned min_pnts)
{
    nbp = num_points;
    levels = max_levels;
    root = createNode(NULL,0.f,0.f);
    root->ID = ID;
    ID++;
    
	points.resize(nbp);
    unsigned i;
    KDPoint* pnt;
    for(i=0;i<nbp;i++)
    {
        pnt = new KDPoint();
        pnt->v[0] = pnts[i*3];
        pnt->v[1] = pnts[i*3+1];
        pnt->v[2] = pnts[i*3+2];
        pnt->ID = i;
        points[i] = pnt;
    }
	
    root->indices.resize(nbp);
    for(i=0;i<nbp;i++) root->indices[i] = i;
    
    KDNode* node;
    std::vector<KDNode*> to_visit;
    to_visit.push_back(root);
    while(to_visit.size())
    {
        std::vector<KDNode*> next_search;
        while(to_visit.size())
        {
            node = to_visit.back();
            to_visit.pop_back();
            if(node->level<levels)
            {
                if(node->indices.size()>min_pnts)
                {
                    KDNode* left = createNode(node, ID,node->level+1);
                    ID++;
                    KDNode* right = createNode(node, ID,node->level+1);
                    ID++;
                    
                    split(node, left, right);
                    
                    // clear the current indices
                    node->indices.clear();
                    
                    node->left = left;
                    node->right = right;
                    node->leftID = left->ID;
                    node->rightID = right->ID;
                    
                    if(left->indices.size())
                        next_search.push_back(left);
                    
                    if(right->indices.size())
                        next_search.push_back(right);
                }
            }
        }
        std::copy(next_search.begin(), next_search.end(), to_visit.begin());
    }
}

}//end namespace BOB
