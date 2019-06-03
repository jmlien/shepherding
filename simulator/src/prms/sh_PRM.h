// PRMS.h: interface for the PRMS class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _SH_PRM_H_
#define _SH_PRM_H_

#include "sh_Roadmap.h"
#include "sh_CollisionDetection.h"
#include "RNG.h"

#include <limits>

class CFlock;
class CEnvironment;
class CObs;
class CObsState;

class PRMS  
{
public:
    //////////////////////////////////////////////////////////////////////
    // Constructor & Destructor
    PRMS(CEnvironment * env, RNG * rng);
    virtual ~PRMS();

    //////////////////////////////////////////////////////////////////////
    // Core
    //Vector2d getRandFreeCfg();
    
    //sample uniformly
    virtual void samplePRMNodes(CRoadMap& rmap, int n, float spacing=0);
    
    //sample near medial axis
    virtual void sampleMAPRMNodes(CRoadMap& rmap, int n, float spacing=0);
    
    //sample near obstacles
    virtual void sampleOBPRMNodes(CRoadMap& rmap, int n, float spacing=0);
    
    virtual int  sampleOBPRMNodes_on_Obsts
    (CObs& obst,CRoadMap& rmap,int n,float spacing=0);
    
    virtual int  sampleOBPRMNodes_on_Obst
    (CObs& obst,CObsState& state,CRoadMap& rmap,int n,float spacing=0);

	//connect each node in rmap to k-closest nodes
    virtual void connectNodes(CRoadMap& rmap, int k);

	//connect each node in rmap to nodes within radius distance
    virtual void connectNodes(CRoadMap& rmap, float radius);
    
    //connect from pos to rmap
    virtual CNodeData * connect2Map(CRoadMap& rmap, const Point2d& pos);

	//connect pairs of nodes in closest  
	virtual void connectNodes(CRoadMap& rmap, list< pair<int,int> >& closest);

    //compute a path from start to goal
    virtual void findPath
    (CRoadMap& rmap,const Point2d& s,const Point2d& g,std::list<Point2d>& path);

    //compute a path from v1 to v2
    virtual bool findPathV1V2
    (CRoadMap& rmap, int v1, int v2, std::list<Point2d>& path);

	//remove unnecessary nodes
	virtual void simplifyMap(CRoadMap& rmap);

    //////////////////////////////////////////////////////////////////////
    // access/setting functions
    
    // allow the connection function to create 
    // good loops so that the max distances between nodes
    // are reduced iteratively
    virtual void setLoopOptimzation(bool flag) { m_create_good_loops=flag; }
    
    //if flag is true, then edges will be added even if the end pts 
    //are in the same CC. As the result, loops are created.
    //if flag is false, nodes in the same CC are not connected
    virtual void setLoopCreation(bool flag) { m_skip_same_cc=!flag; }
    
//////////////////////////////////////////////////////////////////////
// Private
protected:

    typedef pair<VID,VID> IDpair;

    virtual int sampleOBPRMNodes_priviate
    (CRoadMap& rmap,int n,float spacing,
    std::list< pair<Point2d,Point2d> >& boundaries,float length, float ratio);

    //calc cfgs from n1 to 12
    virtual void calcPathCfg
    (CRoadMap& map,const Point2d& p1, const Point2d& p2, std::list<Point2d>& path);

    //smooth the path
    typedef std::list<Point2d>::iterator PIT;
    virtual void pathSmoothing(CRoadMap& map,std::list<Point2d>& path, PIT s);

    virtual void FindKClosest
    (CNodeData * node, int K, vector<CNodeData*>& close, vector<CNodeData*>& nodes);

	//return sorted list of pairs (closest to farthest)
    virtual void sortedPairs
    (vector<CNodeData*>& nodes, std::list< pair<int,int> >& close, int k);

	//return a list of pairs that are at most d apart
    virtual void nearPairs
    (vector<CNodeData*>& nodes, std::list< pair<int,int> >& close, float d);

	//indentify good loops
	virtual void findGoodLoops(CRoadMap& rmap, vector<CNodeData*>& nodes, const list< pair<int,int> >& loops);

    //data member
    CEnvironment * m_pEnv;   //the env
    RNG          * m_pRNG;   //randon number generator
    shCD           m_CD;
    
    bool m_create_good_loops;
    bool m_skip_same_cc;
};


//compute the length of a given path
inline float pathLength(const list<Point2d>& path)
{
	if (path.empty()) return FLT_MAX; //std::numeric_limits<float>::max();
	if(path.size()==1) return 0;

	typedef list<Point2d>::const_iterator CIT;
	CIT last=--path.end();
	float length=0;
	
	for(CIT i=path.begin();i!=last;i++){
		CIT j=i; j++;
		length+=((*i)-(*j)).norm();
	}//end i

	return length;
}

#endif // _SH_PRM_H_

