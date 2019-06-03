// dyPRMS.h
//
//////////////////////////////////////////////////////////////////////

#if !defined(_SH_DYPRM_H_)
#define _SH_DYPRM_H_

#include "sh_Roadmap.h"
#include "sh_PRM.h"

class CFlock;
class CFlockState;
class CEnvironment;
class CNodeData;

class CDynPRM
{
public:
    //////////////////////////////////////////////////////////////////////
    // Constructor & Destructor
    CDynPRM(CEnvironment * env, RNG * rng);
    virtual ~CDynPRM();

    //////////////////////////////////////////////////////////////////////
    // Core

    //find a path from s to g, if failed, expand the map
    bool queryPath(const Point2d& s,const Point2d& g,
    CRoadMap& d_map,CRoadMap& g_map, CFlockState* shepherd,
    std::list<CFlockState*>& flock, std::list<Point2d>& path /*return*/);

    //update the map
    void expandMap(CRoadMap& ,CFlockState* ,std::list<CFlockState*>&);
    //remove nodes if they can't see any flock
    void updateMap(CRoadMap& ,CFlockState* ,std::list<CFlockState*>&);

//////////////////////////////////////////////////////////////////////
// Private
protected:

    //find a path from s to g
    bool findPath(const Point2d& s,const Point2d& g,
    CRoadMap& d_map,CRoadMap& g_map, CFlockState* shepherd,
    std::list<CFlockState*>& flock, std::list<Point2d>& path /*return*/);

    int countNodes(CRoadMap& dy_map, const Point2d& O, float R);

    void sampleInCircle
    (CFlockState* shepherd, std::list<CFlockState*>& flock,
     CRoadMap& dy_map, const Point2d& O, float R, int size);
    Point2d sampleInCircle(const Point2d& O, float R);

    float computeWeight
    (std::list<CFlockState*>& flock, const Point2d& cfg, float R);

    void connectNodes(CRoadMap& rmap);

    int FindKClosest
    (CNodeData * node, CNodeData * close[], vector<CNodeData*>& nodes,bool sortW=false);

    int connectToMap
    (const Point2d& cfg, CRoadMap& d_map, CRoadMap& g_map, std::list<Point2d>& path,
     bool checkDir=false, Vector2d dir=Vector2d(1,0));

    bool findPath_In_G_Map
    (const Point2d& s,const Point2d& g,CRoadMap& g_map, std::list<Point2d>& path);

    bool findPathV1V2
    (CRoadMap& rmap, int v1, int v2, std::list<Point2d>& path);

    int BFS(WDG& g,int v1, int v2,std::list<CNode>& nodes);

    //data member
    CEnvironment * m_pEnv;   //the env
    RNG * m_pRNG;
    shCD           m_CD;
    PRMS m_PRMs;
    vector<float> m_Weight;

    //new sampled nodes
    std::list<CNodeData*> m_NewNodes;
};

#endif // !defined(_SH_DYPRM_H_)

