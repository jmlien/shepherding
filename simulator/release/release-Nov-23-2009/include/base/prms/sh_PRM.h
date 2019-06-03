// PRMS.h: interface for the PRMS class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PRMS_H__0C737B80_C38C_4DED_93EF_C07B225C6F6A__INCLUDED_)
#define AFX_PRMS_H__0C737B80_C38C_4DED_93EF_C07B225C6F6A__INCLUDED_

#include "sh_Roadmap.h"
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
    PRMS(CEnvironment * env);
    virtual ~PRMS();

    //////////////////////////////////////////////////////////////////////
    // Core
    //Vector2d getRandFreeCfg();

    void sampleMAPRMNodes(CRoadMap& rmap, int n, float spacing=0);
    void sampleOBPRMNodes(CRoadMap& rmap, int n, float spacing=0);
    int  sampleOBPRMNodes_on_Obsts
    (CObs& obst,CRoadMap& rmap,int n,float spacing=0);
    int  sampleOBPRMNodes_on_Obst
    (CObs& obst,CObsState& state,CRoadMap& rmap,int n,float spacing=0);

    void connectNodes(CRoadMap& rmap, int k);

    //connect from pos to rmap
    CNodeData * connect2Map(CRoadMap& rmap, const Point2d& pos);

    //compute a path from start to goal
    void findPath
    (CRoadMap& rmap,const Point2d& s,const Point2d& g,std::list<Point2d>& path);

    //compute a path from v1 to v2
    bool findPathV1V2
    (CRoadMap& rmap, int v1, int v2, std::list<Point2d>& path);

	//remove unnecessary nodes
	void simplifyMap(CRoadMap& rmap);

//////////////////////////////////////////////////////////////////////
// Private
private:

    typedef pair<VID,VID> IDpair;

    int sampleOBPRMNodes_priviate
    (CRoadMap& rmap,int n,float spacing,
    std::list< pair<Point2d,Point2d> >& boundaries,float length, float ratio);

    //calc cfgs from n1 to 12
    void calcPathCfg
    (CRoadMap& map,const Point2d& p1, const Point2d& p2, std::list<Point2d>& path);

    //smooth the path
    typedef std::list<Point2d>::iterator PIT;
    void pathSmoothing(CRoadMap& map,std::list<Point2d>& path, PIT s);

    void FindKClosest
    (CNodeData * node, int K, vector<CNodeData*>& close, vector<CNodeData*>& nodes);

    void sortedNodes
    (vector<CNodeData*>& nodes, std::list< pair<int,int> >& close, int k);

	//indentify good loops
	void findGoodLoops(CRoadMap& rmap, vector<CNodeData*>& nodes, const list< pair<int,int> >& loops);

    //data member
    CEnvironment * m_pEnv;   //the env
};


//compute the length of a given path
inline float pathLength(const list<Point2d>& path)
{
	if(path.empty()) return numeric_limits<float>::max();
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

#endif // !defined(AFX_PRMS_H__0C737B80_C38C_4DED_93EF_C07B225C6F6A__INCLUDED_)

