#ifndef _CRoadMap_H_
#define _CRoadMap_H_

/////////////////////////////////////////////////////////////////////////////
#include <iostream>
using namespace std;

/////////////////////////////////////////////////////////////////////////////
#include "sh_Graph.h"
#include "RNG.h"

#include "Vector.h"
using namespace mathtool;
/////////////////////////////////////////////////////////////////////////////
class CFlock;
class CBehaviorRule;
class CRoadMap;

/////////////////////////////////////////////////////////////////////////////
// this data (class) is redundant. All data can get from WeightedMultiDiGraph,
// but in order to get quiker execution time, waste some memory should be fine
class CNodeData {

    friend class CRoadMap;
    
public:
    ///////////////////////////////////////////////////////////////////////
    //  Constructor/Destructor
    CNodeData(const Point2d& m_Pos);
    ~CNodeData();

    //////////////////////////////////////////////////////////////////////
    //  Core
    void addSuccessor(CNodeData * s, float w);
    void changeWeight( CNodeData * s, float w );

    //get successor based on weight and probablistic
    CNodeData * getRandSuccessor(RNG * rng, CNodeData * s,int determin);
    CNodeData * getSuccessor(const int index)
    {
        std::list<EdgeData>::iterator itr = m_EdgeData.begin();
        for(int i = 0; i < index; i++)
        {
            itr++;
        }
        return itr->m_Successor;
    }
    //////////////////////////////////////////////////////////////////////
    //Access
    int getID(){ return m_ID; }
    int getSuccessorSize() const { return m_EdgeData.size(); }
    const Point2d& getPos() const { return m_Pos; }
    void setBehaviorRule(CBehaviorRule * r){ m_BRule=r; }
    CBehaviorRule * getBehaviorRule() const { return m_BRule; }
    //////////////////////////////////////////////////////////////////////
    struct EdgeData {
        CNodeData * m_Successor;
        float m_Weight;
        bool operator==(const EdgeData& other) const 
        { return m_Successor==other.m_Successor; }
    };

/////////////////////////////////////////////////////////////////////////////
//private:
public:
    int m_ID;

    std::list<EdgeData> m_EdgeData;
    Point2d m_Pos;
    CBehaviorRule * m_BRule;
};

class CRoadMap {

public:

    ///////////////////////////////////////////////////////////////////////
    //  Constructor/Destructor
    CRoadMap(CFlock *,int size=100);
    ~CRoadMap();

    //////////////////////////////////////////////////////////////////////
    //  Core
    int addNode(CNode& node);
    void delNode(int id);
    vector<CNodeData*>& getNodes(){ return m_CNodeData; }
    CNodeData * getNode( int id ) const { return m_CNodeData[id]; }
    int getNodeSize() const { return m_CNodeData.size(); }
    void addEdge( CNodeData* n1, CNodeData* n2, float weight );
    void addEdge( int v1, int v2 );
    void addEdge( int v1, int v2, float weight );

    //////////////////////////////////////////////////////////////////////
    // Access Method
    WDG & getGraph() { return m_Graph; }
    CFlock * getFlock() const { return m_F; }

	//find the closest node for a give point
	VID closestNode(const Point2d& pt);

	//read/save
	bool read(string& filename); //read from
	bool save(string& filename); //save to
		
///////////////////////////////////////////////////////////////////////
protected:
	int addNodeData(CNode& node, CNodeData * data);

///////////////////////////////////////////////////////////////////////
//  Private Data
private:
    WDG m_Graph;
    vector<CNodeData*> m_CNodeData;
    CFlock * m_F;
};

#endif

