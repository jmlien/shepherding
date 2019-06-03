#ifndef _SH_MAPBASED_FLOCK_H_
#define _SH_MAPBASED_FLOCK_H_

#include "sh_FlockState.h"
#include "sh_Roadmap.h"

class CMapFlockState : public CFlockState
{
public:

    CMapFlockState(CFlock * type, CRoadMap* rmap);
	virtual ~CMapFlockState(){}

    ///////////////////////////////////////////////////////////////////////////
    const std::list<Point2d>& getGoals() const { return m_Goals; }
    void pushGoal(const Point2d& pos){ m_Goals.push_back(pos); }
    void popGoal() { m_Goals.pop_back(); }
    Point2d peepGoal(){ return m_Goals.back(); }
    bool isGoalEmpty() const { return m_Goals.empty(); }
    ///////////////////////////////////////////////////////////////////////////
    const std::list<CNodeData*>& getMems() const { return m_Mem; }
    void pushMem(CNodeData* node){ m_Mem.push_back(node); }
    void popMem(){ m_Mem.pop_back(); }
    CNodeData* peepMem(){ return m_Mem.back(); }
    bool isMemEmpty() const { return m_Mem.empty(); }
    ///////////////////////////////////////////////////////////////////////////
    CRoadMap * getMap(){ return m_Map;}
    ///////////////////////////////////////////////////////////////////////////
protected:
    CRoadMap *            m_Map;
    std::list<Point2d>    m_Goals;
    std::list<CNodeData*> m_Mem;
};

#endif //_SH_MAPBASED_FLOCK_H_

