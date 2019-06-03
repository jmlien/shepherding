#ifndef _SH_MAPBASED_FLOCK_H_
#define _SH_MAPBASED_FLOCK_H_

#include "sh_FlockState.h"
#include "sh_Roadmap.h"

class CMapFlockState : public CFlockState
{
public:

    CMapFlockState(CFlock * type, CRoadMap* rmap):CFlockState(type),m_Map(rmap){}
	virtual ~CMapFlockState(){}

    ///////////////////////////////////////////////////////////////////////////
	virtual const list<Point2d>& getGoals() const { return m_Goals; }
    virtual void pushGoal(const Point2d& pos){ m_Goals.push_back(pos); }
    virtual void popGoal() { m_Goals.pop_back(); }
    virtual Point2d peepGoal(){ return m_Goals.back(); }
    virtual bool isGoalEmpty() const { return m_Goals.empty(); }
    ///////////////////////////////////////////////////////////////////////////
    virtual const list<CNodeData*>& getMems() const { return m_Mem; }
    virtual void pushMem(CNodeData* node){ m_Mem.push_back(node); }
    virtual void popMem(){ m_Mem.pop_back(); }
    virtual CNodeData* peepMem(){ return m_Mem.back(); }
    virtual bool isMemEmpty() const { return m_Mem.empty(); }
    ///////////////////////////////////////////////////////////////////////////
    virtual CRoadMap * getMap(){ return m_Map;}
    ///////////////////////////////////////////////////////////////////////////
protected:

    CRoadMap *       m_Map;
    list<Point2d>    m_Goals;
    list<CNodeData*> m_Mem;
};

#endif //_SH_MAPBASED_FLOCK_H_

