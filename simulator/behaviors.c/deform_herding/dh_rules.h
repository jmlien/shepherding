#ifndef _DEFORM_HERDING_RULES_H
#define _DEFORM_HERDING_RULES_H

#include "simple_herding_rules.h"
#include "dh_grid.h"

class CDH_ForceRule : public CSimpleHerdingForceRule
{
public:
	CDH_ForceRule(){TargetAttract=1;}
    virtual Vector2d getForce(CFlockState & s);
};


class CDH_BehaviorRule: public CSimpleHerdingBehaviorRule
{
public:

	typedef pair<int,int> CellID;
	typedef list<CellID>  CellIDList;
	
	CDH_BehaviorRule();
    virtual void applyRule( CFlockState& s );
    static dhGrid * getGrid(){ return m_grid; }

protected:
	
	//find traget
	bool findTargets(CHerdingFlockState& s);
	
	//given the milestone find 
	//a cloesest sheep that is in a free cell...
	Point2d findLeadingSheepPos(FSLIST& sheep);
	
	//there are cells that we should push sheeps into
	void markTargetCells(const Point2d& pos, int size);
	
	void unmarkTargetCells();
	
	//these are the cells that sheephs are in
	//excluding those that are in the target cells
	void markSheepCells(FSLIST& sheep);
	
	void unmarkSheepCells();
	
	//these are the cells that shepherd should
	//place themselves in
	void markShepherdCells();
	
	void unmarkShepherdCells();
	
	//get closest target cell for a sheeph cell
	CellID findClosest(CellID& sheepcell) const;
	
	bool validate_and_add_ShepherdCell(CellID& shepherd);
	
	// classify agents into flock and shepherds
	void classifyAgents
	(CHerdingFlockState& s, FSLIST& va, FSLIST& shepherds, FSLIST& flocks);
	
	//
	void assignTarget2Shepherds(FSLIST& shepherds, const Point2d& peak);
	void assignTarget2Shepherds(FSLIST& shepherds, list<Point2d>& tagets);
	
	static dhGrid * m_grid;
	
	CellIDList m_targetcells;    //target cells
	CellIDList m_sheepcells;     //sheep cells
	CellIDList m_shepherdcells;  //shepherd cells

	map<int, bool> m_new_targets_available; //true if a new target is available for the agent
};


#endif // SHEPHERD_RULES_H
