#ifndef SIMPLE_SHEPHERD_RULES_H
#define SIMPLE_SHEPHERD_RULES_H

#include "shepherding_base.h"

#include "Point.h"
using namespace mathtool;

#include <list>
using namespace std;

class CHerdingFlockState;

class CSimpleHerdingForceRule : public CBasicForceRule
{
public:
	CSimpleHerdingForceRule(){TargetAttract=1;}
    virtual Vector2d getForce(CFlockState & s);
	int TargetAttract;
};


class CSimpleHerdingBehaviorRule: public CBehaviorRule
{
public:

	CSimpleHerdingBehaviorRule();

    virtual void applyRule( CFlockState& s );

	//compute milestone of the scared flock
    vector<Point2d> findMilestone
	(CMapFlockState& state, const Point2d& s, const Point2d& g, float r);

protected:

    //check the heading dir of the shepherd
    void checkDirection(CHerdingFlockState& s);

    //compute groups, then compute milestone, then find target
    bool findTarget(CHerdingFlockState& s, Point2d& target);

    list<FSLIST> findTargetGroups(CMapFlockState& state, list<FSLIST>& groups);

    void truncateVel(CFlockState& s, float maxv);

    //check if all flock mem reach the goal
    void reachGoal(CHerdingFlockState& s); 

    //data
    std::list<Point2d> pathfinding_waypoints;   // stores path to sheep when obstacle blocks the shepherd
};


#endif // SHEPHERD_RULES_H
