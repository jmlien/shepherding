#ifndef SIMPLE_HERDING_H
#define SIMPLE_HERDING_H

#include <list>
#include "shepherding_base.h"
#include "ShepherdingInstance.h"
#include "simple_herding_draw.h"

class PRMS;

//herding flock
class CHerdingFlock : public CFlock
{
public:
    CHerdingFlock(CEnvironment * env, const string& name, int size=50);
	PRMS * prm;
	DrawGroups * dgroup;    //drawing groups
	DrawPath * dpath;       //draw flock's path
	DrawMilestones * dmile; //draw milestons
};


//data associated the the flock group
struct FlockGroup{
	FlockGroup(){ shepherd_number=0; radius=0; }
	int shepherd_number; //# of shepherds steering this group
	FSLIST states;
	Point2d center;
	Point2d targetRef;
	float radius;
	vector<Point2d> milestones;
	vector<Point2d> targets;  
};

//herding flock state
class CHerdingFlockState : public CMapFlockState
{
public:
    CHerdingFlockState(CFlock * type, CRoadMap* rmap):CMapFlockState(type,rmap)
    {
        isApproaching=false;
        isTurning=false;
		isTravelling=false;
		swing_step=0;
    }

    Point2d goal;                // final goal
    Point2d target;              // current goal
	int swing_step;              //only used in swing locomotion
    vector<Point2d> milestones;  // the path that push the flock to home
    bool isApproaching;          // set to not initially
    bool isTurning;              // set to not initially
	bool isTravelling;           // set to true if the shepherd in travelling to the target
	FlockGroup flock_group;      // the group of flock under control
};

class SimpleHerding : public ShepherdingInstance
{
protected:
	bool initialize(list< list<string> >& tokens, flock_raw_data& data);
	void parseCmdline(int argc, char * argv[]);
};

#endif
