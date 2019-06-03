#ifndef SIMPLE_HERDING_H
#define SIMPLE_HERDING_H

#include "shepherding_base.h"
#include "herding_flock.h"

//
class glDrawMilestones;
class glDrawMovingPath;
class glDrawGroups;

//herding flock
class CSimpleHerdingFlock : public CHerdingFlock
{
public:

    CSimpleHerdingFlock(CEnvironment * env, const string& name);
	
	//initialize itself
    virtual bool initialize(list< list<string> >& tokens, flock_raw_data& data);
    
    //path from the controlled flock center to the goal 
    list<Point2d> path;
    
    //groups of controlled flock
    list<FSLIST> groups; 
    
protected:

    glDrawGroups * dgroup;      //drawing groups
    glDrawMovingPath * dpath;   //draw flock's path
    glDrawMilestones * dmile;   //draw milestons
    
};


#endif //SIMPLE_HERDING_H
