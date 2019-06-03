

#ifndef BALANCED_WAYPOINT_WAYPOINT_HPP
#define BALANCED_WAYPOINT_WAYPOINT_HPP


#include <vector>
#include "sh_FlockState.h"
#include "Point.h"


namespace Balanced {


struct WAYPOINT
{
    WAYPOINT(const std::vector<CFlockState*>& sheep, const Point2d& goal) :
        sheep(sheep),
        goal(goal)
    {
        // do nothing
    }
    WAYPOINT(const WAYPOINT& wp) :
        sheep(wp.sheep),
        goal(wp.goal)
    {
        // do nothing
    }
    
    const WAYPOINT& operator =(const WAYPOINT& wp)
    {
        this->sheep = wp.sheep;
        this->goal = wp.goal;
        return *this;
    }
    
    std::vector<CFlockState*> sheep;
    Point2d goal;
};


} // end Balanced


#endif // BALANCED_WAYPOINT_WAYPOINT_HPP


