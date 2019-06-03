

#ifndef BALANCED_BEHAVIOR_BEHAVIOR_DEFINITION_HPP
#define BALANCED_BEHAVIOR_BEHAVIOR_DEFINITION_HPP


#include <vector>
#include "WAYPOINT/WAYPOINT.hpp"
#include "simple_herding.h"
#include "sh_BehaviorRules.h"


namespace Balanced {


class BEHAVIOR : public CBehaviorRule
{
    public:
//==============================
// public constructors
//==============================
    BEHAVIOR();
    BEHAVIOR(const BEHAVIOR& behavior);
    ~BEHAVIOR();

//==============================
// public methods
//==============================
    void applyRule(CFlockState& s);
    
    private:
//==============================
// private methods
//==============================    
    void InitializeParameters();

//==============================
// private attributes
//==============================
    std::vector<CHerdingFlockState*> shepherds;
    std::vector<CFlockState*> sheep;
    float sheep_separation;
    float sheep_visibility;
    std::vector<WAYPOINT> waypoints;
    std::vector<Point2d> targets;
//==============================
};


} // end Balanced


#endif // BALANCED_BEHAVIOR_BEHAVIOR_DEFINITION_HPP


