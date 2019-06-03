

#ifndef BALANCED_BEHAVIOR_BEHAVIOR_INLINE_HPP
#define BALANCED_BEHAVIOR_BEHAVIOR_INLINE_HPP


#include <typeinfo>
#include "BEHAVIOR-definition.hpp"
#include "simple_herding.h"
#include "sh_Environment.h"
#include "sh_ForceRules.h"


namespace Balanced {


// TODO remove these constants -- figure out how to get the parameters
//  from config file
inline BEHAVIOR::BEHAVIOR() :
    sheep_separation(3.0f),
    sheep_visibility(3.0f)
{
    // do nothing
}

inline BEHAVIOR::BEHAVIOR(const BEHAVIOR& behavior) :
    shepherds(behavior.shepherds),
    sheep(behavior.sheep),
    sheep_separation(behavior.sheep_separation),
    sheep_visibility(behavior.sheep_visibility),
    targets(behavior.targets),
    waypoints(behavior.waypoints)
{
    // do nothing
}

inline BEHAVIOR::~BEHAVIOR()
{
    // do nothing
}

inline void BEHAVIOR::InitializeParameters()
{
    static bool initialized = false;
    if(initialized == false)
    {
        list<CFlockState*>& va = getEnvironment()->getFlockStates();
        this->shepherds.clear();
        this->sheep.clear();
        for(list<CFlockState*>::iterator i = va.begin(); i != va.end(); i++)
        {
            CFlockState* a = *i;
		    string type = string(typeid(*a->getType()).name());
		    if(type.find("CHerdingFlock") == string::npos)
		    {
		        this->sheep.push_back(a);
		        this->sheep_separation = ((CBasicForceRule*)(a->getForceRule()))->getSeparation() + a->getType()->getGeometry().getRadius();
		        this->sheep_visibility = a->getType()->getViewRadius();
		    }
		    else
		    {
		        this->shepherds.push_back((CHerdingFlockState*)a);
		    }
	    }
	    initialized = true;
    }
}
    

} // end Balanced


#endif // BALANCED_BEHAVIOR_BEHAVIOR_INLINE_HPP


