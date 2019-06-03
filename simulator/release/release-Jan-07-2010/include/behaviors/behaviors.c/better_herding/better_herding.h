#ifndef BETTER_HERDING_H
#define BETTER_HERDING_H

#include "simple_herding.h"
#include "shepherdLocomotion.h"

class CBetterHerdingFlock : public CSimpleHerdingFlock
{
public:
    CBetterHerdingFlock(CEnvironment * env, const string& name):CSimpleHerdingFlock(env,name)
    {
        approach=NULL;
        steer=NULL; //
        turn=NULL;
    }
    bool initialize(list< list<string> >& tokens, flock_raw_data& data);
    
    approachingLine * approach;  //approachingDynMap, approachingSZ approachingLine
    steeringLine    * steer; //steeringLine, steeringSwing
    turningNo       * turn;
};

#endif
