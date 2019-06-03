#ifndef _DEFORM_HERDING_H
#define _DEFORM_HERDING_H

#include "simple_herding.h"

class CDeformHerdingFlock : public CSimpleHerdingFlock
{
public:
    CDeformHerdingFlock(CEnvironment * env, const string& name):CSimpleHerdingFlock(env,name){}
	bool initialize(list< list<string> >& tokens, flock_raw_data& data);
};

#endif
