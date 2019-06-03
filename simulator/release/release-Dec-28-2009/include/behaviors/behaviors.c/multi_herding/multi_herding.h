#ifndef MULTI_HERDING_H
#define MULTI_HERDING_H

#include "better_herding.h"

class CMultiHerdingFlock : public CBetterHerdingFlock
{
public:
    CMultiHerdingFlock(CEnvironment * env, const string& name):CBetterHerdingFlock(env,name){}
	bool initialize(list< list<string> >& tokens, flock_raw_data& data);
};

#endif
