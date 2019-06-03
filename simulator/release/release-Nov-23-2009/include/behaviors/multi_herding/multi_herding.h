#ifndef MULTI_HERDING_H
#define MULTI_HERDING_H

#include "better_herding.h"

class MultiHerding : public BetterHerding
{
protected:
	bool initialize(list< list<string> >& tokens, flock_raw_data& data);
};

#endif
