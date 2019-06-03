#ifndef BETTER_HERDING_H
#define BETTER_HERDING_H

#include "simple_herding.h"

class BetterHerding : public SimpleHerding
{
protected:
	bool initialize(list< list<string> >& tokens, flock_raw_data& data);
};

#endif
