#ifndef _DEFORM_HERDING_H
#define _DEFORM_HERDING_H

#include "simple_herding.h"

class DeformHerding : public SimpleHerding
{
protected:
	bool initialize(list< list<string> >& tokens, flock_raw_data& data);
};

#endif
