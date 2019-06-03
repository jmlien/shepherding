#ifndef BALANCED_HERDING_H
#define BALANCED_HERDING_H 

#include "simple_herding.h"
#include "Balanced.hpp"

class BalancedHerding : public SimpleHerding 
{
protected:
	bool initialize(list< list<string> >& tokens, flock_raw_data& data)
	{
		bool r=SimpleHerding::initialize(tokens,data);
		if (allowAutonomy()) {
			this->m_shepherd->setBehaviorRule(new Balanced::BEHAVIOR());
			CSimpleHerdingForceRule * frule = new Balanced::FORCE();
	        setupBasicForceRule(frule,data);
			this->m_shepherd->setForceRule(frule);
		}
		return r;
	}
};

#endif
