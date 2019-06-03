
#include "multi_herding.h"
#include "multi_herding_rules.h"

bool CMultiHerdingFlock::initialize(list< list<string> >& tokens, flock_raw_data& data)
{
	bool r=CBetterHerdingFlock::initialize(tokens,data);

	//update force rule
	CSimpleHerdingForceRule * oldrule=(CSimpleHerdingForceRule *)getForceRule();
	CMultiHerdingForceRule * frule = new CMultiHerdingForceRule();
	*frule=*((CMultiHerdingForceRule*)oldrule);
	setForceRule(frule);

	//update behavior rule
    delete getBehaviorRule();
    setBehaviorRule(new CMultiHerdingBehaviorRule());

	return r;
}


