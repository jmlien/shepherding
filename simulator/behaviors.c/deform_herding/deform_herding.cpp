#include "deform_herding.h"
#include "dh_rules.h"

bool CDeformHerdingFlock::initialize(list< list<string> >& tokens, flock_raw_data& data)
{
	bool r=CSimpleHerdingFlock::initialize(tokens,data);

	//update force rule
    CSimpleHerdingForceRule * oldrule=(CSimpleHerdingForceRule *)getForceRule();
    CDH_ForceRule * frule = new CDH_ForceRule();
    *frule=*((CDH_ForceRule*)oldrule); //copy data from old rule
    setForceRule(frule);

    //update behavior rule
    delete getBehaviorRule();
    setBehaviorRule(new CDH_BehaviorRule());

	return r;
}


