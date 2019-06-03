#include "better_herding.h"
#include "better_herding_rules.h"

bool CBetterHerdingFlock::initialize(list< list<string> >& tokens, flock_raw_data& data)
{
	bool r=CSimpleHerdingFlock::initialize(tokens,data);

	//update force rule
	CSimpleHerdingForceRule * oldrule=(CSimpleHerdingForceRule *)getForceRule();
	CBetterHerdingForceRule * frule = new CBetterHerdingForceRule();
	*frule=*((CBetterHerdingForceRule*)oldrule);
	setForceRule(frule);

	//update behavior rule
	delete getBehaviorRule();
	setBehaviorRule(new CBetterHerdingBehaviorRule());

	// create loco-motion
	// todo: this should be initilizaed from the configuration file
	approach=new approachingSZ();  //approachingDynMap, approachingSZ approachingLine
	steer=new steeringLine();      //steeringLine, steeringSwing
	turn=new turningStop();
	
	assert(approach&&steer&&turn);
	
	return r;
}


