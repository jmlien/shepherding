#ifndef BETTER_SHEPHERD_RULES_H
#define BETTER_SHEPHERD_RULES_H

#include "simple_herding_rules.h"

class CBetterHerdingForceRule : public CSimpleHerdingForceRule
{
public:
    CBetterHerdingForceRule(){TargetAttract=1;}
    virtual Vector2d getForce(CFlockState & s);
};


class CBetterHerdingBehaviorRule: public CSimpleHerdingBehaviorRule
{
public:
    
    CBetterHerdingBehaviorRule();
    virtual void applyRule( CFlockState& s );

protected:

    void herding(CHerdingFlockState& s);
};


#endif // SHEPHERD_RULES_H
