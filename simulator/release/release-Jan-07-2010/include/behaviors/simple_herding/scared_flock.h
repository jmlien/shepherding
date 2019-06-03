#ifndef SCARED_FLOCK_H
#define SCARED_FLOCK_H

#include "shepherding_base.h"

class ScaredForceRule : public CBasicForceRule
{
    public:
    	
    	ScaredForceRule(){ random_force=0; }
    	
        virtual Vector2d getForce(CFlockState & s);
        Vector2d  scaredForce(CFlockState & s, FSLIST& shepherds);
        void findShepherds(CFlockState & s, FSLIST& shepherds);

        void   setAfraid(float t_afraid)   { afraid = t_afraid; }
        float getAfraid()                  { return afraid;     }

    private:
        float afraid;     // the magnitude of fear for each flock member
        
    public:
        //FOR EXPERIMENT ONLY. PLEASE REMOVE AFTER 2010/1/31
    	float random_force; 
};


class ScaredBehaviorRule : public CBehaviorRule
{
    virtual void applyRule( CFlockState& s );
};


#endif // SCARED_FLOCK_H
