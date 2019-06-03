#include "scared_flock.h"
#include "shepherding.h"

Vector2d ScaredForceRule::getForce(CFlockState& s)
{
    float magnitude = 0.0;
    Vector2d force (0, 0);
    
	FSLIST shepherds;
    findShepherds(s,shepherds);

    if (shepherds.empty())
    {
        force = CBasicForceRule::getForce(s);    
    }
	else
	{
		Vector2d forceA = scaredForce(s, shepherds);
		magnitude = forceA.norm();
	
	
		float oldMaxForce = getMaxForce();
		setMaxForce( oldMaxForce - magnitude );
		force = forceA;
	
		if (getMaxForce() < 0)
			force = force*(oldMaxForce/magnitude);
		else
			force = force + CBasicForceRule::getForce(s);
	
		setMaxForce(oldMaxForce);
    }
    
    //FOR EXPERIMENT ONLY. PLEASE REMOVE AFTER 2010/1/31
    if(random_force>0){
	    RNG * rng=getRNG();
    	Vector2d rand_vec(rng->uniform(), rng->uniform());
    	if(rng->uniform()>0.5) rand_vec[0]=-rand_vec[0];
       	if(rng->uniform()>0.5) rand_vec[1]=-rand_vec[1];
	    rand_vec=rand_vec.normalize()*random_force;  
	    force=force+rand_vec;
    }
    	
    return force;
}
    


Vector2d
ScaredForceRule::
scaredForce(CFlockState & s, FSLIST& shepherds)
{
	CFlock * flock = s.getType();
	Vector2d forceA(0,0);

	for(FSLIST::iterator i=shepherds.begin();i!=shepherds.end();i++){
		CFlockState* shepherd=*i;
		Vector2d dir = s.getPos() - shepherd->getPos();
		float dist = dir.norm();
		// compute repulsive force
		float weight = 1.0f - (dist / flock->getViewRadius());
		float magnitude = afraid * weight;
		forceA = forceA+magnitude*dir.normalize(); //# afraid force
	}
	
	return forceA;
}



void
ScaredForceRule::
findShepherds(CFlockState & s, FSLIST& shepherds)
{
    list<CFlockState*> visibleAgentList  = s.getVisibleAgent();
    typedef list<CFlockState*>::iterator CF_IT;
    for (CF_IT i = visibleAgentList.begin(); i != visibleAgentList.end(); i++)
    {
        if (*i == NULL)
            continue;
        CFlockState * f = *i;
		if( string(typeid(*f->getType()).name()).find("HerdingFlock")!=string::npos ){
			shepherds.push_back(f);
		}
    }//end for
}



void 
ScaredBehaviorRule::
applyRule( CFlockState & s )
{
    // Ensure that the velocity of this flock member has a magnitude
    // no greater than maxV
    float maxV = 2.5;
    Vector2d velocity = s.getVelocity();
    float vnorm = velocity.norm();
    if (vnorm > maxV)
    {
        velocity = velocity * (maxV / vnorm);
        s.setVelocity(velocity);
    }
}

