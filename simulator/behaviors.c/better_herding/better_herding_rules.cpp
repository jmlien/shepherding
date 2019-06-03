#include "better_herding_rules.h"
#include "better_herding.h"
#include "shepherdLocomotion.h"

//-----------------------------------------------------------------------------
Vector2d CBetterHerdingForceRule::getForce(CFlockState & s)
{
    CHerdingFlockState& hs = (CHerdingFlockState&)s; //cast to CHerdingFlockState
    CBetterHerdingFlock * flock=(CBetterHerdingFlock*)s.getType();
    
    float TargetAttract=10;

    Vector2d force;
    if( hs.flock_group.states.empty() )
        force=CBasicForceRule::getForce(s);
    else{
        locomotionInfo lm(hs);
        lm.R_f=lm.R_f+2.5;
        lm.Vt=lm.Vt*(-1);
        lm.Vc=lm.Vc*(-1);

        Vector2d dir;
        if(lm.s.isTurning || lm.s.isApproaching)
            dir=flock->approach->approachingDir(lm);
        else //#steering
            dir=flock->steer->steeringDir(lm);

        //#############################################################################
        force=dir*TargetAttract+CBasicForceRule::getForce(s);
    }

    return force;
}


//-----------------------------------------------------------------------------

CBetterHerdingBehaviorRule::CBetterHerdingBehaviorRule()
{
}

void CBetterHerdingBehaviorRule::applyRule( CFlockState& s )
{
    CHerdingFlockState& hfs=(CHerdingFlockState&)s;

    if(getSI()->checkReachGoal()) //yes we need check if we reach the goal
        reachGoal(hfs);

    Point2d target;
    bool r=findTarget(hfs,target);
    if(!r) target=s.getPos();
    hfs.target=target;

    if(r){
        herding(hfs);
        checkDirection(hfs);
    }
}

void CBetterHerdingBehaviorRule::herding(CHerdingFlockState& s)
{
    if(s.flock_group.states.empty()) return; //nothing to do
    locomotionInfo lm(s);
    CBetterHerdingFlock * flock=(CBetterHerdingFlock*)s.getType();

    if( flock->approach->isApproaching(lm) ){
        flock->approach->approachingPt(lm);
    }

    if( flock->turn->isTurning(lm) ){
        flock->turn->turningPt(lm);
    }
    else if(flock->steer->isSteering(lm)){
        flock->steer->steeringPt(lm);
    }

    //make the target collision free
    CRobot2D& geo=s.getType()->getGeometry();
    if( m_cd.isCollision(geo,s.target) ){ m_cd.Push(s,s.target); }

    return;
}

