// ParticleSolver.cpp: implementation of the CParticleSolver class.
//
//////////////////////////////////////////////////////////////////////

#include "sh_ParticleSolver.h"
#include "sh_FlockState.h"
#include "sh_Environment.h"
#include "sh_CollisionDetection.h"
#include "sh_ForceRules.h"
#include "sh_LocalInfo.h"
#include "sh_BehaviorRules.h"
#include "../../behaviors.c/movable_obstacle/intersection.h"

#include <iostream>
#include <cassert>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CParticleSolver::CParticleSolver(CEnvironment * pEnv)
{
    m_StateSize=0;
    m_TimeStep=0.1f;
    m_Restitution=0.8f;
    m_pEnv=pEnv;
}

CParticleSolver::~CParticleSolver()
{
    m_TimeStep=0;
    m_Restitution=0;
    m_pEnv=NULL;
    dealloc();
}

//////////////////////////////////////////////////////////////////////
//Core
//////////////////////////////////////////////////////////////////////

bool CParticleSolver::updateState()
{
	assert(m_pEnv);

    //get states
    list<CFlockState*>& states=m_pEnv->getFlockStates();

    //copy the old states
    copy(states);

	//collect neighboring info
    getLocalInfo(states,*m_pEnv);

    //apply rules
    applyRules(states); 

    //Compute paticle states
    //solveODE(states,m_TimeStep/2);
    solveODE(states,m_TimeStep);

    //resolve collision
    CollisionCheck(states);

    //update orientation
    updateOrientation(states);

    return true;
}

//////////////////////////////////////////////////////////////////////
// Solve Diff Equ

void CParticleSolver::solveODE
(list<CFlockState*>& states, float step)
{
    derive(states);
    //applyFlockStateCollisions(states);

    typedef list<CFlockState*>::iterator SIT;
    int id=0;
    for(SIT is=states.begin();is!=states.end();is++,id++){
        Point2d  pos = m_oldPos[id]+m_Derive_Vel[id]*step;
        Vector2d vec = m_oldVel[id]+m_Derive_Acc[id]*step;
        (*is)->setPos(pos);
        (*is)->setVelocity(vec);
    }
}

//compute derivtives of given state
void CParticleSolver::derive
(list<CFlockState*>& states)
{
    typedef list<CFlockState*>::iterator SIT;

    {int id=0;
     for(SIT is=states.begin();is!=states.end();is++,id++){
        m_Derive_Vel[id]=(*is)->getVelocity();
    }}

    //compute acceleration
    {int id=0;
     for(SIT is=states.begin();is!=states.end();is++,id++){
        float mass=(*is)->getType()->getMass();
        CForceRule * frule=(*is)->getForceRule();
        Vector2d F;
        if( frule!=NULL ) F=frule->getForce(**is);
        m_Derive_Acc[id]=F/mass;
    }}
}

/*
void CParticleSolver::applyFlockStateCollisions(list<CFlockState*>& states)
{
    list<CFlockState*>::iterator i = states.begin(), j = i, end = states.end();
    int id_i = 0, id_j = 0;
    for(; i != end; i++, id_i++)
    {
        j = i;
        j++;
        id_j = id_i + 1;
        for(; j != end; j++, id_j++)
        {
            std::vector<Point2d> collision = intersections(**i, **j);
            for(int k = 0, size = collision.size(); k < size; k++)
            {
                const Point2d position_i = (*i)->getPos(), position_j = (*j)->getPos();
                const Vector2d collision_vector_i = collision[k] - position_i, collision_vector_j = collision[k] - position_j;
                m_oldVel[id_i] = m_oldVel[id_i] - parallelComponent(m_oldVel[id_i], collision_vector_i);
                m_oldVel[id_j] = m_oldVel[id_j] - parallelComponent(m_oldVel[id_j], collision_vector_j);
                m_Derive_Vel[id_i] = m_Derive_Vel[id_i] - parallelComponent(m_Derive_Vel[id_i], collision_vector_i);
                m_Derive_Vel[id_j] = m_Derive_Vel[id_j] - parallelComponent(m_Derive_Vel[id_j], collision_vector_j);
                m_Derive_Acc[id_i] = m_Derive_Acc[id_i] - parallelComponent(m_Derive_Acc[id_i], collision_vector_i);
                m_Derive_Acc[id_j] = m_Derive_Acc[id_j] - parallelComponent(m_Derive_Acc[id_j], collision_vector_j);
            } 
        }
    }
}
*/

///////////////////////////////////////////////////////////////////////////////
// Rules

void CParticleSolver::applyRules(list<CFlockState*>& states)
{
    typedef list<CFlockState*>::iterator SIT;
    for( SIT is=states.begin();is!=states.end();is++ ){
        CBehaviorRule * brule=(*is)->getBehaviorRule();
        if( brule!=NULL ) brule->applyRule(**is);
    }   
}

///////////////////////////////////////////////////////////////////////////////
// Collision Detection and Checking
void CParticleSolver::CollisionCheck(list<CFlockState*>& states)
{
    typedef list<CFlockState*>::iterator SIT;

    int id=0;
    for( SIT is=states.begin();is!=states.end();is++,id++ ){
        CollisionCheck(**is,m_TimeStep,id);
    }
}

void CParticleSolver::CollisionCheck
(CFlockState& curS, float step, int id)
{

 /*
    //check current status
	if( step<0.001 )
        return; //too small to deal with

    //Check collision
    if( isCollision(curS, m_oldPos[id]) ){
        if( m_Derive_Vel[id]*n_CDNormal>0 )
            return;
        float collideTime = m_dCP*step;
        float remainTime = step-collideTime;
        reflect(curS,collideTime,remainTime,id);
		CollisionCheck(curS, remainTime, id);
    }
*/
	shCD cd(m_pEnv);
	
	if( isCollision(curS, m_oldPos[id]) )
		reflect(curS,0,step,id);

	if( cd.isCollision(curS)){
			Point2d pos=curS.getPos();
			cd.Push(curS,pos);
			curS.setPos(pos);
	}

    return;
}

bool CParticleSolver::
isCollision(CFlockState& curS, Point2d& old_pos)
{
    Vector2d Normal; float dCP=1;
    shCD cd(m_pEnv);
    CRobot2D& r=curS.getType()->getGeometry();
    
    if( !cd.getCollisionInfo(r,old_pos,curS.getPos(),Normal,dCP) )
        return false; //not colliding
    n_CDNormal=Normal;
    m_dCP = dCP;
    return true;
}

void CParticleSolver::reflect
(CFlockState& curS,float collidTime,float remainTime,int id)
{
    //update old stuff
    Vector2d & V=m_Derive_Vel[id];
    Point2d & P=m_oldPos[id];

    P=P+V*collidTime; //collision point

    //reflective velocity
    Vector2d prjVector=(V*n_CDNormal)*n_CDNormal; //the ray project to plane normal
    Vector2d paraVector=V-prjVector; //the ray parallel cd plane
    V=(-m_Restitution)*prjVector+paraVector;

    //update the state
    curS.setPos(P+V*remainTime);
    curS.setVelocity(V);
}

///////////////////////////////////////////////////////////////////////////////

void CParticleSolver::updateOrientation(list<CFlockState*>& states)
{
    typedef list<CFlockState*>::iterator SIT;
	int id=0;
    for(SIT i=states.begin();i!=states.end();i++,id++){
		(*i)->updateRot_using_V();
        //(*i)->updateRot_using_oldP(m_oldPos[id]);
    }
}

///////////////////////////////////////////////////////////////////////////////

void CParticleSolver::copy(list<CFlockState*>& states)
{
    alloc(states.size());
    typedef list<CFlockState*>::iterator SIT;
    int id=0;
    for(SIT i=states.begin();i!=states.end();i++,id++){
        m_oldPos[id]=(*i)->getPos();
        m_oldVel[id]=(*i)->getVelocity();
    }
}

void CParticleSolver::alloc( int size )
{
    if( size==m_StateSize ) return;

    dealloc();
    m_Derive_Vel=new Vector2d[size];
    m_Derive_Acc=new Vector2d[size];
    m_oldPos=new Point2d[size];
    m_oldVel=new Vector2d[size];
	m_StateSize=size;

    if( m_oldVel==NULL||m_oldPos==NULL||
        m_Derive_Acc==NULL||m_Derive_Vel==NULL){
        cerr<<"Error : Not enough memory"<<endl;
        exit(1);
    }
}

void CParticleSolver::dealloc()
{
    m_StateSize=0;
    delete [] m_Derive_Vel;
    delete [] m_Derive_Acc;
    delete [] m_oldPos;
    delete [] m_oldVel;
}

