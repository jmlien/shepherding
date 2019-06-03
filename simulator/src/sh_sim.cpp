/*
 * sh_sim.cpp
 *
 * implements sh_sim.h
 * 
 * a gui-less simulation
 * 
 * gui-based simulation derives this class
 * 
 */

///////////////////////////////////////////////////////////////////////
// main header file
#include "shepherding_base.h"

shSimulate::shSimulate()
{
    pEnv=NULL;
    pSolver=NULL;
    pLogger=NULL;
    m_validated=false;
    CurTimeStep=0;
    rng = new P_RNG(time(NULL)); // default
    assert(rng);
}


bool shSimulate::initialize()
{
    return true;
}

///////////////////////////////////////////////////////////////////////
//simulate until stopped
unsigned int shSimulate::simulate()
{
    assert(checkOnce());
    while(true)
    {
        CurTimeStep++;
        if(pLogger!=NULL) pLogger->UpdateLog(); 
        if(checkStopFunctions()) //stop!
            return CurTimeStep;     
        pSolver->updateState();
        //if(startFunc)
        //    startFunc->start();
    }
}

///////////////////////////////////////////////////////////////////////
unsigned int shSimulate::simulate(unsigned int time_steps)
{
    assert(checkOnce());
    ///////////////////////////////////////////////////////////////////////
    //simulate for "time_steps" iterations
    for(unsigned int i=0;i<time_steps;i++){
        CurTimeStep++;
        if(pLogger!=NULL) pLogger->UpdateLog();
        if(checkStopFunctions()) //stop!
            return i;   
        pSolver->updateState();
        //if(startFunc)
        //    startFunc->start();
    }

    return time_steps;
}

///////////////////////////////////////////////////////////////////////
void shSimulate::simulateOnce()
{
    assert(checkOnce());
    CurTimeStep++;
    if(pLogger!=NULL) pLogger->UpdateLog();
    pSolver->updateState();
}

///////////////////////////////////////////////////////////////////////
bool shSimulate::checkOnce()
{
    if(!m_validated){
        if( pEnv==NULL ){
            cerr<<"Error : No Environment."<<endl;
            return false;
        }
        
        if( pSolver==NULL){
            cerr<<"Error: No Solver"<<endl;
            return false;
        }
        
        if( pEnv->getFlocks().empty() ){
            cerr<<"Error : No Flock in Environment."<<endl;
            return false;
        }
    
        pSolver->setEnvironment(pEnv);
        
        if(!stopFunc.empty()) 
            if(stopFunc.back()==&m_must_stop) 
                stopFunc.pop_back();
        
        m_validated=true;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////
bool shSimulate::checkStopFunctions()
{
    for(list<sh_stoppable*>::iterator i=stopFunc.begin();i!=stopFunc.end();i++)
        if( (*i)->stop() ) return true;
    return false;
}
