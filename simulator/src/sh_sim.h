/*
 * sh_sim.h
 *
 * 
 * a gui-less simulation
 * 
 * gui-based simulation derives this class
 *
 * Last Major Modification : J-M Lien 12/28/2009
 * 
 */
 
#ifndef _SH_SIMULATE_H_
#define _SH_SIMULATE_H_

///////////////////////////////////////////////////////////////////////////////
#include "RNG.h"
#include "sh_Environment.h"
#include "sh_ParticleSolver.h"
#include <cassert>

//forward declaration 
class sh_Draw;   //defined in sh_drawclass.h
class SimLogger; //defined in logger.h 

//help structures for simualtion
struct sh_stoppable {virtual bool stop(){ return true; }};
struct sh_startfunc {virtual void start(){}};


//simulation class
class shSimulate
{
public:
    
    shSimulate();
    
    // initialize the simulator
    // return true when everything is OK
    virtual bool initialize();

    //core functions
    virtual unsigned int simulate();  
    virtual unsigned int simulate(unsigned int time_steps); //simluate mulitple time steps
    virtual void simulateOnce();  //simluate one time step
    virtual void stopsimulate(){ addStopFunc(&m_must_stop); m_validated=false; }
    
    //access functions
    virtual sh_Draw * getRenderer() { return NULL; } //base class does not support rendering
        
    CEnvironment    * getEnvironment(){ return pEnv; }
    CParticleSolver * getSolver()     { return pSolver; }
    SimLogger *       getLogger()     { return pLogger; }

    void setEnvironment(CEnvironment * e)     { pEnv=e; m_validated=false; }
    void setSolver(CParticleSolver * solver)  { pSolver=solver; m_validated=false; }
    void setLogger(SimLogger * logger)        { pLogger=logger; m_validated=false; }
    
    void addStopFunc(sh_stoppable * stop)   { stopFunc.push_back(stop); }
    void clearStopFunc()                    { stopFunc.clear(); } //remove everything
    void addStartFunc(sh_startfunc * start) { startFunc.push_back(start); }
    
    void setTimeStep( float t )   { assert(pSolver); pSolver->setTimeStep(t); }
    void setRestitution( float r ){ assert(pSolver); pSolver->setRestitution(r); }
    
    //current simulation step count
    long getCurrentTimeStep() const { return CurTimeStep; }

	//random number generator
    RNG * getRNG() {return rng;}
    void setRNG(RNG * r) {
        if(rng)
            delete rng;
        if(r)
            rng = r;
        else
            rng = NULL;
    }
    
protected:

    bool checkOnce();
    bool checkStopFunctions(); //check stopping conditions
    
    CEnvironment    * pEnv;
    CParticleSolver * pSolver;
    SimLogger       * pLogger;
    
    list<sh_stoppable*> stopFunc;
    list<sh_startfunc*> startFunc;
    sh_stoppable m_must_stop; //if this is added to stopFunc list, simulation must stop
    
    // random number generator
    RNG * rng;
    
    bool m_validated;
    long CurTimeStep;
};

#endif //_SH_SIMULATE_H_


