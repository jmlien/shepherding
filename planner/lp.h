#ifndef _JML_LP_H_
#define _JML_LP_H_


//--------------
//
// Local Planer
//
//--------------


#include <list>
#include <string>
using namespace std;

class Cfg; //defined in cfg.h

class LP
{
public:

	LP(){ ptr_stat_fun=NULL; ptr_stop_fun=NULL; m_total_sim_steps=0; }

    //create one of the cfg connector
    static LP * create(list<string>& toks);

    //initialize itself
    virtual bool initialize(list<string>& toks)=0;
	//return # of simulation time steps
    virtual unsigned int connect(const Cfg& c1, const Cfg& c2, int steps=0, bool useMA=false, bool regroup=true)=0;
	void setStopFunc(sh_stoppable * fun){ ptr_stop_fun=fun; }
	void setStatFunc(sh_statfunc * statF) { ptr_stat_fun=statF; }

	//get total simulated steps
	unsigned int getTotalSimulateStepCount() const { return m_total_sim_steps; }

protected:
	sh_stoppable * ptr_stop_fun;
	sh_statfunc * ptr_stat_fun;
	unsigned int m_total_sim_steps;
};


//--------------
//
// expanding using shepherd targets
//
//--------------


class LP_st : public LP
{
public:
    virtual bool initialize(list<string>& toks);
    virtual unsigned int connect(const Cfg& c1, const Cfg& c2, int steps=0, bool useMA=false, bool regroup=true);
protected:
    static bool stopFunc();
    unsigned int m_timesteps; //number of time steps for simulation
};


//--------------
//
// expanding using shepherd behaviors
//
//--------------


class LP_sb : public LP_st
{
public:
    virtual unsigned int connect(const Cfg& c1, const Cfg& c2, int steps=0, bool useMA=false, bool regroup=true);
	void setBehavior(CBehaviorRule * br){ m_rule=br; }
protected:
    static bool stopFunc();
	CBehaviorRule * m_rule;
};




#endif
