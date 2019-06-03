
#include "mp.h"
#include "sh_CollisionDetection.h"
//create DM instance

LP * LP::create(list<string>& toks)
{
    if(toks.empty()) return NULL;
    string label = toks.front();
    toks.pop_front();

    LP * tmp=NULL;
    if(label=="st" || label=="ST")
        tmp=new LP_st();
	else if(label=="sb" || label=="SB")
        tmp=new LP_sb();
    else{
        cerr<<"! Error: Unknow Local Planner: "<<label<<endl;
        return false;
    }

    if(!tmp->initialize(toks)){
        delete tmp;
        return false;
    }

    //done
    return tmp;
}

//-------------------------------------------------------------------
//
// expanding using shepherd targets
//
//-------------------------------------------------------------------

//initialize itself
bool LP_st::initialize(list<string>& toks)
{
    m_timesteps=10;
	if(toks.empty()) return true; //done

    string label = toks.front();
    toks.pop_front();
    if(label=="t" || label=="T"){
        if(!toks.empty()){
            m_timesteps=(unsigned int)atoi(toks.front().c_str());
            toks.pop_front();
        }//got m_timesteps
    }
    else{
        cerr<<"! Error: Unknow LP_st Parameter: "<<label<<endl;
        return false;
    }

    return true;
}

unsigned int LP_st::connect(const Cfg& c1, const Cfg& c2, int steps, bool useMA, bool regroup)
{
	//if steps<=0, use default
	if(steps<=0) steps=m_timesteps;

	//turn off/on some simulation features
	if(useMA) getSI()->enableMedialAxis();
	else getSI()->disableMedialAxis();
	if(regroup) getSI()->enableRegroup();
	else getSI()->disableRegroup();

    //setup simulator
    toFlockState(c1);

    //setup shepherds' targets
    typedef vector<Point2d>::const_iterator IT;
    FSLIST::iterator sh_it=getMP()->getShepherds().begin();
    for(IT i=c2.m_shepherd_pos.begin();i!=c2.m_shepherd_pos.end();i++,sh_it++){
        CHerdingFlockState * s=(CHerdingFlockState*)(*sh_it);
        s->target=*i; //set s' target to c2's position
    }

    //run simulator
    unsigned int t=sh_simulate(steps,ptr_stop_fun,ptr_stat_fun);
	m_total_sim_steps+=t;
	return t;
}


//-------------------------------------------------------------------
//
// expanding using shepherd behaviors
//
//-------------------------------------------------------------------


unsigned int LP_sb::connect(const Cfg& c1, const Cfg& c2, int steps, bool useMA, bool regroup)
{
	//if steps<=0, use default
	if(steps<=0) steps=m_timesteps;

	//turn off/on some simulation features
	if(useMA) getSI()->enableMedialAxis();
	else getSI()->disableMedialAxis();
	if(regroup) getSI()->enableRegroup();
	else getSI()->disableRegroup();

    //setup simulator
    toFlockState(c1);

    //setup shepherds' behavior
	CFlock* f=getMP()->getShepherds().front()->getType();
	CBehaviorRule * old_beh=f->getBehaviorRule();
	f->setBehaviorRule(m_rule);

    //run simulator
    unsigned int t=sh_simulate(steps,ptr_stop_fun,ptr_stat_fun);
	m_total_sim_steps+=t;

	//setback
	f->setBehaviorRule(old_beh);

	//done
	return t;
}




