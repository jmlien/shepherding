#include "mp.h"

TreePMP::~TreePMP()
{}

//create one of the tree-based planners
TreePMP * TreePMP::create(list<string>& toks)
{
	if(toks.empty())
		return NULL;

	string label = toks.front();
	toks.pop_front();
	TreePMP * tmp=NULL;

	if(label=="nbrtRRT")        //no behavior random target
		tmp=new nb_rt_RRT();
	else if(label=="nbrmRRT")   //no behavior random milestone
		tmp=new nb_rm_RRT();
	else if(label=="dbrmRRT")   //deterministic behavior random milestone
		tmp=new db_rm_RRT();
	else if(label=="NaiveEST")
		tmp=new NaiveEST();	    // Naive EST Planner
	else if(label=="BasicEST")
		tmp=new BasicEST();	// Basic EST Planner
	else if(label=="MinEST")
		tmp=new MinEST();   // Min EST Planner
	else if(label=="HeuristicEST")
		tmp=new HeuristicEST(); // Heuristic EST Planner
	else if(label=="None")
		tmp=new Simulation_Only();
	else {
		cerr<<"! Error: Unknown Tree-based Planner: "<<label<<endl;
		return false;
	}

	if(!tmp->initialize(toks)) {
		cout << "Could not initialize selected algorithm" << endl;
		delete tmp;
		return false;
	}

	//done
	return tmp;
}

bool TreePMP::initialize(list<string>& toks)
{
	m_sim_budget=UINT_MAX;
	for(list<string>::iterator i=toks.begin();i!=toks.end();i++) {
		const string& s=*i;
		if(s=="simsteps" || s=="SimSteps")
			m_sim_budget=atoi((++i)->c_str());
	}
	return true;
}


bool TreePMP::isGoal(){
    float view_range=getMP()->getFlock().front()->getType()->getViewRadius();
    list<FSLIST> groups=getGroups(getMP()->getFlock(),view_range,getEnvironment());

    if(groups.size()>1)
		return false; //need to be one group

    //need to be close
    pair<float, Point2d> rc=findEC(getMP()->getFlock());
    float dist=(m_goal_pos-rc.second).norm();

    return (dist<m_goal_rad);
}

//
// Modified from Chris Vo's BiRRT, 2008 Fall
//
// RRT
//

bool RRT::initialize(list<string>& toks)
{
	if(!TreePMP::initialize(toks)) return false;
	k_size=INT_MAX;
    goal_bias=0.5;
	for(list<string>::iterator i=toks.begin();i!=toks.end();i++) {
		const string& s=*i;
		if(s=="k" || s=="K")
			k_size=atoi((++i)->c_str());
        if(s=="bias")
      		goal_bias=atof((++i)->c_str());
	}
	return true;
}


bool RRT::findPath(Path& path)
{
	const int meter_width=20;
	if(!m_quiet){ //blah...
		cout << "Running RRT ["<<k_size<<"] iterations"<< endl;
		cout << "[";
		cout.fill('-');
		cout.width(meter_width);
		cout<<"]"<<endl<< " ";
	}

	int meter=(int)(k_size*1.0/meter_width);
	if(meter==0)
		meter=1;

	// expand the tree k times
	for(int i=0; i<k_size; i++) {

		getMP()->printStatistics();

		//check if we reached the goal
		if(isGoal()) {
			// solution found, go ahead and quit
			if(!m_quiet) cout << "$" <<endl;
			return true;
		}

		//check budget
		unsigned int sim_time=getMP()->getLPs().front()->getTotalSimulateStepCount();
		if(sim_time>=m_sim_budget){ //running out of time
			cout << endl << "- Time Budget ("<<sim_time<<"/"<<m_sim_budget<<") Exceeded!" << endl;
			break;
		}

		//keep expanding
		if( !expanding() ) {
			i--;
			continue;
		}

		if(!m_quiet && i%meter==0 && i!=0) {
			// update progress meter
			cout <<"."<<flush;
		}
	}

	if(!m_quiet) cout << endl;
	return false;
}

bool Simulation_Only::findPath(Path& path)
{
	// just add the goal node to the roadmap
	// and simulate for simsteps time steps.
	Cfg& p = getMP()->getRM()->getData(0);
	Cfg q = p;
	q.m_flock_tar = getMP()->getGoal();

	// local planner setup
	LP * lp = getMP()->getLPs().front();
	CFlockState * shepherd = getMP()->getShepherds().front();
	((LP_sb*) lp)->setBehavior(shepherd->getType()->getBehaviorRule());
	lp->setStopFunc(this);
	lp->setStatFunc(this);

	// set goal to end
	for (FSLIST::iterator i = getMP()->getShepherds().begin(); i
	        != getMP()->getShepherds().end(); i++)
		((CHerdingFlockState*) (*i))->goal = q.m_flock_tar;

	// attempt to connect
	q.m_sim_time_steps=getMP()->getLPs().front()->connect(p, q, m_sim_budget, true);
	fromFlockState(q);
	VID gid=getMP()->getRM()->addnode(q);
	getMP()->getRM()->addedge(0,gid,1);

	if(q.m_sim_time_steps<m_sim_budget){ //that means stop returns true and ends simulation early
		cout<<"Path found after "<<q.m_sim_time_steps<<" steps"<<endl;
		return true;
	}
	else{
		cout<<"Path Not found"<<endl;
		return false;
	}
}

bool Simulation_Only::stop()
{
	return isGoal();
}

void Simulation_Only::stat() 
{
	//initialize
	static int count=0;
	static float minDistSqr = numeric_limits<float>::max();
	if(count%100==0) {
		Cfg q;
		fromFlockState(q);
	
		//compute geodesic distance
		CMapFlockState * shepherd=(CMapFlockState *)(getMP()->getShepherds().front());
		list<Point2d> path;
		PRMS * prm=((CHerdingFlock*)shepherd->getType())->prm;
		prm->findPath(*shepherd->getMap(),q.m_flock_cen,getMP()->getGoal(),path); 
		float dist=::pathLength(path);

	    //check if we are making progress
		if (dist<minDistSqr){
			minDistSqr = dist;
			printf("\nExpansion: %d  DistanceToGoal: %f\n", count, minDistSqr);
		}
	}
	count++;
}

