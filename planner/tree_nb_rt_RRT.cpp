#include "mp.h"
// No Behavior - Random Target

bool nb_rt_RRT::stop()
{
    //stop if flock separates
	float view_range=getMP()->getFlock().front()->getType()->getViewRadius();
    list<FSLIST> groups=getGroups(getMP()->getFlock(),view_range,getEnvironment());
    if(groups.size()>1) return true; //need to be one group


	//stop if shepherd is close enough
	FSLIST& shepherds=getMP()->getShepherds();
	pair<float, Point2d> rc=findEC(getMP()->getFlock());
	float max_dist=sqr(rc.first+view_range);
	for(FSLIST::iterator i=shepherds.begin();i!=shepherds.end();i++){
		CHerdingFlockState* shepherd=(CHerdingFlockState*)*i;
		float dist=(shepherd->getPos()-rc.second).normsqr();
		if(dist>max_dist) return true;
	}//end shepherds

    return false;
}

//--------------
//
// no behavior (shepherd targets from milestone)
// &
// random targets
//
//--------------

bool nb_rt_RRT::findPath(Path& path)
{
    //prepare
    {
        //set behavior rule to dummy behavior, the planner will control shepherd's behavior
        FSLIST& shepherds=getMP()->getShepherds();
        delete shepherds.front()->getType()->getBehaviorRule();
        shepherds.front()->getType()->setBehaviorRule(new CBasicBehaviorRule());

        //make sure right lp is used
        LP * lp=getMP()->getLPs().front();
        if(!isClass(lp,"LP_st")){
            cerr<<"! ERROR: LP_st is not used when using nb_rt_RRT"<<endl;
            exit(1);
        }
		lp->setStopFunc(this);

        //make sure right dm is used
        DM * dm=getMP()->getDM();
        if(!isClass(dm,"DM_ShepherdOnly")){
            cerr<<"! ERROR: DM_ShepherdOnly is not used when using nb_rt_RRT"<<endl;
            exit(1);
        }
    }

    //run
    bool r=RRT::findPath(path);

    //done
    return r;
}

float CLOSEST_DIST=FLT_MAX;

// expanding the tree
bool nb_rt_RRT::expanding() {
    Roadmap * rm = getMP()->getRM();
    DM * dm = getMP()->getDM();

    //random cfg
    Cfg c;
    randomize(c);

    //get "from"
    VID from_id=nearest(c);
    Cfg& from = rm->getData(from_id);

	//if(from_id!=0) cout<<from_id<<endl;

    //expand
    unsigned int steps=getMP()->getLPs().front()->connect(from,c);

    if(steps==0)
		return false;  //not expanded

    //add the end node to the tree
    fromFlockState(c); //update c from flock states
    c.m_sim_time_steps=steps;
    c.m_flock_dir=(c.m_flock_cen-from.m_flock_cen).normalize(); //flock direction
	c.m_in_map=true;
    //cout<<"dir="<<c.m_flock_dir<<endl;
    float dist=dm->distsqr(c,from);
    VID c_id=rm->addnode(c);
    rm->addedge(from_id,c_id,dist);

	//
	//float tmp = (c.m_flock_cen-getMP()->getGoal()).normsqr();
	//if(tmp<CLOSEST_DIST){
	//	CLOSEST_DIST=tmp;
	//	cout<<"["<<c_id<<"] CLOSEST_DIST="<<CLOSEST_DIST<<endl;
	//}
	//

	//cout<<from_id<<"->"<<c_id<<endl;

    return true;
}

// finds nearest node to q
VID nb_rt_RRT::nearest(Cfg& q) {

    // init
    Roadmap * rm = getMP()->getRM();
    DM * dm = getMP()->getDM();
    MPGraph& graph = rm->getGraph();

    float min = FLT_MAX;
    vector<VID>::iterator best;
    Roadmap_Node tmp_node;

    //get all nodes in the roadmap
    vector<VID> all_nodes;
    graph.GetVerticesVID(all_nodes);

//cout<<"-------"<<endl;

    // compute distances, choose max
    for(vector<VID>::iterator i=all_nodes.begin();i!=all_nodes.end();i++) {
        Cfg& q1 = rm->getData(*i);
        float tmp = dm->distsqr(q,q1);

        //cout<<tmp<<endl;

        if(tmp < min) {
            best = i;
            min = tmp;
        }
    }

//cout<<"neareast="<<*best<<" dist="<<min<<endl;

//cout<<"-------"<<endl;

    return *best;
}


//
// TODO: Improve this (currently it randomly set shepherds' positions)
//
void nb_rt_RRT::randomize(Cfg& c)
{
    //generate a set of target positions at random
    static const float * bbox=getEnvironment()->getBBX().getBBXValue();
    static float bbox_x=bbox[1]-bbox[0];
    static float bbox_z=bbox[5]-bbox[4];

    //setup shepherds' targets
    int size=getMP()->getShepherds().size();
    c.m_shepherd_tar.clear();
    c.m_shepherd_tar.reserve(size);

    RNG * rng = getMP()->getRNG();

    for(int i=0;i<size;i++){
        Point2d pos(rng->uniform()*bbox_x+bbox[0],rng->uniform()*bbox_z+bbox[4]);
        c.m_shepherd_tar.push_back(pos);
    }

    //let m_shepherd_pos=m_shepherd_tar for finding closest cfg in tree
    c.m_shepherd_pos=c.m_shepherd_tar;
}
