#include "mp.h"
// Deterministic Behavior - Random Milestone (Goal)


bool db_rm_RRT::stop()
{
	return isGoal();
	return false;

    //stop if flock separates
	float view_range=getMP()->getFlock().front()->getType()->getViewRadius();
    //list<FSLIST> groups=getGroups(getMP()->getFlock(),view_range,getEnvironment());
    //if(groups.size()>1) return true; //need to be one group


	//stop if close enough
	CHerdingFlockState* shepherd=(CHerdingFlockState*)(getMP()->getShepherds().front());
	pair<float, Point2d> rc=findEC(getMP()->getFlock());
	float dist=(shepherd->goal-rc.second).norm();
    if( dist<rc.first && dist<view_range )
        return true;

    return false;
}


//--------------
//
// deterministic behavior
// &
// random milestone (of the flock) biased toward a WS roadmap node
//
//--------------
//
//bool db_rm_RRT::initialize(list<string>& toks)
//{
//	if(!RRT::initialize(toks))
//		return false;
//    goal_bias=0.5;
//    for(list<string>::iterator i=toks.begin();i!=toks.end();i++){
//        const string& s=*i;
//        if(s=="bias"){
//      		goal_bias=atof((++i)->c_str());
//	  	}
//    }
//    return true;
//}


bool db_rm_RRT::findPath(Path& path)
{
    //preparation
    {
        CFlockState * shepherd=getMP()->getShepherds().front();
        LP * lp=getMP()->getLPs().front();
        if(!isClass(lp,"LP_sb")){
            cerr<<"! ERROR: LP_sb is not used when using db_rm_RRT"<<endl;
            exit(1);
        }
		lp->setStopFunc(this);
        ((LP_sb*)lp)->setBehavior(shepherd->getType()->getBehaviorRule());

        DM * dm=getMP()->getDM();
		if(!isClass(dm,"DM_FlockOnly") && !isClass(dm,"DM_FlockGeo") ){
            cerr<<"! ERROR: DM_FlockOnly or DM_FlockGeo is not used when using db_rm_RRT"<<endl;
            exit(1);
        }
    }

    //run!
    bool r=RRT::findPath(path);

	//post-processing.
	for(FSLIST::iterator i=getMP()->getShepherds().begin();i!=getMP()->getShepherds().end();i++)
        ((CHerdingFlockState*)(*i))->goal=getMP()->getGoal();

    return r;
}

void db_rm_RRT::randomize(Cfg& c)
{
    //random milestone near the medial axis
    static const float * bbox=getEnvironment()->getBBX().getBBXValue();
    static float bbox_x=bbox[1]-bbox[0];
    static float bbox_z=bbox[5]-bbox[4];

    //get a cd free point
    CFlockState * shepherd=getMP()->getShepherds().front();
    RNG * rng = getMP()->getRNG();
    Point2d goal;
    float die = rng->uniform();
    if(die<goal_bias){ // 1 over 2 using goal
        goal=m_goal_pos;
    }
    else{ //get a point near MA
        do{
            shepherd->setPos(Point2d(rng->uniform()*bbox_x+bbox[0],rng->uniform()*bbox_z+bbox[4]));
        }while(isCollision(*getEnvironment(),*shepherd));

        //push it to ma
        goal=shepherd->getPos();
        Push2Medial(*getEnvironment(),*shepherd,goal);
    }

    //set all shepherds goals to be "goal"
    for(FSLIST::iterator i=getMP()->getShepherds().begin();i!=getMP()->getShepherds().end();i++)
        ((CHerdingFlockState*)(*i))->goal=goal;

    //now create data for cfg c; c will be used for finding the closest cfg in the tree
    //but will not be used to expand the tree, which is done using m_behavior
    c.m_flock_tar=goal;
	if(typeid(*getMP()->getDM())==typeid(DM_FlockGeo))
		c.m_cls_vid=((CMapFlockState*)shepherd)->getMap()->closestNode(goal);
}


