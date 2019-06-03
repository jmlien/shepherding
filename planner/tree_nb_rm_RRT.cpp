#include "mp.h"
// No Behavior - Random Milestone (Goal)

bool nb_rm_RRT::stop()
{
    //stop if flock separates
	float view_range=getMP()->getFlock().front()->getType()->getViewRadius();
    list<FSLIST> groups=getGroups(getMP()->getFlock(),view_range,getEnvironment());
    if(groups.size()>1) return true; //need to be one group

	//stop if shepherd is far from flock
	FSLIST& shepherds=getMP()->getShepherds();
	pair<float, Point2d> rc=findEC(getMP()->getFlock());
	float max_dist=sqr((rc.first+view_range));
	for(FSLIST::iterator i=shepherds.begin();i!=shepherds.end();i++){
		CHerdingFlockState* shepherd=(CHerdingFlockState*)*i;
		float dist=(shepherd->getPos()-rc.second).normsqr();
		if(dist>max_dist) return true;
	}//end shepherds

    return false;
}

//--------------
//
// no behavior
// &
// random milestone (of the flock) biased toward a WS roadmap node
//
//--------------
bool nb_rm_RRT::findPath(Path& path)
{
    //preparation
    {
        //set behavior rule to dummy behavior, the planner will control shepherd's behavior
        FSLIST& shepherds=getMP()->getShepherds();
        delete shepherds.front()->getType()->getBehaviorRule();
        shepherds.front()->getType()->setBehaviorRule(new CBasicBehaviorRule());

        //make sure right lp is used
        CFlockState * shepherd=getMP()->getShepherds().front();
        LP * lp=getMP()->getLPs().front();
        if(!isClass(lp,"LP_st")){
            cerr<<"! ERROR: LP_st is not used when using nb_rm_RRT"<<endl;
            exit(1);
        }
		lp->setStopFunc(this);

        //make sure right dm is used
        DM * dm=getMP()->getDM();
        if(!isClass(dm,"DM_FlockOnly") && !isClass(dm,"DM_FlockGeo") ){
            cerr<<"! ERROR: DM_FlockOnly or DM_FlockGeo is not used when using db_rm_RRT"<<endl;
            exit(1);
        }
    }

    //run
    bool r=RRT::findPath(path);

    //done
    return r;
}

void nb_rm_RRT::randomize(Cfg& c)
{
    //random milestone near the medial axis
    static const float * bbox=getEnvironment()->getBBX().getBBXValue();
    static float bbox_x=bbox[1]-bbox[0];
    static float bbox_z=bbox[5]-bbox[4];
    static float view_range=getMP()->getFlock().front()->getType()->getViewRadius();

    //get a cd free point
    CFlockState * shepherd=getMP()->getShepherds().front();
    RNG * rng = getMP()->getRNG();
    Point2d goal;
    Vector2d dir;
    float die = rng->uniform();
    if(die<0.25){ // 1 over 4 using goal
        goal=m_goal_pos; 
        dir=Vector2d(rng->uniform(),rng->uniform()).normalize();
    }
    else{ //get a point near MA
        do{
            shepherd->setPos(Point2d(rng->uniform()*bbox_x+bbox[0],rng->uniform()*bbox_z+bbox[4]));
        }while(isCollision(*getEnvironment(),*shepherd));

        //push it to ma
        goal=shepherd->getPos();
        Push2Medial(*getEnvironment(),*shepherd,goal);

        //get dir
        CSimpleHerdingBehaviorRule tmp;
        vector<Point2d> milestones=tmp.findMilestone
                                  (*(CMapFlockState*)shepherd,goal,m_goal_pos,view_range);
        if(milestones.empty()){
            dir=(milestones.front()-goal).normalize();
        }
        else{
            dir=Vector2d(rng->uniform(),rng->uniform()).normalize();
        }
    }


    //setup shepherds' targets
    int size=getMP()->getShepherds().size();
    c.m_shepherd_tar.clear();
    c.m_shepherd_tar.reserve(size);
    static float flock_radius=getMP()->getFlock().front()->getType()->getGeometry().getRadius();
    static float group_radius=flock_radius*sqrt(getMP()->getFlock().size()*2.0f)*2;

    Vector2d f_v = -dir*group_radius;

	if(size==1){
		c.m_shepherd_tar.push_back(goal+f_v);
	}
	else{
		float angle=0.1*PI;
		float theta = ((PI-2*angle)/(size-1));
		int j=0;
		for(int j=0;j<size;j++){
			float phi = theta*j + angle;
			float x2 = cos(phi)*f_v[0] - sin(phi)*f_v[1] + goal[0];
			float y2 = sin(phi)*f_v[0] + cos(phi)*f_v[1] + goal[1];
			c.m_shepherd_tar.push_back(Point2d(x2,y2));
		}//end for
	}

    //let m_shepherd_pos=m_shepherd_tar for finding closest cfg in tree
    c.m_shepherd_pos=c.m_shepherd_tar;
	c.m_flock_tar=goal;
	if(typeid(*getMP()->getDM())==typeid(DM_FlockGeo))
		c.m_cls_vid=((CMapFlockState*)shepherd)->getMap()->closestNode(goal);
}

