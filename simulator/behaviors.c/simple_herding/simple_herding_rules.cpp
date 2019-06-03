#include "simple_herding_rules.h"
#include "simple_herding.h"
#include "shepherding.h"

Vector2d CSimpleHerdingForceRule::getForce(CFlockState & s)
{
	CHerdingFlockState& hs = (CHerdingFlockState&)s; //cast to CHerdingFlockState
	Vector2d V=(hs.target-s.getPos());
	float D=V.norm();

	Vector2d force;

	if(D<0.5) 
		force=CBasicForceRule::getForce(s);
	else
		force=V*(TargetAttract/D)+CBasicForceRule::getForce(s);

	return force;
}


//================================================================================
// new pathfinding section - most of these functions should be moved elsewhere
//================================================================================

inline static float Distance(const Point2d& a, const Point2d& b)
{
   const float x = a[0] - b[0];
   const float y = a[1] - b[1];
   return sqrt(x*x + y*y);
}
   
static Point2d Intersection(const Point2d& point_a, const float slope_a, const Point2d& point_b, const float slope_b)
{
   const float x = (slope_a*point_a[0] - slope_b*point_b[0] - point_a[1] + point_b[1])/(slope_a - slope_b);
   const float y = slope_a*(x - point_a[0]) + point_a[1];
   return Point<float, 2>(x, y);
}

static bool Between(const Point2d& point, const Point2d& left, const Point2d& right)
{
   /*
   Vector2d left_to_right = right - left;
   Vector2d tangent = Vector<float, 2>(left_to_right[1], -left_to_right[0]);
   const float slope = left_to_right[1]/left_to_right[0];
   const float tangent_slope = tangent[1]/tangent[0];
   
   Point2d intersect_left = Intersection(point, slope, left, tangent_slope);
   Vector2d tangent_left = point - intersect_left;
   bool same_side_left = ((tangent_left[0]/left_to_right[0]) > 0.0f);
   
   Point2d intersect_right = Intersection(point, slope, right, tangent_slope);
   Vector2d tangent_right = point - intersect_right;
   bool same_side_right = ((tangent_right[0]/left_to_right[0]) < 0.0f);
   return same_side_left && same_side_right;
   */
   
   return (((left[0] <= point[0] && point[0] <= right[0]) || (right[0] <= point[0] && point[0] <= left[0])) && 
           ((left[1] <= point[1] && point[1] <= right[1]) || (right[1] <= point[1] && point[1] <= left[1])));
}
  
  
//================================================================================
//
// CSimpleHerdingBehaviorRule
//
//================================================================================


CSimpleHerdingBehaviorRule::CSimpleHerdingBehaviorRule()
:m_cd(getEnvironment())
{
}

void CSimpleHerdingBehaviorRule::applyRule( CFlockState& s )
{
	CHerdingFlockState& hfs=(CHerdingFlockState&)s;
	if(getSI()->checkReachGoal()) //see if we need check if we reach the goal
		reachGoal(hfs); 
	
	Point2d target;
	bool r=findTarget(hfs,target);
	if(!r)  target=s.getPos(); 

	hfs.target=target; //# update the target
	
	
	const bool path_blocked = m_cd.isCollision(hfs, hfs.target);
	// if the path is not blocked clear the pathfinding waypoints
	if(path_blocked == false)
	{
	    this->pathfinding_waypoints.clear();
	}
	
	// else, use the pathfinding behavior
	else
	{
	    if(this->pathfinding_waypoints.size() == 0)
	    {
	        PRMS prms(getEnvironment(),getRNG());
	        prms.findPath(*hfs.getMap(), hfs.getPos(), hfs.target, this->pathfinding_waypoints);
	    }
	    if(Between(hfs.getPos(), this->pathfinding_waypoints.front(), *(++this->pathfinding_waypoints.begin())))
	    {
	        pathfinding_waypoints.pop_front();
	    }
	    if(this->pathfinding_waypoints.empty() == false)
	    {
	        hfs.target = this->pathfinding_waypoints.front();
	    }
	}
	 
	
	checkDirection(hfs);
}


void CSimpleHerdingBehaviorRule::reachGoal(CHerdingFlockState& s)
{
	//check if it is taking too long
	long time=getSimulator()->getCurrentTimeStep();
	if(time>=getSI()->getSimBudget()){
		cerr<<"! Failed \n Total Time: "<<time<<endl;
		getSI()->stop();
	}

	//keep trying
	list<CFlockState*>& visible=s.getVisibleAgent();
	
	if(visible.empty()) return; //see nothing?!
	
	for(list<CFlockState*>::iterator i=visible.begin();i!=visible.end();i++)
	{
		CFlockState* f=*i;
		if (dynamic_cast<CHerdingFlockState*>(f) != NULL) continue;
		if( !s.goal.isInGoal(f->getPos()) ){
			return;
		}
	}

	cout<<"Herding: Success!!\n"<<"Total Time: "<<time<<endl;
	getSI()->markSucceeded();
	getSI()->stop();
}

//check the heading dir of the shepherd
void CSimpleHerdingBehaviorRule::checkDirection(CHerdingFlockState& s)
{ 	
	Point2d& goal=s.target;
	Vector2d dir=goal-s.getPos();
	float dist=dir.norm();
	const Vector2d & vec=s.getVelocity();
	if(dir*vec<0 || dist<0.25)  //wrong dir
		truncateVel(s,0.5); //stop it
}

//compute groups, then compute milestone, then find target
bool CSimpleHerdingBehaviorRule::findTarget(CHerdingFlockState& s, Point2d& taget)
{
	//#find the group of sheep that the shepherd wants to push
	Point2d targetGoal(0,0);
	FSLIST& vis=s.getVisibleAgent();
	if(vis.empty()) return false;

	CSimpleHerdingFlock* hflock=(CSimpleHerdingFlock*)s.getType();
	hflock->groups=getGroups_CA(vis,6); //get all visible groups

	list<FSLIST> big_groups=findTargetGroups(s,hflock->groups);

	if(big_groups.empty()) 
		return false; //no biggest group?


	//#------ find next milestone ------#
	pair<float,Point2d> big_disc= findEC(big_groups.front());

	vector<Point2d> milestones; //current and next milestones
	if(big_groups.size()==1 || !getSI()->allowRegroup()){ //  #push flock towards milestone
		milestones=findMilestone(s,big_disc.second,s.goal.getPosition(),5); //# get the next step
		s.flock_group.states=big_groups.front();
	}
	else{  
		//push secondBiggest (g2) towards biggest (g1)
		pair<float,Point2d> smll_disc= findEC(big_groups.back());
		milestones=findMilestone(s,smll_disc.second,big_disc.second,5);
		s.flock_group.states=big_groups.back();
	}

	//#------ find shepherd's targetGoal, i.e. position behind sheep
	if(milestones.empty())
		return false;

	s.milestones=milestones; 

	pair<float,Point2d> my_disc = findEC(s.flock_group.states);
	Vector2d dir = (my_disc.second-milestones[0]).normalize();		
	taget=my_disc.second+dir*(my_disc.first); 


	return true;
}

list<FSLIST> CSimpleHerdingBehaviorRule::findTargetGroups
(CMapFlockState& state, list<FSLIST>& groups)
{
	list<FSLIST> big_groups;
	if(groups.empty()) return big_groups; //nothing to be found
    if( groups.size()==1 ) return groups; //only one group

	typedef list<FSLIST>::iterator IT;
	IT biggestGroup, closestGroup;

	{//find the largest group
		unsigned int biggestSize = 0;
		unsigned int secondBiggestSize = 0;
		for(IT g=groups.begin();g!=groups.end();g++){
			if(g->size() > biggestSize){
				biggestGroup = g;
				biggestSize = g->size();
			}
		}//end for g
	    big_groups.push_back(*biggestGroup);
	}//done finding the largest group
    
	{//not find the closest group to state
		float dist=1e10f;
		for(IT g=groups.begin();g!=groups.end();g++){
			if( g==biggestGroup ) continue;
			//compute distance to 
			//get the location of the largest group
			pair<float,Point2d> disc= findEC(*g);
			float d=(state.getPos()-disc.second).norm()-disc.first;
			if(d<dist){
				dist=d;
				closestGroup=g;
			}
		}
		big_groups.push_back(*closestGroup);
	}

	return big_groups; //(biggestGroup,secondBiggestGroup);
}


//compute milestone of the scared flock
vector<Point2d> CSimpleHerdingBehaviorRule::findMilestone
(CMapFlockState& state, const Point2d& s, const Point2d& g, float r)
{
	CSimpleHerdingFlock* hflock=(CSimpleHerdingFlock*)state.getType();
	list<Point2d>& path=hflock->path;
	path.clear();
	vector<Point2d> milestones;
	
	//check if it is allowed to use medial axis
	if(!getSI()->allowMedialAxis()){
		milestones.push_back(g);
		return milestones;
	}
	
	//make sure that s and g are collision free
	Point2d cd_free_s=s, cd_free_g=g;
	CEnvironment * env=getEnvironment();
	CRobot2D& geo=state.getType()->getGeometry();
	if( m_cd.isCollision(geo,cd_free_s) ){ m_cd.Push(state,cd_free_s); }
	if( m_cd.isCollision(geo,cd_free_g) ){ m_cd.Push(state,cd_free_g); }

	PRMS prm(env,getRNG());
	prm.findPath(*state.getMap(),cd_free_s,cd_free_g,path); 

	float r2=r*r; 
	for(list<Point2d>::iterator p=path.begin();p!=path.end();p++){
		if(milestones.empty()){
			if((*p-cd_free_s).norm()>r) milestones.push_back(*p); 
		}
		else{
			float d2=(*p-milestones[0]).normsqr(); //why 0? why not milestones.back()
			if(d2>r2){
				milestones.push_back(*p); 
				break;  //found two
			}
		}
	}//end for

	if(milestones.empty()) 
		milestones.push_back(cd_free_g);

	return milestones;
}

void CSimpleHerdingBehaviorRule::truncateVel(CFlockState& s, float maxv)
{
    /*
	Vector2d vel=s.getVelocity();
	float vn=vel.norm();
	if(vn>maxv){
		vel=vel*(maxv/vn);
		s.setVelocity(vel);
	}
    */
}//end truncate

