#include "multi_herding_rules.h"
#include "multi_herding.h"

Vector2d CMultiHerdingForceRule::getForce(CFlockState & s)
{
	CHerdingFlockState& hs = (CHerdingFlockState&)s; //cast to CHerdingFlockState
    CMultiHerdingFlock * flock=(CMultiHerdingFlock*)s.getType();
    
	float TargetAttract=10;

	//see no flock
	if( hs.flock_group.states.empty() )
		return CBasicForceRule::getForce(s);

	//s is in collision
	shCD cd(getEnvironment());
	if( cd.isCollision(s) )
		return CBasicForceRule::getForce(s);

	locomotionInfo lm(hs);
	lm.R_f=lm.R_f+2.5;
	lm.Vt=lm.Vt*(-1);
	lm.Vc=lm.Vc*(-1);

	Vector2d dir;
	if(hs.isTravelling)
		dir = lm.Vt/lm.D_Vt;
	else if(lm.s.isTurning || lm.s.isApproaching)
		dir=flock->approach->approachingDir(lm);
	else //#steering
		dir=flock->steer->steeringDir(lm);

	//#############################################################################
	return dir*TargetAttract+CBasicForceRule::getForce(s);
}

//-----------------------------------------------------------------------------

CMultiHerdingBehaviorRule::CMultiHerdingBehaviorRule()
{
	chooseTarget = new simpleMinimization();
	distribution = new DistributeShepherdsGroupSizeClose();
	formation    = new circleFormation();

	assert(distribution);
	assert(formation);
	assert(chooseTarget);
}

void CMultiHerdingBehaviorRule::applyRule( CFlockState& s )
{
	CHerdingFlockState& hfs=(CHerdingFlockState&)s;
	if(getSI()->checkReachGoal()) //yes we need check if we reach the goal
		reachGoal(hfs); 

	Point2d target;
	bool r=findTarget(hfs,target);
	if(!r) hfs.target=s.getPos();
	hfs.target=target;

	if(r){
		if(isFarFromTarget(hfs)){
			hfs.isTravelling=true;
            goToTarget(hfs); //reset target
		}
		else{
			herding(hfs);
			hfs.isTravelling=false;
		}
		checkDirection(hfs);
	}
	else{
		hfs.isTravelling=true;
	}

    /*
    //TODO: fix this
	if(getSI()->showVisualHint()){
		CHerdingFlock* hflock=(CHerdingFlock*)s.getType();
		hflock->dmile->path.push_back(hfs.target);
	}
	*/
}

//compute groups, then compute milestone, then find target
bool CMultiHerdingBehaviorRule::findTarget(CHerdingFlockState& s, Point2d& taget)
{
	//#find the group of sheep that the shepherd wants to push
	Point2d targetGoal(0,0);
	FSLIST& vis=s.getVisibleAgent();
	if(vis.empty()) return false;

	FSLIST shepherds, flocks;
	classifyAgents(s,vis,shepherds,flocks);

	//init groups
	vector<FlockGroup> groups;
	createGroups(flocks,groups);

	//-------------- DRAW --------------#
	//todo
	/*
	if(getSI()->showVisualHint()){
		CHerdingFlock* hflock=(CHerdingFlock*)s.getType();
		hflock->dgroup->centers.clear();
		hflock->dgroup->radius.clear();
		int gsize=groups.size();
		for(int i=0;i<gsize;i++) {
			hflock->dgroup->radius.push_back(groups[i].radius);
			hflock->dgroup->centers.push_back(groups[i].center);
		}
	}
	*/
	//#-------------- DRAW --------------#

	// get number of shepherd for each group and groups that has shepherds
	// (in distribute_Shepherds.py)
	distribution->findDistributionPerGroup(shepherds,groups);

	//find milestones for each group (in this file)
    findMilestones(s,groups,1);

	// find all targets using formation (in shepherd_Formation.py)
	formation->findTargetsInFormation(s,groups);

	// find the group and target for this shepherd (in chooseSteeringPoint.py)
	pair<Point2d,FlockGroup> target_group=chooseTarget->choosePoint(s,shepherds,groups);

	taget=target_group.first;
	s.flock_group=target_group.second;
	s.milestones=s.flock_group.milestones;

	return true;
}


// compute milestones for ALL groups
void CMultiHerdingBehaviorRule::findMilestones
(CHerdingFlockState& s, vector<FlockGroup>& groups,float r)
{
	int group_size=groups.size();
	if( group_size==1 || !getSI()->allowRegroup() ){//only one group
		groups[0].milestones=findMilestone(s,groups[0].center,s.goal.getPosition(),r);
	}
	else{//multiple groups
	    //flock separation
		for(int i=0;i<group_size;i++){
			FlockGroup& g=groups[i];
			if(i==0){ // largest
				Vector2d Vg=Vector2d(0,0)-groupVec(g); //negate
				Point2d M1=g.center+Vg*g.radius/2; //stop
				Point2d M2=g.center+Vg*g.radius; //stop
				g.milestones.push_back(M1);
				g.milestones.push_back(M2);
			}
			else{//not the largest
				if(g.shepherd_number==0) continue; // no shepherd....don't need milestone
				else{ // go to the largest group
					g.milestones=findMilestone(s,g.center,groups[0].center,r);
				}
			}
		}//end for
	}//end if
}

// check if s is too far from a group
// R,O are radius and center of the group under consideration
bool CMultiHerdingBehaviorRule::
isFarFromTarget(CHerdingFlockState& s)
{
	return false;
    if(m_cd.isCollision(s,s.target)) return true; //can't see
    return false;
}

// target is too far, use the first milestone
// on the path to target as the target.
void CMultiHerdingBehaviorRule::
goToTarget(CHerdingFlockState& s)
{
	Point2d goal=s.target;
	Point2d start=s.getPos();

	//make sure cd free
	
	CEnvironment * env=getEnvironment();
	CRobot2D& geo=s.getType()->getGeometry();
    if( m_cd.isCollision(geo,start) ){ m_cd.Push(s,start); }
	if( m_cd.isCollision(geo,goal) ){ m_cd.Push(s,goal); }

	//find path to the target
	vector<Point2d> milestones=findMilestone(s,start,goal,5);
	if(!milestones.empty()) s.target=milestones.front();
}


//get an
Vector2d CMultiHerdingBehaviorRule::
groupVec(FlockGroup& g)
{
	Vector2d totalV(0,0);
	for(FSLIST::iterator i= g.states.begin();i!=g.states.end();i++){
		totalV=totalV+(*i)->getVelocity();
	}
	return totalV.normalize();
}

// classify agents into flock and shepherds
void CMultiHerdingBehaviorRule::classifyAgents
(CHerdingFlockState& s, FSLIST& va, FSLIST& shepherds, FSLIST& flocks)
{
	shepherds.push_back( &s );

	for(FSLIST::iterator i=va.begin();i!=va.end();i++){
		CFlockState * a=*i;
		string type=string(typeid(*a->getType()).name());
		if( type.find("HerdingFlock")!=string::npos )
			shepherds.push_back(a);
		else
			flocks.push_back(a);
	}//end
}

//create groups in the flockgroup format
void CMultiHerdingBehaviorRule::
createGroups(FSLIST& flocks, vector<FlockGroup>& groups)
{
	//init groups
	list<FSLIST> _groups=getGroups_CA(flocks,6); //get all visible groups
	groups.reserve(_groups.size());
	for(list<FSLIST>::iterator i=_groups.begin();i!=_groups.end();i++){
		groups.push_back(FlockGroup());
		FlockGroup& g=groups.back();
		pair<float,Point2d> disc= findEC(*i);
		g.states=*i;
		g.radius=disc.first;
		g.center=disc.second;
	}
}

void CMultiHerdingBehaviorRule::
checkTargetAndGroupCenter(CHerdingFlockState& s, Point2d& C_f)
{
	Point2d mypos = s.getPos();
	s.setPos( s.target );
	Point2d t = s.target;

	while ( m_cd.isCollision(s, C_f) ){
		Point2d oldpos = s.getPos();
		Point2d t( (oldpos[0]+C_f[0])/2, (oldpos[1]+C_f[1])/2 );
		s.setPos(t);
		if( (t-C_f).normsqr()< 1 ) //too close
			break;
	}
	//put back
	s.target = t;
	s.setPos( mypos );
}

