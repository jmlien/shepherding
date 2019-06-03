#include "dh_rules.h"
#include "shepherding.h"
#include "simple_herding.h"

//-----------------------------------------------------------------------------
Vector2d CDH_ForceRule::getForce(CFlockState & s)
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


//-----------------------------------------------------------------------------

dhGrid * CDH_BehaviorRule::m_grid=NULL;

CDH_BehaviorRule::CDH_BehaviorRule()
{
}

void CDH_BehaviorRule::applyRule( CFlockState& s )
{
	CHerdingFlockState& hfs=(CHerdingFlockState&)s;

	if (m_grid == NULL){ //create the grid if necessary
		m_grid = new dhGrid();
		assert(m_grid);
		bool r = m_grid->initialize();
		assert(r);

		//create draw objects
		if (getSI()->showVisualHint()){

			sh_Draw * renderer = getSimulator()->getRenderer();
			if (renderer != NULL){
				if (renderer->isCompatible(m_grid)){
					renderer->addDrawObj(m_grid);
					drawCells * drawTarget = new drawCells(m_grid, m_targetcells, Point3d(1, 0, 0));
					drawCells * drawSheep = new drawCells(m_grid, m_sheepcells, Point3d(0, 1, 0));
					drawCells * drawShepherd = new drawCells(m_grid, m_shepherdcells, Point3d(0, 0, 1));
					renderer->addDrawObj(drawTarget);
					renderer->addDrawObj(drawSheep);
					renderer->addDrawObj(drawShepherd);
				}
			}
		}
	}//m_grid == NULL

	static Point2d catched_goal(FLT_MAX, FLT_MAX);

	//we will only work with the shepherd with min id
	if (m_new_targets_available.find(s.getID()) == m_new_targets_available.end() || m_new_targets_available[s.getID()] == false)
	{ 
		if(getSI()->checkReachGoal()) //yes we need check if we reach the goal
			reachGoal(hfs);
		
		//check if dist2goal needs to be updated
		if(hfs.goal.getPosition()!=catched_goal){
			m_grid->clearDist2Goal();
			m_grid->computeDist2Goal(hfs.goal.getPosition());
			catched_goal=hfs.goal.getPosition();
		}

		//now everything below is done every time step
		//this will find new targets for all shepherds
		findTargets(hfs);
	}
	
	//find path to the taget
	const int min_shepherd_target_dist=4;
	Point2d target=hfs.target;
	if( (hfs.getPos()-target).normsqr()>min_shepherd_target_dist ){
		CellIDList path;
		bool found=m_grid->findPath(hfs.getPos(),target,path);
		for(CellIDList::iterator i=path.begin();i!=path.end();i++){
			Point2d pos=m_grid->getPos(*i);
			if( (hfs.getPos()-pos).normsqr()<min_shepherd_target_dist ) continue;
			hfs.target=pos;
			break;
		}//end for
		if(!found) hfs.target=hfs.getPos();
	}
	m_new_targets_available[s.getID()] = false;//this target is used.
}


//compute groups, then compute milestone, then find target
bool CDH_BehaviorRule::findTargets(CHerdingFlockState& s)
{
	
	//get sheep
	FSLIST& vis=s.getVisibleAgent();
	if(vis.empty()) return false; //nothing to do...

	FSLIST shepherds, sheep;
	classifyAgents(s,vis,shepherds,sheep);

	//unmark everything
	unmarkTargetCells();
	unmarkSheepCells();
	unmarkShepherdCells();
	
	//leading sheep
	Point2d peak=findLeadingSheepPos(sheep);
	//Push2Medial(*getEnvironment(),s,peak);
	
	
	//make cells
	markTargetCells(peak,sheep.size()*2);
	markSheepCells(sheep);
	markShepherdCells();

	//
	// assign targets
	//
	assignTarget2Shepherds(shepherds,peak);

	//mark the availability
	for (auto& shepherd : shepherds) this->m_new_targets_available[shepherd->getID()] = true;
		
	return true;
}

//given the milestone find 
//a cloesest sheep that is in a free cell...
Point2d CDH_BehaviorRule::findLeadingSheepPos(FSLIST& sheep)
{
	FSLIST::iterator best=sheep.end();
	float min_dist=FLT_MAX;
	for(FSLIST::iterator i=sheep.begin();i!=sheep.end();i++){
		CFlockState * s=*i;
		dhGrid::cell& c=m_grid->get(s->getPos());
		if(!c.m_free) continue;
		if(min_dist>c.m_dist2goal){
			min_dist=c.m_dist2goal;
			best=i;
		}
	}//end for
	
	return (*best)->getPos();
}


//there are cells that we should push sheeps into
void CDH_BehaviorRule::markTargetCells(const Point2d& pos, int size)
{
	CellIDList open;
	int cell_count=0;
	CellID id=m_grid->getID(pos);
	open.push_back(id);
	
	while(!open.empty()){
		//get the first id
		CellID id=open.front();
		open.pop_front();
		
		//add to the list
		dhGrid::cell& c=m_grid->get(id);
		if(c.m_target) continue; //already added
		cell_count++; 
		c.m_target=true; 
		m_targetcells.push_back(id);

		if( cell_count==size) return; //I am done here
		
		//expand
		pair<int,int> nid;
		for(int i=-1;i<=1;i++) {
			int x=id.first+i;
			if(x<0||x>=m_grid->width()) continue;
			for(int j=-1;j<=1;j++){
				if(i==0&&j==0) continue; //this is c
				int y=id.second+j;			
 			    if(y<0||y>=m_grid->height()) continue;
 			    nid=pair<int,int>(x,y);
 			    dhGrid::cell& nc=m_grid->get(nid);

 			    //already visited or not free...
 			    if(nc.m_target || (!nc.m_free) ) continue;
				open.push_back(nid);

			}//end j
		}//end i
		
		
	}//end while
}

void CDH_BehaviorRule::unmarkTargetCells()
{
	for(CellIDList::iterator i=m_targetcells.begin();i!=m_targetcells.end();i++)
		m_grid->get(*i).m_target=false;
	m_targetcells.clear();
}

//these are the cells that sheephs are in
//excluding those that are in the target cells
void CDH_BehaviorRule::markSheepCells(FSLIST& sheep)
{
	//
	static int expand_size=0;
	if(expand_size==0){
		float cellsize=sheep.front()->getType()->getGeometry().getRadius()*2;
		float viewrange=sheep.front()->getType()->getViewRadius();
		expand_size=(int)floor((viewrange/cellsize)*2.0/4);
	}
	
	int expand_size_sqr=expand_size*expand_size;
	for(FSLIST::iterator i=sheep.begin();i!=sheep.end();i++){
		CFlockState * s=*i;
		CellID id=m_grid->getID(s->getPos());
		dhGrid::cell& c=m_grid->get(id);
		if(c.m_target) continue;
		
		c.m_sheep=true;
		m_sheepcells.push_back(id);
		
		CellIDList open;
		open.push_back(id);
		while(!open.empty()){
			CellID tmp=open.front();
			open.pop_front();
			//expand
			for(int i=-1;i<=1;i++){
				for(int j=-1;j<=1;j++){
					CellID nid(tmp.first+i,tmp.second+j);
					if(nid.first<0 || nid.first>=m_grid->width()) continue;
					if(nid.second<0 || nid.second>=m_grid->height()) continue;
					dhGrid::cell& c=m_grid->get(nid);
					if(!c.m_free || c.m_sheep || c.m_target) continue;
					int dx=nid.first-id.first;
					int dy=nid.second-id.second;
					if(dx*dx+dy*dy>expand_size_sqr) continue; //too far
					c.m_sheep=true;
					m_sheepcells.push_back(nid);
					open.push_back(nid);
				}
			}
		}//end while
		
		/*
		//expand
		for(int i=-expand_size;i<=expand_size;i++){
			for(int j=-expand_size;j<=expand_size;j++){
				CellID nid(id.first+i,id.second+j);
				if(nid.first<0 || nid.first>=m_grid->width()) continue;
				if(nid.second<0 || nid.second>=m_grid->height()) continue;
				
				dhGrid::cell& c=m_grid->get(nid);
				if(c.m_target || !c.m_free || c.m_sheep) continue;
				int dx=nid.first-id.first;
				int dy=nid.second-id.second;
				if(dx*dx+dy*dy>expand_size_sqr) continue; //too far
				c.m_sheep=true;
				m_sheepcells.push_back(nid);
			}
		}
		*/
	}//end for
}

void CDH_BehaviorRule::unmarkSheepCells()
{
	for(CellIDList::iterator i=m_sheepcells.begin();i!=m_sheepcells.end();i++)
		m_grid->get(*i).m_sheep=false;
	m_sheepcells.clear();
}

//these are the cells that shepherd should
//place themselves in
void CDH_BehaviorRule::markShepherdCells()
{
	for(CellIDList::iterator i=m_sheepcells.begin();i!=m_sheepcells.end();i++){
		CellID& sheep=*i;
		dhGrid::cell& c=m_grid->get(sheep);
		if(c.m_target) continue; //already in the target
		
		/*
		float min_dist2goal=FLT_MAX;
		CellID target;
		
		for(int x=-1;x<=1;x++){
			for(int y=-1;y<=1;y++){
				CellID nid(i->first+x,i->second+y);
				if(nid.first<0 || nid.first>=m_grid->width()) continue;
				if(nid.second<0 || nid.second>=m_grid->height()) continue;
				dhGrid::cell& c=m_grid->get(nid);
				if(!c.m_free) continue; //not free
				//
				if(min_dist2goal>c.m_dist2goal){
					min_dist2goal=c.m_dist2goal;
					target=nid;
				}
			}//end for y
		}//end for x
		*/
		
		CellID target=findClosest(sheep);

		//
		//find an empty cell around the sheep to push to the target
		int diff_x=target.first-sheep.first;
		int diff_y=target.second-sheep.second;
		
		//compute shepherd
		CellID shepherd=sheep;
		if(diff_x!=0)
			if(diff_x>0) shepherd.first=sheep.first-1;
			else shepherd.first=sheep.first+1;

		if(diff_y!=0)
			if(diff_y>0) shepherd.second=sheep.second-1;
			else shepherd.second=sheep.second+1;

		bool valid=validate_and_add_ShepherdCell(shepherd);
		
		//if not valid, lets try other options
		if(diff_x!=0 && diff_y!=0 && !valid){
			CellID test1=shepherd; 
			CellID test2=shepherd; 
			test1.first =sheep.first; 
			test2.second=sheep.second;
			validate_and_add_ShepherdCell(test1);
			validate_and_add_ShepherdCell(test2);
		}

	}//end for i
}


void CDH_BehaviorRule::unmarkShepherdCells()
{
	for(CellIDList::iterator i=m_shepherdcells.begin();i!=m_shepherdcells.end();i++)
		m_grid->get(*i).m_shepherd=false;
	m_shepherdcells.clear();
}

//get closest target cell for a sheeph cell
CDH_BehaviorRule::CellID 
CDH_BehaviorRule::findClosest(CellID& sheepcell) const
{
	CellIDList::const_iterator best=m_targetcells.end();
	float min_dist=FLT_MAX;
	
	for(CellIDList::const_iterator i=m_targetcells.begin();i!=m_targetcells.end();i++)
	{
		const CellID& t=*i;
		int x=abs(t.first-sheepcell.first);
		int y=abs(t.second-sheepcell.second);
		int d=x*x+y*y;
		if(d<min_dist){
			min_dist=d;
			best=i;
		}
	}//end for i
	
	return *best;
}

bool CDH_BehaviorRule::validate_and_add_ShepherdCell(CellID& shepherd)
{
	bool valid=true;
	if(shepherd.first<0||shepherd.first>=m_grid->width())    valid=false;
	if(shepherd.second<0||shepherd.second>=m_grid->height()) valid=false;
	if(valid) {
		dhGrid::cell& c=m_grid->get(shepherd);
		if((!c.m_free)||c.m_sheep||c.m_target) 
			valid=false;
		else{
			//add to the list if have not already added
			if(!c.m_shepherd){ 
				c.m_shepherd=true;
				m_shepherdcells.push_back(shepherd);
			}
		}
	}
	
	return valid;
}


// classify agents into flock and shepherds
void CDH_BehaviorRule::classifyAgents
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


//
// divide the targets using angles
// the axis is from the peak to the milesone
// If there are n shepherds then we will create
// n radial bins one for each shepherd
// 

void CDH_BehaviorRule::assignTarget2Shepherds(FSLIST& shepherds, const Point2d& peak)
{	

	//find target for peak...
	//
	Point2d milestone;
	const int min_target_dist=16;
	{
		Point2d goal=((CHerdingFlockState *)(shepherds.front()))->goal.getPosition();
		if( (peak-goal).normsqr()<min_target_dist ) 
			milestone=goal;
		else{
			CellIDList path;
			bool found=m_grid->findPath(peak,goal,path,false);
			for(CellIDList::iterator i=path.begin();i!=path.end();i++){
				Point2d pos=m_grid->getPos(*i);
				if( (peak-pos).normsqr()<min_target_dist ) continue;
				milestone=pos;
				break;
			}//end for
		}
	}
	
	//
	//
	const Vector2d vec=milestone-peak;
	const Vector2d n_vec(-vec[1],vec[0]);

	//
	vector< pair<float,CellID> > sorted_celss;
	for(CellIDList::iterator i=m_shepherdcells.begin();i!=m_shepherdcells.end();i++){
		Point2d pos=m_grid->getPos(*i);
		Vector2d v_pi=pos-peak;
		float dot1=v_pi*vec;
		float dot2=v_pi*n_vec;
		float d=0;
		if(dot2>=0)
			d=1-dot1;
		else//dot2<0
			d=3+dot1;
		sorted_celss.push_back(pair<float,CellID>(d,*i));
	}
	
	//sort
	sort(sorted_celss.begin(),sorted_celss.end());
	
	//partition
	int shepherd_size=shepherds.size();
	int cell_size=sorted_celss.size();
	int res=(int)ceil(cell_size*1.0/shepherd_size);
	
	//
	//pick the furthest shepherd cell as the target
	//
	int s=0;
	int e=s+res;
	list<Point2d> targets;
	for(int i=0;i<shepherd_size;i++){
		if(s>=cell_size) break;
		if(e>cell_size) e=cell_size;
		//
		float max_dist=0;
		Point2d best;
		for(int j=s;j<e;j++){
			CellID id=sorted_celss[j].second; //
			float d=m_grid->get(id).m_dist2goal;
			if(d>max_dist){
				max_dist=d;
				best=m_grid->getPos(id);
			}
		}//end for j
		
		//
		targets.push_back(best);
		
		//
		s=e;
		e=e+res;
	}//end for i
	
	//final assignment
	assignTarget2Shepherds(shepherds, targets);
	//if(max_dist==0) return false;
}


//
// assign targets to the shepherds
//
struct match_shepherd
{
	match_shepherd(){ shepherd=NULL; assigned=false; }
	CHerdingFlockState * shepherd;
	bool assigned;
};

struct match_target
{
	match_target(){ assigned=false; }
	Point2d target;
	bool assigned;
};

struct match_shepherd_target
{
    bool operator<(const match_shepherd_target& o) const { return dist<o.dist; }
	float dist;
	int sid; //shepherd id
	int tid; //target id
};

void CDH_BehaviorRule::assignTarget2Shepherds(FSLIST& shepherds, list<Point2d>& targets)
{	
	vector<match_shepherd>        match_shepherds;
	vector<match_target>          match_targets;
	vector<match_shepherd_target> matches;
	
	//
	int shepherd_size=shepherds.size();
	match_shepherds.reserve(shepherd_size);
	for(FSLIST::iterator i=shepherds.begin();i!=shepherds.end();i++){
		match_shepherd tmp;
		tmp.shepherd=(CHerdingFlockState*)(*i);
		match_shepherds.push_back(tmp);
	}
	//
	int target_size=targets.size();
	match_targets.reserve(target_size);
	for(list<Point2d>::iterator i=targets.begin();i!=targets.end();i++){
		match_target tmp;
		tmp.target=*i;
		match_targets.push_back(tmp);
	}
	//
	for(int i=0;i<shepherd_size;i++){
		for(int j=0;j<target_size;j++){
			match_shepherd_target tmp;
			tmp.sid=i;
			tmp.tid=j;
			tmp.dist=(match_shepherds[i].shepherd->getPos()-match_targets[j].target).normsqr();
			matches.push_back(tmp);
		}//j
	}//i
	//
	sort(matches.begin(),matches.end());
	//
	int total_match_size=shepherd_size*target_size;
	int number_of_match=min(shepherd_size,target_size);
	for(int i=0;i<total_match_size;i++){
		match_shepherd_target & tmp=matches[i];
		if(match_shepherds[tmp.sid].assigned || match_targets[tmp.tid].assigned) 
			continue;
		
		//ok, find an assignment
		match_shepherds[tmp.sid].assigned=true;
		match_targets[tmp.tid].assigned=true;
		match_shepherds[tmp.sid].shepherd->target=match_targets[tmp.tid].target;
		number_of_match--;
		if(number_of_match==0)
			break;
	}//end i...
	//
	//
}

