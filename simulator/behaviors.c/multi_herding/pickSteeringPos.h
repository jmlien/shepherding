#ifndef SHEPHERD_PICKSTEERPOS_H
#define SHEPHERD_PICKSTEERPOS_H

#include "shepherding_base.h"
#include "simple_herding.h"

struct S_data // data for shepherd
{
	S_data(CFlockState * _s){ s=_s; assigned=false; }
	CFlockState * s;
	bool assigned;
};

struct T_data // data for a target
{
	T_data(const T_data& other):t(other.t),g(other.g){
		assigned=other.assigned;
	}

	T_data(Point2d _t, FlockGroup& _g):t(_t),g(_g){
		assigned=false;
	}
	void operator=(const T_data& other){
		g=other.g;
		t=other.t;
		assigned=other.assigned;
	}

	FlockGroup& g;
	Point2d t;
	bool assigned;
};

struct STpair
{
	STpair(float d, int s, int t){
		dist=d;
		s_id=s;
		t_id=t;
	}
	float dist;
	int s_id, t_id;
};

bool compareDist(const STpair& i1, const STpair& i2);

class simpleMinimization
{
public:

	pair<Point2d,FlockGroup> //return a target and the associated group
	choosePoint(CHerdingFlockState& shepherd, FSLIST& shepherds, vector<FlockGroup>& groups)
	{

		// find my group and target point by distance
        vector<S_data> S_d;      // used to check if this shepherd is assigned
        vector<T_data> T_d;      // used to check if this target is assigned
        vector<STpair> ST_dist;  // list of all distances for shepherds and targets

		int shepherd_size=shepherds.size();
		// create S_data
		for(FSLIST::iterator s=shepherds.begin();s!=shepherds.end();s++)
			S_d.push_back(S_data(*s));

		// create T_data
		for(vector<FlockGroup>::iterator g=groups.begin();g!=groups.end();g++){
			int tsize=g->targets.size();
			for(int t=0;t<tsize;t++)
				T_d.push_back(T_data(g->targets[t],*g));
		}

		//compute distance
		int target_size=T_d.size();
		for(int sid=0;sid<shepherd_size;sid++){
			for(int tid=0;tid<target_size;tid++){
				float dist = (S_d[sid].s->getPos()-T_d[tid].t).normsqr();
				ST_dist.push_back(STpair(dist,sid,tid));
			}
		}

		//find assignment for s
		sort(ST_dist.begin(),ST_dist.end(),compareDist);
		int my_target_id=-1;
		int st_size=ST_dist.size();
		for(int i=0;i<st_size;i++){
			STpair& st=ST_dist[i];
			//s_id or t_id is no longer availiable
			if( S_d[st.s_id].assigned || T_d[st.t_id].assigned ) continue;
			//OK, s_id/t_id is an assignment
			S_d[st.s_id].assigned=true;
			T_d[st.t_id].assigned=true;

			//check if s is assigned
			if(&shepherd==S_d[st.s_id].s){
				my_target_id=st.t_id;
				break;
			}
		}

		return pair<Point2d,FlockGroup>(T_d[my_target_id].t,T_d[my_target_id].g);
	}
};

/*
#this should be simplified
# this is a very simple method...but this looks very complicate
class lineProjection(simpleMinimization):

    def choosePoint(self,s,shepherds,groups):
        target_dir = groups[0].milestones[0]-groups[0].center;
        target_dir = target_dir.normalize();
        # Compute the equation line for the shepherd position
        # Vector perpendicular to the direction vector
        proj_dir = Vector2d(-target_dir.y,target_dir.x);

		#create T_data
		target_ref_pt=0; #//groups[0].targets[0];
		T_d=[];
		for g in groups:
			for t in g.targets:
				td=T_data(t,g);
				if target_ref_pt==0: target_ref_pt=t;
				td.dist=(t-target_ref_pt)*proj_dir;
				T_d.append(td);
		T_d.sort(compareDist);

		# count how many shepherd are on the negative side of s
		neg_size=0;
		for shep in shepherds:
			if shep==s: continue;
			dist=(shep.getPos()-s.getPos())*proj_dir;
			if dist<0: neg_size=neg_size+1;

		#assign
		return  (T_d[neg_size].target,T_d[neg_size].group);

*/

#endif//SHEPHERD_PICKSTEERPOS_H
