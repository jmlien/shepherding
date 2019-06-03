#ifndef SHEPHERD_FORMATION_H
#define SHEPHERD_FORMATION_H

#include "shepherding_base.h"
#include "simple_herding.h"


class circleFormation
{
public:

	void findTargetsInFormation(CFlockState& s, vector<FlockGroup>& groups)
	{
		computeTargetRefs(groups);

		//loop through each group and find targets
		int gsize=groups.size();
		for(int i=0;i<gsize;i++){
			FlockGroup& g=groups[i];
			//float angle=0.1*PI; //?

			if( g.shepherd_number==0 ) continue;  //no shepherd
			if( g.shepherd_number==1 ){ //one shepherd
				g.targets.push_back(g.targetRef);
			}
			else{ //more than one shepherds
				//if( g.shepherd_number==2 ) angle=0.28*PI; //??
				computeTargets(g);
			}
		}//end for i
	}

	// compute target points for a group
	// M,R,C are milestone, radius, center of the group under consideration
	//void computeTargets(self,s_size,angle,M,R,C){
	void computeTargets(FlockGroup& g)
	{
		float angle=0.1*PI; //?
		if( g.shepherd_number==2 ) angle=0.28*PI; //??

		Vector2d dir = g.milestones[0]-g.center;
		// larget group -> milestone
		Vector2d target_dir=dir.normalize();

		// Vector perpendicular to the direction vector
		Vector2d target2_dir(-target_dir[1],target_dir[0]);
		
		float theta = ((PI-2*angle)/(g.shepherd_number-1));
	    Point2d sTP = g.center+target2_dir*(g.radius+1);
		Vector2d f_v = sTP-g.center;

		int j=0;
		for(int j=0;j<g.shepherd_number;j++){
			float phi = theta*j + angle;
			float x2 = cos(phi)*f_v[0] - sin(phi)*f_v[1] + g.center[0];
			float y2 = sin(phi)*f_v[0] + cos(phi)*f_v[1] + g.center[1];
			g.targets.push_back(Point2d(x2,y2));
		}//end for
	}
		
	// compute target references for all groups
	void computeTargetRefs(vector<FlockGroup>& groups){
		int gsize=groups.size();
		for(int i=0;i<gsize;i++){
			FlockGroup& g=groups[i];
			if( g.shepherd_number==0 ) continue; //not need to compute target ref
			//targetGoal=Point2d(0,0);
			Vector2d dir = (g.center-g.milestones[0]).normalize();		
			g.targetRef= g.center + dir*(g.radius+1);
		}//end for
	}//end
};

/*
//another formaiton
class lineFormation(circleFormation):

	def findTargetsInFormation(self,s,groups):
     		self.computeTargetRefs(groups);
		for g in groups:
			s_size=g.shepherd_number;
			if s_size==0: continue;  #no shepherd
			if(s_size==1): #one shepherd
				g.targets.append(g.targetRef);
			else:#more than one shepherds
				g.targets=self.computeTargets(s_size,g.milestones[0],g.radius,g.center);

	# compute target points for a group
	# M,R,C are milestone, radius, center of the group under consideration
	def computeTargets(self,s_size,M,R,C):
		dir = M-C;
		# larget group -> milestone
		target_dir = Vector2d( dir.x,dir.y );
		target_dir = target_dir.normalize();
		# Vector perpendicular to the direction vector
		target2_dir = Vector2d(-dir.y,dir.x);
		target2_dir = target2_dir.normalize();
		
		listTarget=[];
		
		d = 2*R;
		eps = R*0.05;
		sep = (d-2*eps)/(s_size-1);
		sTP = C+(target_dir)*(-1*(R+1) ) + target2_dir*(-1*(R-eps) );
		f_v = sTP - C;
		j=0;
		while(j < s_size):
			tp = sTP + target2_dir*(j*sep);
			listTarget.append(tp);
			j = j + 1;
		return listTarget;

	# compute target references for all groups
	def computeTargetRefs(self,groups):
		for g in groups:
			if g.shepherd_number==0: continue; # not need to compute target ref
			targetGoal=Point2d(0,0);
			dir = (g.center-g.milestones[0]).normalize();		
			g.targetRef= g.center + dir*(g.radius+1);
		*/

#endif //SHEPHERD_FORMATION_H

