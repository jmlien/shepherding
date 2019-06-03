#ifndef SHEPHERD_DISTRIBUTION_H
#define SHEPHERD_DISTRIBUTION_H

#include "sh_ForceRules.h"
#include "sh_BehaviorRules.h"
#include "sh_MapBasedFlockState.h"
#include "func/sh_FlockFunc.h"
#include "simple_herding.h"
#include "sh_dyPRM.h"
#include "sh_CollisionDetection.h"

#include "simple_herding.h"

//# Here the shepherds will be distributed among groups based on
//# the size of each group
//# if num groups is bigger than num shepherds
//# the groups closer to big group center will be assigned a shepherd

class DistributeShepherdsGroupSizeClose
{
public:

	void findDistributionPerGroup(FSLIST& shepherds, vector<FlockGroup>& groups)
	{
		typedef vector<FlockGroup>::iterator IT;
		int shepherd_size = shepherds.size();
		int group_size=groups.size();

		if( groups.size()==1){
			groups.front().shepherd_number=shepherd_size;
	        return;
		}
		else{
			for(int i=0;i<group_size;i++) groups[i].shepherd_number=0;
		}

		if( shepherd_size >= group_size ){ // more shepherds than groups
            
			// compute total number of flock (can't we get this somewhere else?)
			int flock_size = 0; 
			for(IT g=groups.begin();g!=groups.end();g++) flock_size+=g->states.size();

			//assign shepherds to groups
            int sheps_assigned=0;

			for(IT g=groups.begin();g!=groups.end();g++){
				g->shepherd_number=int((group_size/flock_size)*shepherd_size)+1;
                sheps_assigned += g->shepherd_number;
			}
				
			//# assign the rest..	
			if( sheps_assigned < shepherd_size ){
				int diff=shepherd_size-sheps_assigned;
				for(int i=0;i<diff;i++){
					int gid=i%group_size;
					groups[gid].shepherd_number++;
				}//end for
			}
		}
		else{//more group than shepherds
            // decide which group the shepherd should go to based on their distance
			for(int i=0;i<shepherd_size;i++){
                int closest_id = farGroup(groups);		    
				groups[closest_id].shepherd_number=1;
			}
		}//end else
	}

    //assuming groups are sorted from small to large
    int farGroup(vector<FlockGroup>& groups)
	{
    	int closest_id = -1;
        float dist = -1e10f;

	    Point2d pos=groups[0].center;
		int size=groups.size();
		for(int id=0;id<size;id++){
			if(groups[id].shepherd_number>0) continue;
			float d = (groups[id].center-pos).norm()-groups[id].radius;
			if (d>dist){
				dist = d;
                closest_id = id;
			}
		}
		return closest_id;
	}

};

/*

//This is another function from py

# Here the shepherds will be distributed among groups based on
# the size of each group
# based on size only order comes from getcc...I think
class DistributeShepherdsGroupSize:

    # return (1) # of shepherds for each group (2) groups that will have shepherds
    def findDistributionPerGroup(self,shepherds,groups):
        
        if( len(groups) == 1 ):
	    groups[0].shepherd_number=shepherds.size();
            return;
        shepherd_size = shepherds.size();
        if( shepherd_size >= len(groups) ): # more shepherds than groups
            flock_size = 0.0; #total number of flock
            for g in groups: flock_size = flock_size + len(g.members);

	    #assign shepherds to groups
            sheps_assigned=0;
            for g in groups:
                sa = int((len(groups)/flock_size)*shepherd_size)+ 1;
		g.shepherd_number=sa;
                sheps_assigned = sheps_assigned + sa;
  	    # assign the rest..	
            if( sheps_assigned < shepherd_size ):
		for i in range(shepherd_size-sheps_assigned):
                    groups[i].shepherd_number=groups[i].shepherd_number+1;
        else: # more group than shepherds
            for i in range(shepherd_size):
		    groups[i].shepherd_number=1;
*/

#endif //SHEPHERD_DISTRIBUTION_H
