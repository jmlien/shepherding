

#include "BEHAVIOR.hpp"
#include "Balanced.hpp"
#include "CIRCLE/CIRCLE.hpp"
#include "GROUP/GROUP.hpp"
//#include "sh_gui_main.h"
#include "sh_Environment.h"
#include "sh_CollisionDetection.h"


namespace Balanced {


void BEHAVIOR::applyRule(CFlockState& s)
{
    CHerdingFlockState& shepherd = (CHerdingFlockState&)s;
    CRoadMap& map = *shepherd.getMap();
    this->InitializeParameters();
     
    
    std::vector<GROUP> groups = DefineGroups(this->sheep, this->sheep_separation);
    

    const GROUP* current_group = &ClosestGroupToShepherd(groups, shepherd, map);
    Point2d destination = Destination(*current_group, groups, shepherd.goal, map);
    
    if(WithinRange(*current_group, destination, this->sheep_separation) == true)
    {
        current_group = &FarthestGroupFromGoal(groups, shepherd.goal, map);
        destination = Destination(*current_group, groups, shepherd.goal, map);
    }
    
    Point2d goal = CurrentGoal(*current_group, destination, map);
    shepherd.target = Herd(shepherd, this->shepherds, *current_group, goal, this->sheep_visibility);
    
    
    // TODO use higher resolution push so that shepherd can get closer to wall
    if(isCollision(*getEnvironment(), shepherd.getType()->getGeometry(), shepherd.target) == true ||
        LineOfSight(current_group->center, shepherd.target) == false)
    {
        Point2d target = current_group->center;
        Vector2d push = (shepherd.target - current_group->center).normalize()*shepherd.getType()->getGeometry().getRadius();
        while(isCollision(*getEnvironment(), shepherd.getType()->getGeometry(), target) == false)
        {
            shepherd.target = target;
            target = target + push;
        }
        
    }
    
    
    if(LineOfSight(shepherd.getPos(), shepherd.target) == false)
    {
        std::vector<Point2d> path = Path(shepherd.getPos(), shepherd.target, map);
        shepherd.target = path[1];
    }
    
    
    // fix target in case of NaN values
    // TODO still need to find out what is causing the NaN's
    if(shepherd.target[0] != shepherd.target[0] || shepherd.target[1] != shepherd.target[1])
    {
        cout << "resetting target, due to NaN error\n";
        shepherd.target = shepherd.getPos();
    }
    
    
    /*** used to log automatic trials *****************
    if(getSI()->checkReachGoal() == true)
    {
        bool completed = true;
        const int number_of_sheep = this->sheep.size();
        const int max_radius = this->sheep_visibility*this->sheep_visibility;
        for(int i = 0; i < number_of_sheep; i++)
        {
            if((sheep[i]->getPos() - shepherd.goal).normsqr() > max_radius)
            {
                completed = false;
                break;
            }
        }
        if(completed == true)
        {
            std::ofstream fout("AI.txt", ios::app);
            fout << environment_string << " " << getCurrentTimeStep() << "\n";
            fout.close();
            sh_stop();
            exit(0);
        }
    }***********************/
    
		
    // temporary debug-drawing======================================================
    CHerdingFlock* hflock=(CHerdingFlock*)shepherd.getType();
        if(shepherds.size() == 1 || shepherds[0] == &shepherd)
        {
		    hflock->dgroup->centers.clear();
		    hflock->dgroup->radius.clear();
		}
	    hflock->dgroup->centers.push_back(shepherd.target);
	    hflock->dgroup->radius.push_back(1.0f);
		for(int i = 0; i < groups.size(); i++)
		{
		    hflock->dgroup->centers.push_back(groups[i].center);
	        hflock->dgroup->radius.push_back(std::max(groups[i].radius, 1.0f));
	    }
	//==============================================================================
}    
     
    
    
} // end Balanced


