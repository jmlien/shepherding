

#include <cassert>
#include <cmath>
#include <algorithm>
#include <float.h>
#include "Balanced-functions.hpp"
#include "./CIRCLE/CIRCLE.hpp"
#include "./GROUP/GROUP.hpp"

#include "shepherding.h"
#include "shepherding_base.h"

namespace Balanced {


//====================================================
// high-level herding functions
//====================================================
std::vector<GROUP> DefineGroups(const std::vector<CFlockState*>& sheep, const float sheep_separation)
{
    std::vector<CFlockState*> ungrouped_sheep = sheep;
    std::vector<GROUP> groups;
    
    //==========================================
    // TODO : this is REALLY inefficient right now--I'm 
    //  just trying to get the new approach working for
    //  the moment
    //==========================================
    
    while(ungrouped_sheep.size() > 0)
    {
        std::vector<CFlockState*> current_group(1, ungrouped_sheep.back());
        ungrouped_sheep.pop_back();
        int size = ungrouped_sheep.size();
        int i = 0;
        while(i < size)
        {
            current_group.push_back(ungrouped_sheep[i]);
            CIRCLE circle = CIRCLE::Enclosing(current_group);
            if(circle.radius < 1.5f*sheep_separation*sqrt((double)current_group.size()) &&
                LineOfSight(current_group.back()->getPos(), circle.center))
            {
                ungrouped_sheep.erase(ungrouped_sheep.begin() + i);
                size--;
                i = 0;
            }
            else
            {
                current_group.pop_back();
                i++;
            }
        }
        groups.push_back(GROUP::Construct(current_group));
    }
    return groups;
}

// computes a groups destination--when the group reaches that point, the shepherd switches to herding a 
//  different group
// when there is only one group, the destination is environment's final goal
// when there are multiple groups, the destination is the point where the current group should
//  meet up with its neighboring group
Point2d Destination(const GROUP& group, const std::vector<GROUP>& groups, const Point2d& goal, CRoadMap& map)
{
    //================================
    // TODO : this is also really inefficient--need to cache Path() results
    //=================================
    int size = groups.size();
    // if there is only one group, no need to merge
    if(size == 1)
    {
        return goal;
    }
    // otherwise merge group with other nearby group
    std::vector<std::pair<float, const Point2d*> > distances;
    distances.reserve(size);
    for(int i = 0; i < size; i++)
    {
        if(&groups[i] != &group)
        {
            distances.push_back(std::pair<float, const Point2d*>(PathLength(Path(groups[i].center, group.center, map)), &groups[i].center));
        }
    }
    Point2d closest = *std::min_element(distances.begin(), distances.end())->second;
    // next select where the two groups should merge (the waypoint on their path that is closest to the goal)
    std::vector<Point2d> path = Path(group.center, closest, map);
    size = path.size();
    distances.clear();
    distances.reserve(size);
    for(int i = 0; i < size; i++)
    {
        distances.push_back(std::pair<float, const Point2d*>(PathLength(Path(path[i], goal, map)), &path[i]));
    }
    return *std::min_element(distances.begin(), distances.end())->second;
}

// returns a references to whichever group is closest to the shepherd, using paths allowed by the roadmap
const GROUP& ClosestGroupToShepherd(const std::vector<GROUP>& groups, const CHerdingFlockState& shepherd, CRoadMap& map)
{
    const int size = groups.size();
    std::vector<std::pair<float, const GROUP*> > group_distances;
    group_distances.reserve(size);
    
    for(int i = 0; i < size; i++)
    {
        group_distances.push_back(std::pair<float, const GROUP*>(PathLength(Path(groups[i].center, shepherd.getPos(), map)), &groups[i]));
    }
    return *std::min_element(group_distances.begin(), group_distances.end())->second;
}

// returns a references to whichever group is closest to the shepherd, using paths allowed by the roadmap
const GROUP& FarthestGroupFromGoal(const std::vector<GROUP>& groups, const Point2d& goal, CRoadMap& map)
{
    const int size = groups.size();
    std::vector<std::pair<float, const GROUP*> > group_distances;
    group_distances.reserve(size);
    
    for(int i = 0; i < size; i++)
    {
        group_distances.push_back(std::pair<float, const GROUP*>(PathLength(Path(groups[i].center, goal, map)), &groups[i]));
    }
    return *std::max_element(group_distances.begin(), group_distances.end())->second;
}


// used to see if sheep are close enough to the current goal
bool WithinRange(const GROUP& group, const Point2d& goal, const float sheep_separation)
{
    const int size = group.sheep.size();
    // max_distance = (2*separation*sqrt(size))^2
    const float max_distance = 1.5f*sheep_separation*sheep_separation*size;
    for(int i = 0; i < size; i++)
    {
        if((group.sheep[i]->getPos() - goal).normsqr() > max_distance)
        {
            return false;
        }
    }
    return true;
}


//====================================================
// low-level herding functions
//====================================================
// single shepherd version
Point2d Herd(const CHerdingFlockState& shepherd, const std::vector<CHerdingFlockState*> shepherds, const GROUP& group, const Point2d& goal, const float sheep_visibility)
{
    const CFlockState& target_sheep = SelectTargetSheep(shepherd, shepherds, group.sheep, goal);
    
    Point2d target = target_sheep.getPos() + (target_sheep.getPos() - goal).normalize()*0.2f*sheep_visibility;
    
    float angle = AngleBetween(shepherd.getPos() - target_sheep.getPos(), target_sheep.getPos() - goal);
    bool clear_path;
    if(fabs(angle) < 0.5f*M_PI || group.sheep.size() < 3)
    {
        clear_path = true;
    }
    else
    {
        clear_path = ClearPathThroughSheep(shepherd.getPos(), target, group.sheep, goal, sheep_visibility);
    }
    
    return RotateTargetBehindSheep(shepherd.getPos(), target, group.sheep, goal, sheep_visibility, clear_path, angle);
}


Point2d CurrentGoal(const GROUP& group, const Point2d& goal, CRoadMap& map)
{
    std::vector<Point2d> path = Path(group.center, goal, map);
    const Point2d& current_goal = path[1];
    const int size = group.sheep.size();
    for(int i = 0; i < size; i++)
    {
        if(LineOfSight(group.sheep[i]->getPos(), current_goal) == false)
        {
            std::vector<Point2d> new_path = Path(group.sheep[i]->getPos(), goal, map);
            return new_path[1];
        }
    }
    return current_goal;
}


const CFlockState& SelectTargetSheep(const CHerdingFlockState& shepherd, const std::vector<CHerdingFlockState*>& shepherds, const std::vector<CFlockState*>& sheep, const Point2d& goal)
{
    const int number_of_sheep = sheep.size();
    const int number_of_shepherds = shepherds.size();
    assert(number_of_sheep > 0);
    std::vector<std::pair<float, CFlockState*> > sheep_distances;
    sheep_distances.reserve(number_of_sheep);
    
    // create a vector of distance to the goal & sheep pairs
    for(int i = 0; i < number_of_sheep; i++)
    {
        sheep_distances.push_back(std::pair<float, CFlockState*>(-(goal - sheep[i]->getPos()).normsqr(), sheep[i]));
    }
    // sort the sheep, going farthest from the goal to closest
    sort(sheep_distances.begin(), sheep_distances.end());
    
    // copy the vector of sheep, to keep track of which ones have not yet been assigned a target sheep
    // they are also stored in a float,shepherd pair, so that they can be sorted by distance to the sheep
    std::vector<std::pair<float, CHerdingFlockState*> > available_shepherds(number_of_shepherds);
    for(int i = 0; i < number_of_shepherds; i++)
    {
        available_shepherds[i] = std::pair<float, CHerdingFlockState*>(0.0f, shepherds[i]);
    }
    
    CFlockState* target_sheep;
    
    // keep assigning sheep until every shepherd has target, or the current shepherd's target is found
    for(int i = 0; i < number_of_shepherds; i++)
    {   
        // sheep are sorted by distance from goal, and are assigned shepherds in that order
        // if there are more shepherds than sheep, the counter will wrap around and assign
        //   a single sheep to multiple shepherds
        CFlockState* priority_sheep = sheep_distances[i%number_of_sheep].second;
        const int size = available_shepherds.size();
        
        // for each shepherd, calculate its distance to the current highest priority sheep
        for(int j = 0; j < size; j++)
        {
            available_shepherds[j].first = -(priority_sheep->getPos() - available_shepherds[j].second->getPos()).normsqr();
        }
        // sort the shepherds based on this distance
        sort(available_shepherds.begin(), available_shepherds.end());
        
        // give the current high priority sheep the shepherd closest to it
        CHerdingFlockState& closest_shepherd = *available_shepherds.back().second;
        available_shepherds.pop_back();
        
        // if the closest shepherd is the one were selecting a target for, break from the loop
        if(&closest_shepherd == &shepherd)
        {
            target_sheep = priority_sheep;
            break;
        }
    }
    return *target_sheep;
}

// adjusts the shepherds target to make sure it approaches the sheep from directly behind, without interfering
// with any other sheep
Point2d RotateTargetBehindSheep(const Point2d& shepherd, const Point2d& target, const std::vector<CFlockState*>& sheep, 
    const Point2d& goal, const float sheep_visibility, 
    const bool clear_path, const float behind_angle)
{
    CIRCLE group = CircleAroundGroup(sheep);
    if(clear_path == false)
    {
        if(Between(shepherd, group.center, goal) == true)
        {
            group.radius += sheep_visibility;
        }
        Vector2d center_to_shepherd = (shepherd - group.center).normalize()*group.radius;
        Vector2d center_to_target = target - group.center;
        const float angle = AngleBetween(center_to_shepherd, center_to_target);
        center_to_target = RotateVector(center_to_shepherd, 0.3f*angle/fabs(angle));
        
        return group.center + center_to_target;
    }
    else
    {
        if(fabs(behind_angle) < 0.3f)
        {
            return target;
        }
        else
        {
            const float scalar = 0.25f + 0.75f*(fabs(behind_angle)*2.0f/M_PI);
            Point2d sheep_position = target + ((goal - target).normalize()*sheep_visibility*0.25f);
            Vector2d sheep_to_shepherd = (shepherd - sheep_position).normalize()*scalar*sheep_visibility;
            Vector2d sheep_to_target = RotateVector(sheep_to_shepherd, 0.3f*behind_angle/fabs(behind_angle));
            return sheep_position + sheep_to_target;
        }
    }
}

// - Test if the shepherd can go in a straight path to its target,
//  without pushing any sheep away from the goal
// - This is done by finding the intersection of the shepherd's line to the target
//  with the visibility circles of each sheep
// - If any intersections occur between the sheep and their goal, the path is blocked
// - Intersections behind the sheep (which push them towards the goal) are acceptable
bool ClearPathThroughSheep(const Point2d& shepherd, const Point2d& target, const std::vector<CFlockState*>& sheep, const Point2d& goal, const float sheep_visibility)
{
    const int size = sheep.size();
    CIRCLE sheep_circle;
    sheep_circle.radius = sheep_visibility;
    const Vector2d shepherd_to_target = (target - shepherd);
    
    for(int i = 0; i < size; i++)
    {   
        sheep_circle.center = sheep[i]->getPos();
        std::vector<Point2d> intersection = sheep_circle.Intersection(shepherd, shepherd_to_target);
        for(int j = 0; j < 2; j++)
        {
            if((intersection[j] - shepherd)[0]/shepherd_to_target[0] > 0.0f &&
                Between(intersection[j], shepherd, sheep[i]->getPos()) &&
                Between(intersection[j], sheep[i]->getPos(), goal))
            {
                return false;
            }
        }
    }
    return true;
} 

CIRCLE CircleAroundGroup(const std::vector<CFlockState*>& sheep)
{
    assert(sheep.size() > 0);
    const int size = sheep.size();
    float diameter = 0.0f;
    int max_a = 0, max_b = 0;
    for(int i = 0; i < size; i++)
    {
        for(int j = i + 1; j < size; j++)
        {
            const float distance = Distance(sheep[i]->getPos(), sheep[j]->getPos());
            if(distance > diameter)
            {
                diameter = distance;
                max_a = i;
                max_b = j;
            }
        }
    }
    const float center_x = 0.5f*(sheep[max_a]->getPos()[0] + sheep[max_b]->getPos()[0]);
    const float center_y = 0.5f*(sheep[max_a]->getPos()[1] + sheep[max_b]->getPos()[1]);
    return CIRCLE(Point2d(center_x, center_y), 0.5f*diameter);
}

struct FIND_BOTTOM_RIGHT {
        bool operator()(CFlockState* const a, CFlockState* const b) const {
            return (a->getPos()[1] < b->getPos()[1]);// ||
                //(a->getPos()[1] == b->getPos()[1] && a->getPos()[0] > b->getPos()[0]));
        }
    };
    

std::vector<Point2d> ConvexHull(const std::vector<CFlockState*>& sheep)
{
    const int size = sheep.size();
    assert(size > 0);
    if(size == 1)
    {
        return std::vector<Point2d>(1, sheep[0]->getPos());
    }
    else if(size == 2)
    {
        Point2d points[] = {sheep[0]->getPos(), sheep[1]->getPos()};
        return std::vector<Point2d>(points, points + 2);
    }
    
    
    FIND_BOTTOM_RIGHT fbr;
    CFlockState* bottom_right = *std::min_element(sheep.begin(), sheep.end(), fbr);
    std::vector<std::pair<float, CFlockState*> > sorted;
    sorted.reserve(size);
    sorted.push_back(std::pair<float, CFlockState*>(0.0f, bottom_right));
    int i;
    for(i = 0; sheep[i] != bottom_right; i++)
    {
        sorted.push_back(std::pair<float, CFlockState*>(
            atan2(bottom_right->getPos()[1] - sheep[i]->getPos()[1], bottom_right->getPos()[0] - sheep[i]->getPos()[0]),
            //(bottom_right->getPos()[0] - sheep[i]->getPos()[0])/(bottom_right->getPos()[1] - sheep[i]->getPos()[1]),
            sheep[i]));
    }
    for(i = i + 1; i < size; i++)
    {
        sorted.push_back(std::pair<float, CFlockState*>(
            atan2(bottom_right->getPos()[1] - sheep[i]->getPos()[1], bottom_right->getPos()[0] - sheep[i]->getPos()[0]),
            //(bottom_right->getPos()[0] - sheep[i]->getPos()[0])/(bottom_right->getPos()[1] - sheep[i]->getPos()[1]), 
            sheep[i]));
    }
    sort(sorted.begin() + 1, sorted.end());
    
    assert(sorted.size() == sheep.size());
    
    std::vector<Point2d> convex_hull;
    convex_hull.reserve(size);
    convex_hull.push_back(sorted[0].second->getPos());
    convex_hull.push_back(sorted[1].second->getPos());
    convex_hull.push_back(sorted[2].second->getPos());
    for(i = 3; i < size; i++)
    {
        while(CounterClockwise(convex_hull[convex_hull.size() - 2], convex_hull.back(), sorted[i].second->getPos()) == false)
        {
            assert(convex_hull.size() > 0);
            convex_hull.pop_back();
        }
        convex_hull.push_back(sorted[i].second->getPos());
    }
    return convex_hull;
}

//====================================================
// environment functions
//====================================================
bool LineOfSight(const Point2d& a, const Point2d& b)
{
    CRobot2D robot;
    return !(isCollision(*getEnvironment(), robot, a, b));
}

std::vector<Point2d> Path(const Point2d& a, const Point2d& b, CRoadMap& map)
{
    std::list<Point2d> path;
    PRMS prms(getEnvironment());
    prms.findPath(map, a, b, path);
    if(path.size() < 2)
    {
        Point2d array[2] = {a, b};
        return std::vector<Point2d>(array, array + 2);
    }
    // make sure that points a and b are the endpoints of the path--I assumed
    // find path always appends them to the beginning and end, but it looks like
    // this isnt always the case
    if(path.front().almost_equ(a) == false)
    {
        path.push_front(a);
    }
    if(path.back().almost_equ(b) == false)
    {
        path.push_back(b);
    }
    return std::vector<Point2d>(path.begin(), path.end());
}

int FarthestFromGoal(const std::vector<std::vector<CFlockState*> >& groups, const Point2d& goal, CRoadMap& map)
{
    int farthest = 0;
    float max_distance = 0.0f;
    const int size = groups.size();
    for(int i = 0; i < size; i++)
    {
        const float distance = PathLength(Path(groups[i].front()->getPos(), goal, map));
        if(distance > max_distance)
        {
            max_distance = distance;
            farthest = i;
        }
    }
    return farthest;
}

// - groups is a vector of groups of adjacent sheep
// - first_group is an index into the vector, refering to the group we are trying to match
// - this function finds which group in groups[] has the shortest path to groups[first_group], 
//  and returns its index
// - groups must contain at least two groups
int ClosestGroup(const int first_group, const std::vector<std::vector<CFlockState*> >& groups, CRoadMap& map)
{
    int closest = 0;
    float min_distance = FLT_MAX;
    const int size = groups.size();
    for(int i = 0; i < size; i++)
    {
        if(i != first_group)
        {
            const float distance = PathLength(Path(groups[first_group].front()->getPos(), groups[i].front()->getPos(), map));
            if(distance < min_distance)
            {
                min_distance = distance;
                closest = i;
            }
        }
    }
    return closest;
}

int ClosestPoint(const std::vector<Point2d>& points, const Point2d& goal, CRoadMap& map)
{
    int closest = 0;
    float min_distance = FLT_MAX;
    const int size = points.size();
    for(int i = 0; i < size; i++)
    {
        const float distance = PathLength(Path(points[i], goal, map));
        if(distance < min_distance)
        {
            min_distance = distance;
            closest = i;
        }
    }
    return closest;
}
       
//====================================================
// geometric functions
//====================================================
float Distance(const Point2d& a, const Point2d& b)
{
   const float x = a[0] - b[0];
   const float y = a[1] - b[1];
   return sqrt(x*x + y*y);
}   

float PathLength(const std::vector<Point2d>& path)
{
    float length = 0.0f;
    std::vector<Point2d>::const_iterator i = path.begin();
    std::vector<Point2d>::const_iterator end = path.end() - 1;
    
    while(i != end)
    {
        std::vector<Point2d>::const_iterator j = i;
        i++;
        length += Distance(*i, *j);
    }
    return length;
}

Point2d ClosestPointOnLine(const Point2d& point, const Point2d& point_on_line, const Vector2d& line)
{
    const float t = ((point - point_on_line)*line)/(line*line);
    return point_on_line + (line*t);
}

Point2d Intersection(const Point2d& point_a, const float slope_a, const Point2d& point_b, const float slope_b)
{
   const float x = (slope_a*point_a[0] - slope_b*point_b[0] - point_a[1] + point_b[1])/(slope_a - slope_b);
   const float y = slope_a*(x - point_a[0]) + point_a[1];
   return Point<float, 2>(x, y);
}

float AngleBetween(const Vector2d& a, const Vector2d& b)
{
    const float angle = acos((a*b)/(a.norm()*b.norm()));
    if(angle != angle)
    {
        return 0.0f;
    }
    const float cross_product_z = a[0]*b[1] - b[0]*a[1];
    if(cross_product_z > 0.0f)
    {
        return angle;
    }
    return -angle;
}

Vector2d RotateVector(const Vector2d& v, const float angle)
{
    const float cosa = cos(angle);
    const float sina = sin(angle);
    return Vector2d(cosa*v[0] - sina*v[1], sina*v[0] + cosa*v[1]);
}

bool Between(const Point2d& point, const Point2d& left, const Point2d& right)
{
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
}

bool CounterClockwise(const Point2d& a, const Point2d& b, const Point2d& c)
{
    Vector2d previous = b - a;
    Vector2d next = c - b;
    const float z = previous[0]*next[1] - previous[1]*next[0];
    return (z >= -0.05f);
}

float Area(const std::vector<Point2d>& polygon)
{
    const int size = polygon.size();
    float area = 0.0f;
    for(int i = 0, j = 1; j < size; i = j, j++)
    {
        area += (polygon[i][0]*polygon[j][1] - polygon[i][1]*polygon[j][0]);
    }
    return 0.5f*area;
}

Point2d Centroid(const std::vector<Point2d>& polygon)
{
    const int size = polygon.size();
    float area = 0.0f, x = 0.0f, y = 0.0f;
    for(int i = 0, j = 1; j < size; i = j, j++)
    {
        const float cross = (polygon[i][0]*polygon[j][1] - polygon[i][1]*polygon[j][0]);
        area += cross;
        x += (polygon[i][0] + polygon[j][0])*cross;
        y += (polygon[i][1] + polygon[j][1])*cross;
    }
    area *= 0.5f;
    const float centroid_scalar = 1.0f/(6.0f*area);
    return Point2d(x*centroid_scalar, y*centroid_scalar);
}


} // end Balanced




