

#ifndef BALANCED_BALANCED_FUNCTIONS_HPP
#define BALANCED_BALANCED_FUNCTIONS_HPP


#include <vector>
#include "WAYPOINT/WAYPOINT.hpp"
class CFlockState;
class CHerdingFlockState;
class CRoadMap;


namespace Balanced {


class CIRCLE;
class GROUP;


//====================================================
// high-level herding functions
//====================================================
std::vector<GROUP> DefineGroups(const std::vector<CFlockState*>& sheep, const float sheep_separation);
Point2d Destination(const GROUP& group, const std::vector<GROUP>& groups, const Point2d& goal, CRoadMap& map);
const GROUP& ClosestGroupToShepherd(const std::vector<GROUP>& groups, const CHerdingFlockState& shepherd, CRoadMap& map);
const GROUP& FarthestGroupFromGoal(const std::vector<GROUP>& groups, const Point2d& goal, CRoadMap& map);
Point2d CurrentGoal(const GROUP& group, const Point2d& goal, CRoadMap& map);
bool WithinRange(const GROUP& group, const Point2d& goal, const float sheep_separation);

//====================================================
// low-level herding functions
//====================================================
Point2d Herd(const CHerdingFlockState& shepherd, const std::vector<CHerdingFlockState*> shepherds, const GROUP& group, const Point2d& goal, const float sheep_visibility);
const CFlockState& SelectTargetSheep(const CHerdingFlockState& shepherd, const std::vector<CHerdingFlockState*>& shepherds, const std::vector<CFlockState*>& sheep, const Point2d& goal);
Point2d RotateTargetBehindSheep(const Point2d& shepherd, const Point2d& target, const std::vector<CFlockState*>& sheep, 
    const Point2d& goal, const float sheep_visibility, 
    const bool clear_path, const float behind_angle);
bool ClearPathThroughSheep(const Point2d& shepherd, const Point2d& target, const std::vector<CFlockState*>& sheep, const Point2d& goal, const float sheep_visibility);
CIRCLE CircleAroundGroup(const std::vector<CFlockState*>& sheep);
std::vector<Point2d> ConvexHull(const std::vector<CFlockState*>& sheep);

//====================================================
// environment functions
//====================================================
bool LineOfSight(const Point2d& a, const Point2d& b);
std::vector<Point2d> Path(const Point2d& a, const Point2d& b, CRoadMap& map);
int FarthestFromGoal(const std::vector<std::vector<CFlockState*> >& group, const Point2d& goal, CRoadMap& map);
int ClosestGroup(const int first_group, const std::vector<std::vector<CFlockState*> >& groups, CRoadMap& map);
int ClosestPoint(const std::vector<Point2d>& points, const Point2d& goal, CRoadMap& map);

//====================================================
// geometric functions
//====================================================
float Distance(const Point2d& a, const Point2d& b);
float PathLength(const std::vector<Point2d>& path);
Point2d ClosestPointOnLine(const Point2d& point, const Point2d& point_on_line, const Vector2d& line);
Point2d Intersection(const Point2d& point_a, const float slope_a, const Point2d& point_b, const float slope_b);
float AngleBetween(const Vector2d& a, const Vector2d& b);
Vector2d RotateVector(const Vector2d& v, const float angle);
bool Between(const Point2d& point, const Point2d& left, const Point2d& right);
bool CounterClockwise(const Point2d& a, const Point2d& b, const Point2d& c);
Point2d Centroid(const std::vector<Point2d>& polygon);


} // end Balanced


#endif // BALANCED_BALANCED_FUNCTIONS_HPP


