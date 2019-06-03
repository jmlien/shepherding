#ifndef _SH_FLOCKFUNC_H_
#define _SH_FLOCKFUNC_H_

#include <list>
using namespace std;

#include <Point.h>
using namespace mathtool;

class CFlockState;
class CEnvironment;

bool visible(CFlockState& s, const Point2d& pt, CEnvironment* env=NULL);
bool visible(CFlockState& s1, CFlockState& s2, CEnvironment* env=NULL);

typedef std::list<CFlockState*> FSLIST;
std::list<FSLIST> 
getGroups(FSLIST& fslist, float dist=-1, CEnvironment * env=NULL);

//get groups with compact zone
std::list<FSLIST> 
getGroups_CA(FSLIST& fslist, float scale=1, CEnvironment * env=NULL);

//compute the center and radius of a set of flock states
pair<float, Point2d> findEC(FSLIST& states);

//all group safe zone approach
Point2d allGroupsSafeZone
(CFlockState& s, const Point2d& goal,
 const std::list<Point2d>& centers, 
 const std::list<float>& radii);

#endif //_SH_FLOCKFUNC_H_

