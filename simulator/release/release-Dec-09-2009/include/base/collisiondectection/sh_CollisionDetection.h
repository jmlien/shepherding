#ifndef _SH_COLLISION_DETECTION_H_
#define _SH_COLLISION_DETECTION_H_

#include "sh_Environment.h"
#include "sh_Robot2D.h"
class CFlockState;
class CObs2D;

//return true when collision found and information is put in corrosponding variables.
bool getCollisionInfo
( CEnvironment& env, 
  CRobot2D & robot, const Point2d & oldPos, const Point2d & newPos,
  Vector2d & Normal/*out*/, float & dCP/*out, collision point*/ );

//check if the robot in pos is in collision
bool isCollision( CEnvironment& env, CRobot2D & robot, const Point2d & pos );

//check if the straight line between pos1 and pos2 is in collision
bool isCollision
(CEnvironment& env, CRobot2D & robot, const Point2d & pos1, const Point2d & pos2);

//push the pos in to the MA of the free space
float Push2Medial
(CEnvironment& env, CRobot2D & robot, Point2d & pos );

//push the pos in to the free space
void Push
(CEnvironment& env, CRobot2D & robot, Point2d & pos );

//get clearance of the point
float getClearance
(CEnvironment& env, CRobot2D & robot, const Point2d & pos, Point2d & cd_pt);

float getObstBoundary
(CObs2D& obst, std::list< pair<Point2d,Point2d> >& boundaries);

//obst boundary and its length
float getObstBoundary
(CObs * obst, std::list< pair<Point2d,Point2d> >& boundaries);

//obst boundary and its length
float getObstBoundary
(CEnvironment& env,std::list< pair<Point2d,Point2d> >& boundaries);

bool isCollision(CObs * obst, const CFlockState& s);

//check if the flock in state is in collision
bool isCollision( CEnvironment& env, CRobot2D & robot, const Point2d & pos );

//check if the straight line between pos1 and pos2 is in collision
bool isCollision(CEnvironment& env, const CFlockState& s);

//check if the straight line between pos1 and pos2 is in collision
bool isCollision(CEnvironment& env, const CFlockState& s1, const CFlockState& s2);

//check if the straight line between pos1 and pos2 is in collision
bool isCollision( CEnvironment& env, const CFlockState& s1, const Point2d & pos );

//push the pos in to the MA of the free space
float Push2Medial(CEnvironment& env, const CFlockState& s1, Point2d & pos );

//push the pos in to the free space
void Push(CEnvironment& env, const CFlockState& s1, Point2d & pos );

//push the pos in to the free space in dir
void PushInDir
(CEnvironment& env, const CFlockState& s1, const Vector2d& dir, Point2d & pos );


//tmp
bool mychecklinecollision(const Point2d& p1, const Point2d& p2, CFlockState& s);
//tmp



#endif //_SH_COLLISION_DETECTION_H_


