#ifndef _SH_COLLISION_DETECTION_H_
#define _SH_COLLISION_DETECTION_H_

#include "sh_Environment.h"
#include "sh_Robot2D.h"
#include "sh_FlockState.h"
#include "sh_ObstState.h"
#include "sh_Obs2D.h"
#include <float.h>
#include <Point.h>
using namespace mathtool;

class shCD
{
public:

    shCD(CEnvironment * env){ m_pEnv=env; Last_ObState=NULL; }
    
    //return true when collision found and information is put in corrosponding variables.
    bool getCollisionInfo
    ( CRobot2D & robot, const Point2d & oldPos, const Point2d & newPos,
      Vector2d & Normal/*out*/, float & dCP/*out, collision point*/ );
    
    //check if the robot in pos is in collision with obstacles
    bool isCollision(CRobot2D & robot, const Point2d & pos );
    
    //check if the straight line between pos1 and pos2 is in collision
    bool isCollision(CRobot2D & robot, const Point2d & pos1, const Point2d & pos2);
    
    //push the pos in to the MA of the free space
    float Push2Medial(CRobot2D & robot, Point2d & pos );
    
    //push the pos in to the free space
    void Push(CRobot2D & robot, Point2d & pos );
    
    //get clearance of the point
    float getClearance(CRobot2D & robot, const Point2d & pos, Point2d & cd_pt);
    
    //check if s is in collision
    bool isCollision(const CFlockState& s);
    
    //check if the straight line between pos1 and pos2 is in collision
    bool isCollision(const CFlockState& s1, const CFlockState& s2);
    
    //check if the straight line between pos1 and pos2 is in collision
    bool isCollision(const CFlockState& s1, const Point2d & pos );
    
    //push the pos in to the MA of the free space
    float Push2Medial(const CFlockState& s1, Point2d & pos );
    
    //push the pos in to the free space
    void Push(const CFlockState& s1, Point2d & pos );
    
    //push the pos in to the free space in dir
    void PushInDir(const CFlockState& s1, const Vector2d& dir, Point2d & pos );
    
    
    //tmp
    bool mychecklinecollision(const Point2d& p1, const Point2d& p2, CFlockState& s);
    //tmp
    
    
    //
    //obst-level collision detection
    //
    float getObstBoundary(CObs2D& obst, list< pair<Point2d,Point2d> >& boundaries);
    
    //obst boundary and its length
    float getObstBoundary(CObs * obst, list< pair<Point2d,Point2d> >& boundaries);
    
    //obst boundary and its length
    float getObstBoundary(list< pair<Point2d,Point2d> >& boundaries);

protected:

	//
	void PushInDir(CRobot2D & robot, const Vector2d& dir, Point2d & pos );
    
    //
    //obst-level collision detection
    //
	bool isCollision(CObs * obst, const CFlockState& s);
   	bool isCollision( CObs * obst, CRobot2D & robot, const Point2d & pos);
    bool isCollision( CObs2D& obst, CRobot2D & robot, const Point2d & pos, bool gatherInfo=false);
    bool isCollision(CObs2D& obst, RAPID_model * line, const Point2d & pos1, 
                     const Point2d & pos2, bool gatherInfo=false);

	bool getCollisionInfo( CObs2D& obst,
	  CRobot2D & robot, const Point2d & oldPos, const Point2d & newPos,
	  Vector2d & Normal/*out*/, float & dCP/*out, collision point*/ );
  
	float getClearance
    ( CObs2D& obst, CRobot2D & robot, const Point2d & pos, Point2d & cd_pt, float clear);
    
    void Push( CObs2D& obst, CRobot2D & robot, Point2d & pos);
    void PushInDir( CObs2D& obst, CRobot2D & robot, const Vector2d& v, Point2d & pos );
    
    //
    //data
    //
    
    CEnvironment * m_pEnv;
    vector<int> CDTriID; //valid iff isCollision return true
    const CObsState * Last_ObState;
    
    //
    //some very low level help functions    
    //
    
    typedef IModel::PtVector  PtVector;
	typedef IModel::TriVector TriVector;
	typedef IModel::Tri       Triangle;

    inline Vector2d Vec2d(const Point2d& pos){ return Vector2d(pos[0],pos[1]); }
    inline Vector3d Vec3d(const Point3d& pos){ return Vector3d(pos[0],pos[1],pos[2]); }
    
    //the square of the distance from pos to p1p2
    inline float distsqr
    (const Point2d& pos, const Point2d& p1, const Point2d& p2, Point2d& cd_pt)
    {
    
        Vector2d n=p1-p2;
        float t=(n*(pos-p1))/(n*(p2-p1));
        if( t>=0 && t<=1 ){
            for(int i=0;i<2;i++) cd_pt[i]=(1-t)*p1[i]+t*p2[i];
            return (pos-cd_pt).normsqr();
        }
        else{ //closest point is end pt
            float d1=(p1-pos).normsqr();
            float d2=(p2-pos).normsqr();
            if( d1<d2 ){ cd_pt=p1; return d1; }
            else { cd_pt=p2; return d2; }
        }
    }
    
    //convert the point P to global
    inline Point2d toGlobal
    (const Point3d& P, const Matrix3x3 & Rot, const Point3d& Pos)
    {
        Point3d tmp = Pos+Rot*Vec3d(P);    //local to world
        return Point2d(tmp[0],tmp[2]);
    }
    
    
    //get the coordinate of an edge of a given triangle
    inline pair<Point2d,Point2d> getEdge
    (int iD, const Triangle& Tri, const PtVector& Geo, 
     const Matrix3x3 & Rot, const Point3d& Pos)
    {
        int vid1=Tri[iD];            //id1 of line end pt
        int vid2=Tri[(iD+1)%3];      //id2 of line end pt
        Point2d tmpp1 = toGlobal(Geo[vid1],Rot,Pos);    //local to world
        Point2d tmpp2 = toGlobal(Geo[vid2],Rot,Pos);    //local to world
    
        return pair<Point2d,Point2d>(tmpp1, tmpp2);
    }
    
    //check if the line segment e1 & e2 intersect, if so report the intersection
    inline Point2d LineSegIntersect
    (const pair<Point2d,Point2d>& e1,const pair<Point2d,Point2d>& e2)
    {
        Vector2d v1=e1.second-e1.first;
        Vector2d v2=e2.second-e2.first;
        Vector2d v3=e1.first-e2.first;
        float a=v1*v1;
        float b=-(v1*v2);
        float c=v2*v2;
        float d=v1*v3;
        float e=-(v2*v3);
        float ac_b2=a*c-b*b;
        if(ac_b2<1e-10) return Point2d(FLT_MAX,FLT_MAX);
        float s=(b*e-c*d)/ac_b2;
        if( s>1 || s<0 ) return Point2d(FLT_MAX,FLT_MAX);
        float t=(b*d-a*e)/ac_b2;
        if( t>1 || t<0 ) return Point2d(FLT_MAX,FLT_MAX);
        return e2.first+v2*(t*1.1f);
    }
    
    inline bool isLineSegIntersect
    (const pair<Point2d,Point2d>& e1,const pair<Point2d,Point2d>& e2)
    {
        Vector2d v1=e1.second-e1.first;
        Vector2d v2=e2.second-e2.first;
        Vector2d v3=e1.first-e2.first;
        float a=v1*v1;
        float b=-(v1*v2);
        float c=v2*v2;
        float d=v1*v3;
        float e=-(v2*v3);
        float ac_b2=a*c-b*b;
        if(ac_b2<1e-10) return false;
        float s=(b*e-c*d)/ac_b2;
        if( s>1 || s<0 ) return false;
        float t=(b*d-a*e)/ac_b2;
        if( t>1 || t<0 ) return false;
        return true;
    }
    
    
    //----------------------------------------------------------------------------
    //   CHECKS IF 2D POINT P IS IN TRIANGLE ABC. RETURNS 1 IF IN, 0 IF OUT
    //   Given a triangle ABC and a point P, determines if P is inside
    //   ABC (regardless the vertex ordering - CCW or CW). 2D version only, but
    //   this handles the 3D case if appropriately projected to the XY, YZ, or XZ
    //   planes (by dropping corresponding component; i.e. drop Z in XY projection).
    //----------------------------------------------------------------------------
    inline bool isCollision
    (const Point2d& A, const Point2d& B, const Point2d& C,const Point2d & P)
    {
        // FIRST CHECK THE SIGN OF THE Z-COMPONENT OF THE NORMAL BY CALCULATING
        // THE CROSS-PRODUCT (ABxBC). THIS WILL DETERMINE THE ORDERING OF THE
        // VERTICES. IF NEGATIVE, VERTICES ARE CLOCKWISE ORDER; OTHERWISE CCW.
        // THEN EVALUATE SIGN OF Z-COMPONENTS FOR ABxAP, BCxBP, and CAxCP TO
        // DETERMINE IF P IS IN "INSIDE" HALF-SPACE FOR EACH EDGE IN TURN ("INSIDE"
        // IS DETERMINED BY SIGN OF Z OF NORMAL (VERTEX ORDERING).
        // NOTE: FULL CROSS-PRODS ARE NOT REQUIRED; ONLY THE Z-COMPONENTS
        Vector2d dAB=B-A, dBC=C-B;  // "REPEATS"
        if ((dAB[0]*dBC[1]-dAB[1]*dBC[0]) < 0) // CW
        {
            if (dAB[0]*(P[1]-A[1]) >= dAB[1]*(P[0]-A[0])) return false;           // ABxAP
            if (dBC[0]*(P[1]-B[1]) >= dBC[1]*(P[0]-B[0])) return false;           // BCxBP
            if ((A[0]-C[0])*(P[1]-C[1]) >= (A[1]-C[1])*(P[0]-C[0])) return false; // CAxCP
        }
        else // CCW
        {
            if (dAB[0]*(P[1]-A[1]) < dAB[1]*(P[0]-A[0])) return false;           // ABxAP
            if (dBC[0]*(P[1]-B[1]) < dBC[1]*(P[0]-B[0])) return false;           // BCxBP
            if ((A[0]-C[0])*(P[1]-C[1]) < (A[1]-C[1])*(P[0]-C[0])) return false; // CAxCP
        }
        return true; // "INSIDE" EACH EDGE'S IN-HALF-SPACE (PT P IS INSIDE TRIANGLE)
    }
    
    inline bool isCollision
    (const Point2d& A, const Point2d& B, const Point2d& C,const Point2d & P,float Rsqr)
    {
        if( isCollision(A,B,C,P) ) return true;
        Point2d tmp;
        if( distsqr(P,A,B,tmp)<Rsqr ) return true;
        if( distsqr(P,B,C,tmp)<Rsqr ) return true;
        if( distsqr(P,C,A,tmp)<Rsqr ) return true;
        return false;
    }
};

#endif //_SH_COLLISION_DETECTION_H_


