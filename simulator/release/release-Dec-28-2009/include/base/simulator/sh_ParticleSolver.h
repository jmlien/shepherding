// ParticleSolver.h: interface for the CParticleSolver class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PARTICLESOLVER_H__FA0FDB91_2266_4485_A5FD_F94EBE754E55__INCLUDED_)
#define AFX_PARTICLESOLVER_H__FA0FDB91_2266_4485_A5FD_F94EBE754E55__INCLUDED_

//////////////////////////////////////////////////////////////////////
#include <list>
using namespace std;

//////////////////////////////////////////////////////////////////////
#include <Vector.h>
#include <Point.h>
using namespace mathtool;

//////////////////////////////////////////////////////////////////////
class CRobotState;
class CEnvironment;
class CFlockState;

class CParticleSolver  
{
public:
    //////////////////////////////////////////////////////////////////////
    //Constructors/Destructor
    //////////////////////////////////////////////////////////////////////
    CParticleSolver(CEnvironment * pEnv);
    virtual ~CParticleSolver();

    //////////////////////////////////////////////////////////////////////
    //Core
    bool updateState();

    //////////////////////////////////////////////////////////////////////
    //Access
    void setRestitution( float r ) { m_Restitution=r; }
    void setTimeStep( float t ) { m_TimeStep=t; }
    void setEnvironment(CEnvironment * e){ m_pEnv=e; }
//////////////////////////////////////////////////////////////////////
//Protected
protected:

    float m_TimeStep;          //simluation time step
    float m_Restitution;       //collision restitution
    CEnvironment * m_pEnv;

    //////////////////////////////////////////////////////////////////////
    //solve differential equation
    void derive(list<CFlockState*>& states);
    void solveODE(list<CFlockState*>& states, float step);

    //////////////////////////////////////////////////////////////////////

    int m_StateSize;            //the number of elements of following arrays.
    Vector2d * m_Derive_Vel;    //pre-allocated derivitives of state
    Vector2d * m_Derive_Acc;    //pre-allocated derivitives of state
    Point2d  * m_oldPos;        //cached old pos
    Vector2d * m_oldVel;        //cached old vec

    //////////////////////////////////////////////////////////////////////
    //rules
    void applyRules(list<CFlockState*>& states);

    //////////////////////////////////////////////////////////////////////
    //Resolve Collision
    void CollisionCheck(list<CFlockState*>& states);
    void CollisionCheck(CFlockState& curS, float step, int id);
    bool isCollision(CFlockState& curS, Point2d& old_pos);
    void reflect
    (CFlockState& curS,float collidTime,float remainTime,int id);
    //used for collision detection
    float m_dCP;           //collision position
    Vector2d n_CDNormal;    //normal of collision plane

    //////////////////////////////////////////////////////////////////////
    void alloc( int size );
    void dealloc();
    void copy(list<CFlockState*>& states);
    void updateOrientation(list<CFlockState*>& states);
};

#endif // !defined(AFX_PARTICLESOLVER_H__FA0FDB91_2266_4485_A5FD_F94EBE754E55__INCLUDED_)

