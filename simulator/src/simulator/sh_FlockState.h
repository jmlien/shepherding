#ifndef _SH_FLOCK_STATE_H_
#define _SH_FLOCK_STATE_H_

//////////////////////////////////////////////////////////////////////////
#include <Point.h>
#include <Basic.h>
//////////////////////////////////////////////////////////////////////////
#include "sh_Robot2D.h"
#include "RNG.h"

class CEnvironment;
class CForceRule;
class CBehaviorRule;
//////////////////////////////////////////////////////////////////////////
#include <list>
#include <iostream>
using namespace std;

//////////////////////////////////////////////////////////////////////////
//State for each instance of robot
class CFlock;
class CFlockState{
public:

    CFlockState(CFlock * type);
    virtual ~CFlockState();

    //////////////////////////////////////////////////////////////////////////
    // Position
    virtual void setPos(const Point2d& pos){ m_Position=pos; }
    virtual const Point2d& getPos() const { return m_Position; }
    //////////////////////////////////////////////////////////////////////////
    // Velocity
    virtual void setVelocity(const Vector2d& v)  { m_Velocity=v;      }
    virtual const Vector2d& getVelocity() const  { return m_Velocity; }
    //////////////////////////////////////////////////////////////////////////
    // Rotation
    virtual void  setRot(float y){ m_Y=(y/180)*PI; rotateY(m_R,m_Y); }
    virtual float getRot() const { return 180*m_Y/PI; }
    virtual float * getRotM() const { return (float*)m_R; }
    virtual void updateRot_using_V(){
        if( m_Velocity.normsqr()<1e-10 ) return;
        m_Y=atan2(m_Velocity[0],m_Velocity[1]);
        rotateY(m_R,m_Y);
    }
    virtual void updateRot_using_oldP(const Point2d& oldP){
        Vector2d v=m_Position-oldP;
        if( v.normsqr()<1e-10 ) return;
        m_Y=atan2(v[0],v[1]);
        rotateY(m_R,m_Y);
    }
    virtual Vector2d getFacingDir(){ return Vector2d(((float)sin(m_Y)),(float)(cos(m_Y))); }
    //////////////////////////////////////////////////////////////////////////
    // Type, ID
    virtual CFlock * getType() const { return m_Type; }
    virtual int getID() const { return m_ID; }
    static int getTotalSize(){ return TOTAL_STATE_SIZE; }
    //////////////////////////////////////////////////////////////////////////
    // color
    virtual void setColor( float r, float g, float b ){
        m_Color[0]=r; m_Color[1]=g; m_Color[2]=b;
    }
    virtual const Point3d& getColor() const { return m_Color; }
    //////////////////////////////////////////////////////////////////////////
    // Visible Agents
    virtual void setVisibleAgents(list<CFlockState*>& v){
        m_VisibleAgents.clear();
        m_VisibleAgents.swap(v);
    }
    virtual list<CFlockState*>& getVisibleAgent(){ return m_VisibleAgents; }
    //////////////////////////////////////////////////////////////////////////
    // Visible Obst
    virtual void setVisibleObst(){m_SeeObstacle=false;} //not seeing any obst
    virtual void setVisibleObst(const Point2d& pos, const Vector2d& n)
    {m_SeeObstacle=true; m_ObstPos=pos; m_ObstN=n;}
    virtual bool seeObstalce() const { return m_SeeObstacle; }
    virtual const Point2d& getVisibleObstPt() const { return m_ObstPos; }
    virtual const Vector2d& getVisibleObstN() const { return m_ObstN; }
    ///////////////////////////////////////////////////////////////////////////
    virtual CBehaviorRule * getBehaviorRule();
    virtual void setBehaviorRule(CBehaviorRule * r){m_BR=r;}
    ///////////////////////////////////////////////////////////////////////////
    virtual CForceRule * getForceRule();
    virtual void setForceRule(CForceRule * r){m_FR=r;}
    ///////////////////////////////////////////////////////////////////////////
    virtual bool isFriend(CFlockState* s){ return false; }

    // Enable / disable displaying the model of this flock member in the GUI
    virtual void enableDraw()    { m_drawEnabled = true;  }
    virtual void disableDraw()   { m_drawEnabled = false; }
    virtual bool isDrawEnabled() { return m_drawEnabled;  }

///////////////////////////////////////////////////////////////////////////////
protected:

    //id
    static int TOTAL_STATE_SIZE;
    int m_ID;

    //Data
    Point2d m_Position;    //current pos
    Vector2d m_Velocity;   //current velocity
    float m_R[3][3];      //current orientation
    float m_Y;
    Point3d m_Color;       //color of this member

    //visibile angents/obst
    list<CFlockState*> m_VisibleAgents;
    bool m_SeeObstacle;
    Point2d m_ObstPos;   //the visible point on the obstacle
    Vector2d m_ObstN;    //the normal at m_ObstPos

    //type
    CFlock * m_Type;     //container of this

    //behavior rule
    CBehaviorRule * m_BR;

    //forceRule
    CForceRule * m_FR;

    //should flock member be displayed in GUI?
    bool    m_drawEnabled;

};

///////////////////////////////////////////////////////////////////////////////
//Info for each type of robot
class CFlock
{

public:

    //Constructor
    CFlock(CEnvironment * env, const string& name, int size=50);
    CFlock(CEnvironment * env, IModel * model, int size=50);
    virtual ~CFlock();

    ///////////////////////////////////////////////////////////////////////////
    //deploy flock members using gaussian dist
    virtual void deployFlock
    (RNG * rng,const Point2d& pos,float dev,float vm=1,Vector2d dir=Vector2d(0,0));

    ///////////////////////////////////////////////////////////////////////////
    virtual void configure(const CFlockState& s){
        const Point2d & pos=s.getPos();
        m_Robot.setPos(pos[0],0,pos[1]);
        m_Robot.setRot(s.getRotM());
    }
	
	///////////////////////////////////////////////////////////////////////////
	virtual void createTrueProject(); //istead of using circle projection (default)
	                                  //call this function to recreate m_Robot 
	                                  //usinf the true projection of m_Model
	
    ///////////////////////////////////////////////////////////////////////////
    //Access
    virtual CFlockState& addState();
    virtual void addState(CFlockState * s);
    virtual CFlockState& getState(int i){ return *m_State[i]; }
    virtual int getStateSize() const { return m_State.size(); }
    virtual vector<CFlockState*>& getStates(){ return m_State; }
    virtual int getID() const {    return m_ID;  }
    ///////////////////////////////////////////////////////////////////////////
    virtual CRobot2D& getGeometry() { return m_Robot; }
    virtual IModel * getRawModel() { return m_Model; }
    ///////////////////////////////////////////////////////////////////////////
    virtual void setMass( float m ) { if(m>0) m_Mass=m; }
    virtual float getMass() const { return m_Mass; }
    ///////////////////////////////////////////////////////////////////////////
    virtual void setColor( float r, float g, float b ){
        m_Color[0]=r; m_Color[1]=g; m_Color[2]=b;
        for(unsigned int i=0;i<m_State.size();i++) m_State[i]->setColor(r,g,b);
    }

    ///////////////////////////////////////////////////////////////////////////
    // model functions
    virtual void setTexture( const string& name ){ m_Texture=name; }
    virtual const string& getTexture() const { return m_Texture; }
    ///////////////////////////////////////////////////////////////////////////
    virtual void  setViewAngle(float v) { m_ViewAngle=(float)((v/180)*PI); }
    virtual float getViewAngle() const { return m_ViewAngle; }
    virtual void  setViewRadius(float r) { m_ViewRadius=r; }
    virtual float getViewRadius() const { return m_ViewRadius; }
    virtual void setCDforView(bool flag){ m_CheckCDforView=flag; }
    virtual bool getCDforView() const { return m_CheckCDforView; }
    ///////////////////////////////////////////////////////////////////////////
    virtual CForceRule * getForceRule(){ return m_ForceRule; }
    virtual void setForceRule(CForceRule * rule){ m_ForceRule=rule; }
    ///////////////////////////////////////////////////////////////////////////
    virtual CBehaviorRule * getBehaviorRule(){ return m_BehaviorRule; }
    virtual void setBehaviorRule(CBehaviorRule * rule){ m_BehaviorRule=rule; }

///////////////////////////////////////////////////////////////////////////////
protected:

    virtual void initialize(CEnvironment * env, IModel * model, int size);
	
private:
	
	//point to the environment that this flock is in
	CEnvironment * m_pEnv;
	
    //id
    static int TOTAL_FLOCK_SIZE;
    int m_ID;

    //Data
    CRobot2D m_Robot;         //the geo
    IModel * m_Model;         //the model loader, contains raw geometric model information
    //string m_ModelName;  //the filename of the model

    //Property
    float m_Mass;
    string m_Texture;

    //view
    float m_ViewAngle;
    float m_ViewRadius;

    // when this is ture, a collision check between the observer and
    // the viewed object will be performed during the visiblility test.
    // this will block the agent to see each other if there is a wall
    // in between.
    // This is set to FALSE by default.
    bool  m_CheckCDforView;

    //instances
    vector<CFlockState*> m_State;    //< Cfgs of robot instances

    //force rule
    CForceRule * m_ForceRule;

    //behavior rule
    CBehaviorRule * m_BehaviorRule;

    //other properties
    Point3d m_Color;       //color
};

#endif


