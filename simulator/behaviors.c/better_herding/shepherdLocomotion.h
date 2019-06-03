#ifndef SHEPHERD_LOCOMOTION_H
#define SHEPHERD_LOCOMOTION_H

#include "shepherding_base.h"
#include "simple_herding.h"
#include "shepherding.h"
#include "sh_dyPRM.h"

inline Vector2d rotate(const Vector2d& V, float sin_theta)
{
    float cos_theta=sqrt(1-(sin_theta*sin_theta));  
    return Vector2d(V[0]*cos_theta+V[1]*sin_theta,-V[0]*sin_theta+V[1]*cos_theta);
}


//number of flock member that can see shepherd
inline int viewShepherdSize(CHerdingFlockState& s)
{
    FlockGroup& flock=s.flock_group;
    int count=0;
    for(FSLIST::iterator i=flock.states.begin();i!=flock.states.end();i++){
        float vr=(*i)->getType()->getViewRadius();
        if( ((*i)->getPos()-s.getPos()).normsqr()<vr*vr )
            count++;
    }
    return count;
}

struct locomotionInfo
{
    locomotionInfo(CHerdingFlockState& _s):s(_s)
    {
        pair<float, Point2d> tmp=findEC(s.flock_group.states);
        R_f=tmp.first;
        C_f=tmp.second;
        Vc=(s.getPos()-C_f);
        Vt=(s.getPos()-s.target);
        D_Vc=Vc.norm();
        D_Vt=Vt.norm();
    }

    CHerdingFlockState& s; 
    float R_f;
    Point2d C_f;
    Vector2d Vc;
    Vector2d Vt; 
    float D_Vc;
    float D_Vt;
};

//------------------------------------------------------------------------------
//
//  Approaching:
//      approachingLine: Use a straight line
//  approachingSZ: Use a circular safe zone
//  approachingDynMap: Use a dynamic roadmap
//
//------------------------------------------------------------------------------

//straight line approoching
struct approachingLine
{
    virtual void approachingPt(locomotionInfo& lm){ return; } //did nothing
    
    virtual Vector2d approachingDir(locomotionInfo& lm)
    {
        if(lm.D_Vt<0.5) return Vector2d(0,0);
        return lm.Vt/lm.D_Vt;
    }

    virtual bool isApproaching(locomotionInfo& lm)
    {
        lm.s.isApproaching=0;
        if(lm.D_Vc<lm.D_Vt) lm.s.isApproaching=1; //target offset
        return lm.s.isApproaching;
    }
};


//safe zone approoching
struct approachingSZ: public approachingLine
{
    virtual Vector2d approachingDir(locomotionInfo& lm)
    {
        if( lm.D_Vt<lm.D_Vc ){ //closer to the target than to the flock center
            if( lm.D_Vt<0.5 ) return Vector2d(0,0);
            return lm.Vt/lm.D_Vt; //straight to the target
        }
        else if( lm.D_Vc<lm.R_f ){ // inside the flock radius 
            return (lm.Vc/(-lm.D_Vc)).normalize();  
        }
        else{ //normal situation
            float sin_theta=lm.R_f/lm.D_Vc;
            Vector2d D1=rotate(lm.Vc, sin_theta).normalize();
            Vector2d D2=rotate(lm.Vc,-sin_theta).normalize();
            shCD cd(getEnvironment());
            Point2d P1=lm.s.getPos()+D1;
            Point2d P2=lm.s.getPos()+D2;

            if(cd.isCollision((const CFlockState&)lm.s,P1))
                return D2;
            else if( cd.isCollision((const CFlockState&)lm.s,P2) )
                return D1;

            //no collision
            if(D1*lm.Vt>D2*lm.Vt)
                return D1; //find which on is closer to dir
            else
                return D2;
        }
    } 
};

//dynamic map approoching
struct approachingDynMap: public approachingLine
{
    CRoadMap * dyn_map; // dynamic roadmap, init to be empty
    CDynPRM *  dyn_prm;
    list<Point2d> path;

    approachingDynMap()
    {
        dyn_map=NULL;
        dyn_prm=NULL;
        //-------------- DRAW --------------//
        //dm=DrawMap();
        //d_dypath=DrawPath();
        //d_next=DrawGoal();
        //-------------- DRAW --------------//
    }
    
    virtual void approachingPt(locomotionInfo& lm)
    {
        if( dyn_map==NULL || dyn_prm==NULL) 
            createMap(lm.s);
        lm.s.target=findDynPath(lm.s);
        //-------------- DRAW --------------//
        //d_next.goal=s.target;
        //d_next.message="next";
        //-------------- DRAW --------------//
    }

    virtual bool isApproaching(locomotionInfo& lm)
    {
        bool app=approachingLine::isApproaching(lm);
        if( dyn_prm!=0 )
            dyn_prm->updateMap(*dyn_map,&lm.s,lm.s.flock_group.states);
        if(!app) path.clear();
        return app;
    }
    
private:

    void createMap(CHerdingFlockState& s)
    {
        dyn_map = new CRoadMap(s.getType(),200);
        dyn_prm = new CDynPRM(getEnvironment(),getRNG());
        assert(dyn_map);
        assert(dyn_prm);
        //-------------- DRAW --------------//
        //dm.rmap=dyn_map;
        //dm.color=Point3d(0.8,0.5,0.5);
        //addDrawObj(dm);
        //addDrawObj(d_dypath);
        //addDrawObj(d_next);
        //-------------- DRAW --------------//
    }

    Point2d findDynPath(CHerdingFlockState& s)
    {
        if((s.target-s.getPos()).normsqr()<4) 
            return s.target;

        //check if the target is moving too far away..
        if(!path.empty()){
            if( (s.target-path.back()).normsqr()>15 )
                path.clear();
        }

        if( path.empty() ){
            dyn_prm->queryPath(s.getPos(),s.target,*dyn_map,*s.getMap(),&s,s.flock_group.states,path);
            //-------------- DRAW --------------//
            //d_dypath.color=Point3d(0.5,0.7,0.2);
            //d_dypath.path=path;
            //-------------- DRAW --------------//
        }

        while( !path.empty() ){
            if( (path.front()-s.getPos()).normsqr()>9 )
                return path.front(); // return the first element
            else path.pop_front();
        }

        return s.target; //end 
    }
};

//------------------------------------------------------------------------------
//
//  Turning:
//
//------------------------------------------------------------------------------

//TODO the display objects are always visible--should be able to turn them off with the 3 key
//TODO display objects are never removed from the 'addDrawObj()' list

//Interface class
class turningNo
{
   public:
   
   virtual void turningPt(locomotionInfo& lm)
   {   
       turningPt(lm.s, lm.R_f, lm.C_f, lm.Vc, lm.Vt, lm.D_Vc, lm.D_Vt);
   }
   
   virtual void turningDir(locomotionInfo& lm){
       turningDir(lm.s, lm.R_f, lm.C_f, lm.Vc, lm.Vt, lm.D_Vc, lm.D_Vt);
   }
   virtual bool isTurning(locomotionInfo& lm){
       return isTurning(lm.s, lm.R_f, lm.C_f, lm.Vc, lm.Vt, lm.D_Vc, lm.D_Vt);
   }
   
   virtual void turningPt(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)=0;
   virtual void turningDir(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)=0;
   virtual bool isTurning(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt){return false;}
};


//NOTE -- the following two classes add draw sh_Draw type objects to a global(?) drawing list with the addDrawObj() function,
// however, there is no code in place to remove them once the objects are destructed.  Im not sure if it is done automatically 
// or if it just doesnt matter, but the python classes dont do any removal.  This might need to be fixed if the draw code is
// causing crashes from referencing dead pointers.


//Utility class -- (I think) it determines a way point for the flock to go to for turning?
class turningStop : public turningNo
{
public:

   turningStop(){}
   virtual ~turningStop(){}
   
   virtual void turningPt
   (CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
   {
       // temporary? constant value
       float dist = 2.5f;
    
       float D_Vg = Vg.norm();
    
       // if at the goal, exit
       if(D_Vg < 0.000001f) return;
    
       Vg = Vg / D_Vg;
       Point2d turning_point = C_f + Vg*(R_f + dist);
    
    
       //=============== Drawing Code =======================//
       //d_tp.goal = turning_point;
       //d_tp.message = "turn here";
       //====================================================//
    
       s.target = turning_point;
   }
   
   virtual void turningDir
   (CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
   {
       //do nothing
   }
   
   virtual bool isTurning
   (CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
   {
       static bool dothisonce=false;
       if(!dothisonce){
            //createDrawObj();
            dothisonce=true;
       }
    
       if(s.flock_group.shepherd_number == 0 || (int)(s.flock_group.radius) == 0)
       {
          s.isTurning = false;
          return false;
       }
    
       Vg = groupVec(s.flock_group);
    
       //if the radius is greater than 6, the turning operation is too slow to be useful
       if(R_f > 6)
       {
          s.isTurning = false;
          return false;
       }
    
       float D_Vg = Vg.norm();
       Vg = Vg/D_Vg;
       Vector2d Vtc = (C_f - s.target).normalize();
    
       //I cant find this function anywhere, I assume the cmath version was overloaded to work with vectors
       float theta = acos(Vg*Vtc);
       /* If that doesnt work, use...
       float theta = acos(Vtc*Vg.y / Vtc*Vg.x); */
    
       s.isTurning = theta > M_PI / 4.0f;
       return s.isTurning;
   }

protected:

   Vector2d groupVec(FlockGroup& g) const
   {
        Vector2d totalV(0,0);
        for(FSLIST::iterator i = g.states.begin();i!=g.states.end();i++){
            totalV=totalV+(*i)->getVelocity();
        }
        return totalV.normalize();
    }

   //DrawDir d_Vg;
   //DrawGoal d_tp;
   Vector2d Vg;
   
};

//Utility class -- unfortunately, Im not really sure what this one is doing, but it should work similarly to the python version
class turningPrevent : public turningStop
{
public:

   turningPrevent(){}
   virtual ~turningPrevent(){}

   virtual void turningPt
   (CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
   {
        Point2d  M1 = s.milestones[0];
        Point2d  M2 = s.milestones[1];
        Vector2d V1 = (M2 -  M1).normalize();
        Vector2d V2 = (M1 - C_f).normalize();
        Vector2d V3 = (V2 + V2 - V1).normalize();
        
        s.target = C_f + V3*(R_f + 1.5);
        /* alternative version was commented out in python code:
        s.target = C_f + V3*(R_f + 2.5);  */
        
        //d_tp.goal = s.target;
        //d_tp.message = "turn here";
   }
   
   virtual void turningDir
   (CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
   {
       //do nothing
   }
   
   virtual bool isTurning
   (CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
   {
       /* Not sure this is correct?, original python code is as follows...
       if self.d_tp!=0: self.d_tp.goal=Point2d(-1000,0);  */
    
       s.isTurning = false;
       Point2d  M1 = s.milestones[0];
       Point2d  M2 = s.milestones[1];
    
       if((s.getPos() - M1).norm() > (R_f + 25.0f))
       {
          return false;  //the flock is not close enough to the milestone
       }
       if(groupVec(s.flock_group).norm() < 1)
       {
          return false;  //the flock is already under control
       }
    
       Vector2d V1 = M2 - M1;
       float D_V1 = V1.norm();
    
       //this case probably should not be occuring? (it would require the milestones to be on top of each other?)
       if(fabs(D_V1) < 0.00001f)
       {
          return false;
       }
    
       V1 = V1/D_V1;
       Vector2d V2=(M1 - C_f).normalize();
    
       float dot_product = V1*V2;
       /* Not sure * operator is overloaded in C++ version, else use...
       float dot_product = V1.x*V2.x + V1.y*V2.y;   */
    
       if(dot_product > 1) dot_product = 1.0f;
       float theta = acos(dot_product);
    
       s.isTurning = theta > (M_PI/4);
       return s.isTurning;
   }
};

//------------------------------------------------------------------------------
//
//  Steering:
//      steeringLine: use a straight line to steer
//  steeringSwing: move side-to-side behide the flock
//
//------------------------------------------------------------------------------

//straight line steering
struct steeringLine
{
    virtual Point2d steeringPt(locomotionInfo& lm){ return Point2d(); } //did nothing
    
    virtual Vector2d steeringDir(locomotionInfo& lm){
        if( lm.D_Vt<0.5 ) return Vector2d(0,0);
        return lm.Vt/lm.D_Vt;
    }

    virtual bool isSteering(locomotionInfo& lm){
        return true; //yes
    }
};

//swing steering
struct steeringSwing : public steeringLine
{
    steeringSwing(){}

    virtual Point2d steeringPt(locomotionInfo& lm)
    {

        //cout<<"swing_step="<<lm.s.swing_step<<endl;

        //compute swing point
        Vector2d Vct=(lm.s.target-lm.C_f);
        Point2d X=swingPt(lm.s.swing_step,lm.s.target,Vct,lm.C_f);

        //cout<<"lm.s="<<lm.s.getPos()<<" X="<<X<<endl;

        if( isClose(lm.s,X) ){
            lm.s.swing_step=(lm.s.swing_step+1)%8;
            X=swingPt(lm.s.swing_step,lm.s.target,Vct,lm.C_f);

        //  cout<<"2 swing_step="<<lm.s.swing_step<<endl;
        }
        //-------------- DRAW --------------//
        //.dswingpt.goal=X;
        //.dswingpt.message="swing pt";
        //-------------- DRAW --------------//
        lm.s.target=X;
        return X;
    }
        
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Point2d swingPt
    (int Swing_step, const Point2d& target, const Vector2d& Vc, const Point2d& O)
    {
        if( Swing_step==0 || Swing_step==4 ) return target;
        if( Swing_step==1 || Swing_step==3 || Swing_step==5 || Swing_step==7 ) //ie. odd # 
            return swingB(Vc,O);
        if( Swing_step==2 ) return swingC(Vc,O);
        if( Swing_step==6 ) return swingD(Vc,O);

        return target;
    }

    Point2d swingB(const Vector2d& Vc, const Point2d& O){ return O+Vc; }//+Vc.normalize()*2; }
    Point2d swingC(const Vector2d& Vc, const Point2d& O){ return swingSide(Vc,O,1); } // to right
    Point2d swingD(const Vector2d& Vc, const Point2d& O){ return swingSide(Vc,O,-1); }// to lef

    Point2d swingSide(const Vector2d& Vc, const Point2d& O,int sign)
    {
        float sin_60=sin(PI/3);
        Vector2d newV;

        if( sign==1 )
            newV=rotate(Vc,sin_60);
        else
            newV=rotate(Vc,-sin_60);

        return swingB(newV,O);
    }

    bool isClose(CHerdingFlockState& s, const Point2d& pt)
    {
        if( (s.getPos()-pt).normsqr()<1 ) return true;
        return false;
    }
};

#endif //SHEPHERD_LOCOMOTION_H
