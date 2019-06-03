

#ifndef SHEPHERDLOCOMOTOIN_EXT_H
#define SHEPHERDLOCOMOTION_EXT_H


#include "shepherdLocomotion.h"
#include "../simple_herding/simple_herding_draw_ext.h"   //needed for DrawDir definition


//TODO the display objects are always visible--should be able to turn them off with the 3 key
//TODO display objects are never removed from the 'addDrawObj()' list

//Interface class
class turningNo
{
   public:
   turningNo();
   virtual ~turningNo();
   
   virtual void turningPt(locomotionInfo&) = 0;
   virtual void turningPt(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt) =  0;
   virtual void turningDir(locomotionInfo&) = 0;
   virtual void turningDir(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt) = 0;
   virtual bool isTurning(locomotionInfo&) = 0;
   virtual bool isTurning(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt) = 0;
};
inline turningNo::turningNo() { }
inline turningNo::~turningNo() { }
inline void turningNo::turningPt(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt) { }
inline void turningNo::turningDir(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt) { }
inline bool turningNo::isTurning(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt) { return false; }


//NOTE -- the following two classes add draw sh_Draw type objects to a global(?) drawing list with the addDrawObj() function,
// however, there is no code in place to remove them once the objects are destructed.  Im not sure if it is done automatically 
// or if it just doesnt matter, but the python classes dont do any removal.  This might need to be fixed if the draw code is
// causing crashes from referencing dead pointers.


//Utility class -- (I think) it determines a way point for the flock to go to for turning?
class turningStop : public turningNo
{
   protected:
   //DrawDir d_Vg;
   //DrawGoal d_tp;
   Vector2d Vg;

   public:
   turningStop();
   virtual ~turningStop();

   virtual void turningPt(locomotionInfo&);
   virtual void turningPt(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt);
   virtual void turningDir(locomotionInfo&);
   virtual void turningDir(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt);
   virtual bool isTurning(locomotionInfo&);
   virtual bool isTurning(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt);

   Vector2d groupVec(FlockGroup& fg) const;
   void createDrawObj();
};

//=======================================//
//     Method Definitions
//=======================================//
inline turningStop::turningStop()
{
   Vg = Vector2d(1.0f, 0.0f);
}
inline turningStop::~turningStop()
{
   //do nothing
}

inline void turningStop::turningPt(locomotionInfo& lm)
{
   turningStop::turningPt(lm.s, lm.R_f, lm.C_f, lm.Vc, lm.Vt, lm.D_Vc, lm.D_Vt);
}

inline void turningStop::turningPt(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
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

inline void turningStop::turningDir(locomotionInfo& lm)
{
   turningStop::turningDir(lm.s, lm.R_f, lm.C_f, lm.Vc, lm.Vt, lm.D_Vc, lm.D_Vt);
}

inline void turningStop::turningDir(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
{
   //do nothing
}

inline bool turningStop::isTurning(locomotionInfo& lm)
{
   return turningStop::isTurning(lm.s, lm.R_f, lm.C_f, lm.Vc, lm.Vt, lm.D_Vc, lm.D_Vt);
}

inline bool turningStop::isTurning(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
{
   static bool dothisonce=false;
   if(!dothisonce){
		createDrawObj();
		dothisonce=true;
   }

   if(s.flock_group.shepherd_number == 0 || (int)(s.flock_group.radius) == 0)
   {
      s.isTurning = false;
      return false;
   }

   Vg = groupVec(s.flock_group);
   //============ Draw =================//

   //note -- moved createDrawObj call to constructs so that they are created by default

   //this value was copied from the python code -- I assume its meant to be temporary debugging value
   //d_tp.goal = Point2d(-1000,0);
   //d_Vg.dir = Vg;
   //d_Vg.pos = C_f;
   //===================================//

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

//This was function was defined in another class which I copied it from, but I added it to this one as
// well because it was in the python class defintion
inline Vector2d turningStop::groupVec(FlockGroup& g) const
{
	Vector2d totalV(0,0);
	for(FSLIST::iterator i = g.states.begin();i!=g.states.end();i++){
		totalV=totalV+(*i)->getVelocity();
	}
	return totalV.normalize();
}

inline void turningStop::createDrawObj()
{
   //d_Vg = DrawDir();
   //addDrawObj(&d_Vg);
   //d_tp = DrawGoal();
   //addDrawObj(&d_tp);
}



//Utility class -- unfortunately, Im not really sure what this one is doing, but it should work similarly to the python version
class turningPrevent : public turningStop
{
   public:
   turningPrevent();

   virtual void turningPt(locomotionInfo&);
   virtual void turningPt(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt);
   virtual void turningDir(locomotionInfo&);
   virtual void turningDir(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt);
   virtual bool isTurning(locomotionInfo&);
   virtual bool isTurning(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt);
};


//=======================================//
//     Method Definitions
//=======================================//

//not sure this is correct -- turningStop constructor adds a DrawGoal and a DrawDir to the drawing list, but it looks like
//turningPrevent only needs the DrawGoal, even though it inherits both
inline turningPrevent::turningPrevent() :
   turningStop()
{
   //just calls parent constructor
}

inline void turningPrevent::turningPt(locomotionInfo& lm)
{
   turningPrevent::turningPt(lm.s, lm.R_f, lm.C_f, lm.Vc, lm.Vt, lm.D_Vc, lm.D_Vt);
}

inline void turningPrevent::turningPt(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
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

inline void turningPrevent::turningDir(locomotionInfo& lm)
{
   turningPrevent::turningDir(lm.s, lm.R_f, lm.C_f, lm.Vc, lm.Vt, lm.D_Vc, lm.D_Vt);
}

inline void turningPrevent::turningDir(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
{
   //do nothing
}

inline bool turningPrevent::isTurning(locomotionInfo& lm)
{
   return turningPrevent::isTurning(lm.s, lm.R_f, lm.C_f, lm.Vc, lm.Vt, lm.D_Vc, lm.D_Vt);
}

inline bool turningPrevent::isTurning(CHerdingFlockState& s, float R_f, Point2d C_f, Vector2d Vc, Vector2d Vt, float D_Vc, float D_Vt)
{
   //d_tp.goal = Point2d(-1000, 0);

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


#endif


