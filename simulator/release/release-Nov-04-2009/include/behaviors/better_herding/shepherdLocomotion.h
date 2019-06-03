#ifndef SHEPHERD_LOCOMOTION_H
#define SHEPHERD_LOCOMOTION_H

#include "sh_ForceRules.h"
#include "sh_BehaviorRules.h"
#include "sh_MapBasedFlockState.h"
#include "func/sh_FlockFunc.h"
#include "simple_herding.h"
#include "sh_dyPRM.h"
#include "sh_CollisionDetection.h"

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
//  	approachingLine: Use a straight line
// 	approachingSZ: Use a circular safe zone
//	approachingDynMap: Use a dynamic roadmap
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
			CEnvironment& env=*getEnvironment();
			Point2d P1=lm.s.getPos()+D1;
			Point2d P2=lm.s.getPos()+D2;

			if(isCollision(env,(const CFlockState&)lm.s,P1))
				return D2;
			else if( isCollision(env,(const CFlockState&)lm.s,P2) )
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
		dyn_prm = new CDynPRM(getEnvironment());
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

/*

struct turningNo //dummy class
{
	virtual Point2d turningPt(locomotionInfo& lm){return Point2d();}
	virtual Vector2d turningDir(locomotionInfo& lm){return Vector2d();}
	virtual bool isTurning(locomotionInfo& lm){return false;} //no turning
};

class turningStop(turningNo):
	//-------------- DRAW --------------//
	d_Vg=0;
	d_tp=0;
	//-------------- DRAW --------------//
	Vg=Vector2d(1,0);
	def turningPt(,s,R_f,C_f,Vc,Vt,D_Vc,D_Vt):
		dist=2.5;
		//if viewShepherdSize(s)<1: dist=0;
		D_Vg=.Vg.norm();
		if D_Vg<0.000001: return;
		.Vg=.Vg/D_Vg;
		tp=C_f+.Vg*(R_f+dist);
		//-------------- DRAW --------------//
		.d_tp.goal=tp;
		.d_tp.message="turn here";
		//-------------- DRAW --------------//
		s.target=tp;
		return;

	def turningDir(,s,R_f,C_f,Vc,Vt,D_Vc,D_Vt):
		return;

	def isTurning(,s,R_f,C_f,Vc,Vt,D_Vc,D_Vt):
		if s.flock_group==0:
			s.isTurning=0;
			print "Fuck me:";
			return 0; //no need
		.Vg=.groupVec(s.flock_group);
		//-------------- DRAW --------------//
		if .d_Vg==0: .createDrawObj();
		.d_tp.goal=Point2d(-1000,0);
		.d_Vg.dir=.Vg;
		.d_Vg.pos=C_f;
		//-------------- DRAW --------------//
		if R_f>6: s.isTurning=1; return 1;
		D_Vg=.Vg.norm();
		//if s.isTurning==1 and D_Vg<0.5: 
		if D_Vg<0.5: 
			s.isTurning=0;
			return 0; //too slow
		//if s.isTurning==0 and D_Vg<0.7: return 0; //too slow
		.Vg=.Vg/D_Vg;
		Vtc=(C_f-s.target).normalize();
		theta=acos(.Vg*Vtc);
		s.isTurning=theta>(pi/4);
		return s.isTurning;

	def groupVec(,g):
		totalV=Vector2d(0,0);
		for s in g :
			totalV=totalV+s.getVelocity();
		return totalV/g.size();

	def createDrawObj():
		.d_Vg=DrawDir();
		addDrawObj(.d_Vg);
		.d_tp=DrawGoal();
		addDrawObj(.d_tp);

class turningPrevent(turningNo):
	//-------------- DRAW --------------//
	d_tp=0;
	//-------------- DRAW --------------//
	def turningPt(,s,R_f,C_f,Vc,Vt,D_Vc,D_Vt):
		M1=s.milestones[0];
		M2=s.milestones[1];
		V1=(M2-M1).normalize();
		V2=(M1-C_f).normalize();
		V3=(V2+V2-V1).normalize();
		s.target=C_f+V3*(R_f+1.5); //2.5
		//-------------- DRAW --------------//
		if .d_tp==0:
			.d_tp=DrawGoal();
			addDrawObj(.d_tp);
		.d_tp.goal=s.target;
		.d_tp.message="turn here";
		//-------------- DRAW --------------//
		return;
	def turningDir(,s,R_f,C_f,Vc,Vt,D_Vc,D_Vt):
		return;
	def isTurning(,s,R_f,C_f,Vc,Vt,D_Vc,D_Vt):
		if .d_tp!=0: .d_tp.goal=Point2d(-1000,0);
		s.isTurning=0;
		M1=s.milestones[0];
		M2=s.milestones[1];
		if (s.getPos()-M1).norm()>(R_f+25) : return 0; //not close enough...
		if .groupVecMag(s.flock_group)<1 : return 0; // flock is under control
		V1=M2-M1;
		D_V1=V1.norm();
		if D_V1==0:
			return 0; //M1==M2...
		V1=V1/D_V1;
		V2=(M1-C_f).normalize();
		dot=V1*V2;
		if dot>1: dot=1;
		theta=acos(dot);
		s.isTurning=theta>(pi/4);
		return s.isTurning;

	def groupVecMag(,g):
		totalV=Vector2d(0,0);
		for s in g :
			totalV=totalV+s.getVelocity();
		return totalV.norm()/g.size();
*/

//------------------------------------------------------------------------------
//
//  Steering:
//  	steeringLine: use a straight line to steer
//	steeringSwing: move side-to-side behide the flock
//
//------------------------------------------------------------------------------

//straight line steering
struct steeringLine
{
	virtual Point2d steeringPt(locomotionInfo& lm){	return Point2d(); } //did nothing
	
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

		//	cout<<"2 swing_step="<<lm.s.swing_step<<endl;
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
