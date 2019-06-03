#include "shepherding_base.h"

///////////////////////////////////////////////////////////////////////////
CBasicForceRule::CBasicForceRule()
{
    m_Cohesion=5;
    m_Separation=5;
    m_Alignment=1;
    m_ObstRepulsion=1;
    m_Max_Force=5;
    m_Dampling=0.3f;
}

CBasicForceRule::CBasicForceRule(const CBasicForceRule& other)
:CForceRule()
{
    m_Cohesion=other.m_Cohesion;
    m_Separation=other.m_Separation;
    m_Alignment=other.m_Alignment;
    m_ObstRepulsion=other.m_ObstRepulsion;
    m_Max_Force=other.m_Max_Force;
    m_Dampling=other.m_Dampling;
}

///////////////////////////////////////////////////////////////////////////
Vector2d CBasicForceRule::getForce(CFlockState& s)
{
    Vector2d force_sep,force_alg,force_coh,force_obs;
    Point2d nei_center;  //the center of neigbors
    const Point2d& p1=s.getPos();
    Vector2d v1=s.getVelocity();

    float vr=s.getType()->getViewRadius();
    float vr_2=vr*vr;
    Vector2d force=((float)-m_Dampling)*v1; //with Damping
	
	//nothing left to do?
	if(m_Separation==0 && m_Alignment==0 && m_Cohesion==0 && m_ObstRepulsion==0)
		return force;

    ///////////////////////////////////////////////////////////////////////
    list<CFlockState*>& neighbors=s.getVisibleAgent();
    int nei_size=0;

	float r1=s.getType()->getGeometry().getRadius();

    typedef list<CFlockState*>::iterator NIT;
	bool in_collision_with_others=false;

    for( NIT in=neighbors.begin();in!=neighbors.end();in++ ){
        if( (*in)->getType()!=s.getType() ) continue; //alien...
        nei_size++;
		float r2=(*in)->getType()->getGeometry().getRadius();
		float r12_sqr=(r1+r2)*(r1+r2);

        const Point2d& p2=(*in)->getPos();
        Vector2d v2=(*in)->getVelocity();

        //compute distance
        Vector2d u12=p2-p1; 
        float distsqr=u12.normsqr();
        if( distsqr==0 ) u12.set(1,0);

		//check if in collision
		if(distsqr<r12_sqr){
			in_collision_with_others=true;
		}
        
        //compute forces
        float weight=1-distsqr/vr_2;
        
        //seperation
        Vector2d sep=(-1*weight*m_Separation)*u12.normalize();
        force_sep=force_sep+sep;

        //alignment
        if( v1.normsqr()*v2.normsqr()>0.01 ){
			Vector2d v2_v1=v2.normalize()-v1.normalize();
			float v2_v1_norm=v2_v1.norm();
			if( v2_v1_norm!=0 ){
				Vector2d alg=(weight*m_Alignment)*v2_v1/v2_v1_norm;
				force_alg=force_alg+alg;
			}
        }

        //Cohersion, find center
        nei_center[0]+=p2[0]; 
        nei_center[1]+=p2[1]; 
    }//end in

	//normalize the forces
    if( nei_size!=0 ){
        force_sep=force_sep/nei_size;
        force_alg=force_alg/nei_size;

        nei_center[0]/=nei_size;
        nei_center[1]/=nei_size;
        Vector2d c_dir=nei_center-p1;
        force_coh=(m_Cohesion*c_dir.normsqr()/vr_2)*c_dir.normalize();
    }

    /*
	bool in_collision_with_obst=isCollision(*getEnvironment(),s);
	if(in_collision_with_obst){
		Point2d freept=s.getPos();
		Push(*getEnvironment(),s,freept);
		float amount=(m_ObstRepulsion<5)?5:m_ObstRepulsion;
		Vector2d v=freept-s.getPos();
		float norm=v.norm();
		if( norm>0 )
			force_obs=v*amount/norm;;
	}
	*/

	//check if in collision
	//if(in_collision_with_others||in_collision_with_obst){
	if(in_collision_with_others){
		//if( in_collision_with_obst ) 
		//	force=force+force_obs;
		//if( in_collision_with_others ){
			float amount=(m_Separation<5)?5:m_Separation;
			force=force+force_sep.normalize()*amount;
		//}
		return force;
	}

    //compute obstacle repulsion
    if( s.seeObstalce() ){
        Vector2d obst_dir=p1-s.getVisibleObstPt();
        const Vector2d obst_n=s.getVisibleObstN();

        float distsqr=obst_dir*obst_n;
        force_obs= m_ObstRepulsion*(1-(distsqr/vr_2))*obst_n;
    }

    //compute final force
    if( truncate(force,force_obs) ) return force;
    if( truncate(force,force_sep) ) return force;
    if( truncate(force,force_alg) ) return force;
    if( truncate(force,force_coh) ) return force;

    ///////////////////////////////////////////////////////////////////
    //float remain=m_Max_Force-force.norm();
	//Vector2d head=s.getFacingDir();
    //force=force+(remain/(m_Dampling*50))*head; //do what you do before
	//force=force+Vector2d(drand48(),drand48())*0.1f;

    return force;
}

//return ture if f1 is full
bool CBasicForceRule::truncate(Vector2d& f1, const Vector2d& f2)
{   
    float f1m=f1.norm();
    float f2m=f2.norm();
    if( f2m==0 ) return false;
    float remain=m_Max_Force-f1m;
    if(remain<0) return true; //full...
    if(remain>f2m){
        f1=f1+f2;
        return false;
    }
    Vector2d remainF=f2*(remain/f2m);
    f1=f1+remainF;
    return true;
}


