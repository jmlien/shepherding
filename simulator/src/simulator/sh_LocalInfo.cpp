
//
// this needs to be rewritten...
//

#include <sh_LocalInfo.h>
#include <sh_CollisionDetection.h>
#include <sh_FlockState.h>

char * g_collide=NULL;
inline void alloc()
{
    static int totalsize=0;
    static int totalsize_2=0;
    if( totalsize!=CFlockState::getTotalSize()){
        totalsize=CFlockState::getTotalSize();
        totalsize_2=totalsize*totalsize;
        delete [] g_collide;
        g_collide=new char[totalsize_2];
    }
    memset(g_collide,'2',totalsize_2*sizeof(char));
}

inline bool checkCollision
(CEnvironment& env,CFlockState* fi,CFlockState* fj)
{
	shCD cd(&env);
	
    int id=fi->getID()*CFlockState::getTotalSize()+fj->getID();
    if( g_collide[id]=='2' )
        g_collide[id]=(cd.isCollision(*fi,*fj))?1:0;
    if( g_collide[id]==1 ) return false;; //collision
    return true;
}

inline void 
getNeighboringAgent(list<CFlockState*>& flock,CEnvironment& env)
{
    typedef list<CFlockState*>::iterator FIT;
    alloc();
    ///////////////////////////////////////////////////////////////////////////
    for( FIT i=flock.begin();i!=flock.end();i++ ){
        CFlockState * fi=*i;
        if( fi->getType()->getForceRule()==NULL ) continue;
        list<CFlockState*> visible;
        float radius=fi->getType()->getViewRadius();
        float angle=fi->getType()->getViewAngle();
        float radius_2=radius*radius;
        float angle_2=angle/2;
        Vector2d xdir=fi->getFacingDir();
        ///////////////////////////////////////////////////////////////////////
        for( FIT j=flock.begin();j!=flock.end();j++ ){
            CFlockState * fj=*j;
            if( fi==fj ) continue; //same
            Vector2d dir=fj->getPos()-fi->getPos();
            float dist_sqr=dir.normsqr();
            if( dist_sqr>radius_2 ) continue; //too far
            if( acos((dir/sqrt(dist_sqr))*xdir)>angle_2 ) continue; //not in the range
            if(fi->getType()->getCDforView()) //need to check CD
                if( !checkCollision(env,fi,fj) ) continue;
            visible.push_back(fj);
            //cout<<fi->getID()<<" sees "<<fj->getID()<<endl;
        }//end j
        
        fi->setVisibleAgents(visible);
    }//end i
    ///////////////////////////////////////////////////////////////////////////
    //cout<<"---------------------------------------"<<endl;
}

void getPotentialObstColl(list<CFlockState*>& flock, CEnvironment& env)
{
    typedef list<CFlockState*>::iterator FIT;
    shCD cd(&env);
    ///////////////////////////////////////////////////////////////////////////
    for( FIT i=flock.begin();i!=flock.end();i++ ){
        CFlockState * fi=*i;
        const Point2d& p=fi->getPos();
        Vector2d v=fi->getFacingDir();
		CRobot2D & robot=fi->getType()->getGeometry();
        //the farest point can see
        Point2d farp=p+v*(fi->getType()->getViewRadius()-robot.getRadius());
        Vector2d n; float t;
        //if there is an expected collision
		if( cd.getCollisionInfo(robot,p,farp,n,t) ){
            Point2d cdp((1-t)*p[0]+t*farp[0],(1-t)*p[1]+t*farp[1]);
            fi->setVisibleObst(cdp,n);
        }
        else{
            fi->setVisibleObst(); //disable
        }
    }//end i
}

void getLocalInfo(list<CFlockState*>& flock,CEnvironment& env)
{
    getNeighboringAgent(flock,env);
    getPotentialObstColl(flock,env);
}

