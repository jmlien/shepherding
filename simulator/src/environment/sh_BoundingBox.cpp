//////////////////////////////////////////////////////////////////////
//Include Shepherding headers
#include "shepherding_base.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CBoundingBox::CBoundingBox()
{
    memset(m_BBX,0,sizeof(float)*6);
    memset(m_CBBX,0,sizeof(float)*6);
}

CBoundingBox::~CBoundingBox()
{
    memset(m_BBX,0,sizeof(float)*6);
}

//////////////////////////////////////////////////////////////////////
//Core
//////////////////////////////////////////////////////////////////////
bool 
CBoundingBox::createCSPace(float Radius)
{
    
    m_CBBX[0]=m_BBX[0]+Radius; m_CBBX[1]=m_BBX[1]-Radius; 
    m_CBBX[2]=m_BBX[2]+Radius; m_CBBX[3]=m_BBX[3]-Radius; 
    m_CBBX[4]=m_BBX[4]+Radius; m_CBBX[5]=m_BBX[5]-Radius; 

    return true;
}

bool CBoundingBox::getCollisionInfo
( const Point2d & oldPos, const Point2d & newPos,
  Vector2d & Normal/*out*/, float & dCP/*out, collision point*/ )
{
    if( isInBBX(newPos) ) return false; //no collision found
    //Test which plane it might collide
    dCP=1.0; //to to max
    if( newPos[0]<m_CBBX[0] ) getCollisionInfo_Left(oldPos, newPos, Normal, dCP);
    else if( newPos[0]>m_CBBX[1] ) getCollisionInfo_Right(oldPos, newPos, Normal, dCP);
    if( newPos[1]<m_CBBX[4] ) getCollisionInfo_Back(oldPos, newPos, Normal, dCP);
    else if( newPos[1]>m_CBBX[5] ) getCollisionInfo_Front(oldPos, newPos, Normal, dCP);
    Normal=Normal.normalize();
    return true;
}

void CBoundingBox::Push(Point2d & Pos)
{
    if( Pos[0]<m_CBBX[0] ) Pos[0]=m_CBBX[0]+1e-2f;
    else if( Pos[0]>m_CBBX[1] ) Pos[0]=m_CBBX[1]-1e-2f;
    if( Pos[1]<m_CBBX[4] ) Pos[1]=m_CBBX[4]+1e-2f;
    else if( Pos[1]>m_CBBX[5] ) Pos[1]=m_CBBX[5]-1e-2f;
}

bool CBoundingBox::isCollision( const Point2d & oldPos )
{
    return !isInBBX(oldPos);
}

Point2d CBoundingBox::getRandomPoint(RNG * rng)
{
    float s=rng->uniform();
    float t=rng->uniform();
    Point2d randPos;
    randPos[0]=s*m_CBBX[0]+(1-s)*m_CBBX[1];
    randPos[1]=t*m_CBBX[4]+(1-t)*m_CBBX[5];

    return randPos;
}

float CBoundingBox::getClearance
( const Point2d & pos, Point2d & cd_pt )
{
    float min_dist=1e10;
    float dist[4]={pos[0]-m_CBBX[0],m_CBBX[1]-pos[0],
                    pos[1]-m_CBBX[4],m_CBBX[5]-pos[1]};
    if( dist[0]<min_dist ){     
        min_dist=dist[0];
        cd_pt.set(m_CBBX[0], pos[1]);
    }
    if( dist[1]<min_dist ){
        min_dist=dist[1];
        cd_pt.set(m_CBBX[1], pos[1]);
    }
    if( dist[2]<min_dist ){
        min_dist=dist[2];
        cd_pt.set(pos[0], m_CBBX[4]);
    }
    if( dist[3]<min_dist ){
        min_dist=dist[3];
        cd_pt.set(pos[0], m_CBBX[5]);
    }

    if( min_dist<0 ) min_dist=0;
    return min_dist;
}

//////////////////////////////////////////////////////////////////////
//Protected Methods
//////////////////////////////////////////////////////////////////////

void CBoundingBox::getCollisionInfo_Left
(const Point2d& oldPos,const Point2d& newPos,Vector2d& Normal,float& dCP)const
{
    //find collision point
    static Vector2d normal(1,0);
    getCollisionInfo(oldPos[0],newPos[0],m_CBBX[0],Normal,normal,dCP);
}

void CBoundingBox::getCollisionInfo_Right
(const Point2d& oldPos,const Point2d& newPos,Vector2d& Normal,float& dCP)const
{
    //find collision point
    static Vector2d normal(-1,0);
    getCollisionInfo(oldPos[0],newPos[0],m_CBBX[1],Normal,normal,dCP);
}

void CBoundingBox::getCollisionInfo_Front
(const Point2d& oldPos,const Point2d& newPos,Vector2d& Normal,float& dCP)const
{
    //find collision point
    static Vector2d normal(0,-1);
    getCollisionInfo(oldPos[1],newPos[1],m_CBBX[5],Normal,normal,dCP);
}

void CBoundingBox::getCollisionInfo_Back
(const Point2d& oldPos,const Point2d& newPos,Vector2d& Normal,float& dCP)const
{
    //find collision point
    static Vector2d normal(0,1);
    getCollisionInfo(oldPos[1],newPos[1],m_CBBX[4],Normal,normal,dCP);
}

void CBoundingBox::getCollisionInfo
(float dOld, float dNew, float dPlane, Vector2d& oldNormal, Vector2d& newNormal,float& dCP) const
{
    float disOld = fabs(dOld-dPlane);
    float disNew = fabs(dNew-dPlane);
    float dT= disOld/(disOld+disNew);
    if( dT>dCP ) return; //late collision
    if( dT==dCP ) oldNormal=oldNormal+newNormal;
    else oldNormal=newNormal;
    dCP=dT;
}

