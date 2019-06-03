#include "sh_CollisionDetection.h"

#ifdef _WIN32
#pragma warning(disable : 4786)
#endif




/******************************************************************************

  ENV-level collision detection

******************************************************************************/

//return true when collision found and information is put in  the corresponding variables.
bool shCD::getCollisionInfo
( CRobot2D & robot, const Point2d & oldPos, const Point2d & newPos,
  Vector2d & Normal/*out*/, float & dCP/*out, collision point*/ )
{
    //if( oldPos.almost_equ(newPos) ) return false;

	//check bbox first
    dCP=FLT_MAX;
    bool bResult=false; 
    if( m_pEnv->getBBX().getCollisionInfo(oldPos, newPos, Normal, dCP) ) bResult=true;

    //robot.buildCDLine(oldPos,newPos);

    //push the newPos further
    //Vector2d u=(newPos-oldPos).normalize();
    //Point2d newP=newPos+robot.getRadius()*u;

	//start checking ecah obst
    list<CObs*>& obsts=m_pEnv->getObstacles();
    typedef list<CObs*>::iterator OIT;
    for( OIT iO=obsts.begin();iO!=obsts.end();iO++ ){
        CObs* obst=*iO;
        int size=obst->getStateSize();
        CObs2D& geo=obst->getGeometry();
        for( int iI=0;iI<size;iI++ ){
            const CObsState& member=obst->getState(iI);
            obst->Configure(member);
            if( getCollisionInfo(geo, robot, oldPos, newPos, Normal, dCP) )
                bResult=true;
        }//end iI
    }//end OIT
    return bResult;//false if no collision
}

//check if the robot in pos is in collision
bool shCD::isCollision(CRobot2D & robot, const Point2d & pos)
{
    Last_ObState=NULL;
    if( m_pEnv->getBBX().isCollision(pos) ){ return true; }

    list<CObs*>& obsts=m_pEnv->getObstacles();
    typedef list<CObs*>::iterator OIT;

    for( OIT iO=obsts.begin();iO!=obsts.end();iO++ ){
        if( isCollision(*iO,robot,pos) )
            return true;
    }//end OIT
    return false;//false if no collision
}

//check if the straight line between pos1 and pos2 is in collision
bool shCD::isCollision(CRobot2D & robot, const Point2d & pos1, const Point2d & pos2)
{
    if( pos1.almost_equ(pos2) )
        return isCollision(robot,pos1);
    
    robot.buildCDLine(pos1,pos2);
    list<CObs*>& obsts=m_pEnv->getObstacles();
    typedef list<CObs*>::iterator OIT;

    for( OIT iO=obsts.begin();iO!=obsts.end();iO++ ){
        CObs* obst=*iO;
        int size=obst->getStateSize();
        CObs2D& geo=obst->getGeometry();
        for( int iI=0;iI<size;iI++ ){
            const CObsState& member=obst->getState(iI);
            obst->Configure(member);
            if( isCollision(geo,robot.getCDLine(),pos1,pos2) )
                return true;
        }//end iI
    }//end OIT
    return false;//false if no collision
}

//push the pos in to the free space
void shCD::Push(CRobot2D & robot, Point2d & pos )
{
    if( Last_ObState==NULL ) { cerr<<"! WARNING: Push: Last_ObState is NULL"<<endl; return; }
    CObs * obst=Last_ObState->getType();
    obst->Configure(*Last_ObState);
    Push(obst->getGeometry(),robot,pos);
}

//push the pos in to the free space
void shCD::PushInDir(CRobot2D & robot, const Vector2d& dir, Point2d & pos )
{
    if( Last_ObState==NULL ) {cerr<<"! WARNING: PushInDir: Last_ObState is NULL"<<endl; return; }
    CObs * obst=Last_ObState->getType();
    obst->Configure(*Last_ObState);
    PushInDir(obst->getGeometry(),robot,dir,pos);
}

//get clearance of the point
float shCD::getClearance(CRobot2D & robot, const Point2d & pos, Point2d & cd_pt)
{
    
    float min_Dist=m_pEnv->getBBX().getClearance(pos,cd_pt);

    list<CObs*>& obsts=m_pEnv->getObstacles();
    typedef list<CObs*>::iterator OIT;

    for( OIT iO=obsts.begin();iO!=obsts.end();iO++ ){
        CObs* obst=*iO;
        int size=obst->getStateSize();
        CObs2D& geo=obst->getGeometry();
        for( int iI=0;iI<size;iI++ ){
            //put obs in correct position
            const CObsState& member=obst->getState(iI);
            obst->Configure(member);
            //find clearance
            Point2d c;
            float dist = getClearance(geo,robot,pos,c,min_Dist);
            if( dist<min_Dist ){ min_Dist=dist; cd_pt=c; }
        }//end for iI
    }//end for iO

    return min_Dist;   
}

//push the pos in to the MA of the free space
float shCD::Push2Medial(CRobot2D & robot, Point2d & pos )
{
    Point2d closest;
    float clear=getClearance(robot,pos,closest);
    Vector2d dir=(pos-closest).normalize()*0.5;
    Point2d newClosest; 
    do{
        pos=pos+dir;
        clear=getClearance(robot,pos,newClosest);
    }while( (newClosest-closest).normsqr()<0.01 );
    
    return clear;
}

//obst boundary length 
float shCD::getObstBoundary(list< pair<Point2d,Point2d> >& boundaries)
{
    list<CObs*>& obsts=m_pEnv->getObstacles();
    typedef list<CObs*>::iterator OIT;
    float length=0;
    //get total boundary length of obstacle
    for( OIT i=obsts.begin();i!=obsts.end();i++ )
        length+=getObstBoundary(*i,boundaries);

    //get bounding box length
    const float * bbx=m_pEnv->getBBX().getBBXValue();
    Point2d p1(bbx[0],bbx[4]); Point2d p2(bbx[1],bbx[4]);
    Point2d p3(bbx[1],bbx[5]); Point2d p4(bbx[0],bbx[5]);
    boundaries.push_back(pair<Point2d,Point2d>(p2,p1));
    boundaries.push_back(pair<Point2d,Point2d>(p1,p4));
    boundaries.push_back(pair<Point2d,Point2d>(p4,p3));
    boundaries.push_back(pair<Point2d,Point2d>(p3,p2));

    return length+float(2*(bbx[1]-bbx[0]))+2*float((bbx[5]-bbx[4]));
}

//check if the straight line between pos1 and pos2 is in collision
bool shCD::isCollision(const CFlockState& s)
{
    const Point2d& p=s.getPos();
    return isCollision(s.getType()->getGeometry(),p);
}

//check if the straight line between pos1 and pos2 is in collision
bool shCD::isCollision(const CFlockState& s1, const CFlockState& s2)
{
    const Point2d& p1=s1.getPos();
    const Point2d& p2=s2.getPos();
    return isCollision(s1.getType()->getGeometry(),p1,p2);
}

//check if the straight line between pos1 and pos2 is in collision
bool shCD::isCollision(const CFlockState& s1, const Point2d & pos)
{
    const Point2d& p1=s1.getPos();
    return isCollision(s1.getType()->getGeometry(),p1,pos);
}

//push the pos in to the MA of the free space
float shCD::Push2Medial(const CFlockState& s1, Point2d & pos )
{
    return Push2Medial(s1.getType()->getGeometry(),pos);
}

//push the pos in to the free space
void shCD::Push(const CFlockState& s1, Point2d & pos )
{
    //if( !isCollision(m_pEnv,s1.getType()->getGeometry(),pos) ) return;
	//check if outside bbox
	if(m_pEnv->getBBX().isCollision(pos)){
		m_pEnv->getBBX().Push(pos);
	}
	else
		Push(s1.getType()->getGeometry(),pos);
}

//push the pos in to the free space
void shCD::PushInDir(const CFlockState& s1, const Vector2d& dir, Point2d & pos )
{
    //if( !isCollision(m_pEnv,s1.getType()->getGeometry(),pos) ) return;

	//cout<<" Before push="<<pos;
    PushInDir(s1.getType()->getGeometry(),dir,pos);
	//cout<<" After push="<<pos<<endl;
}


//
// temp check line collision
//
bool shCD::mychecklinecollision(const Point2d& p1, const Point2d& p2, CFlockState& s)
{
	//setup robot
	CRobot2D & robot=s.getType()->getGeometry();
	robot.buildCDLine(p1,p2);

	//start checking ecah obst
    list<CObs*>& obsts=m_pEnv->getObstacles();
    typedef list<CObs*>::iterator OIT;
    for( OIT iO=obsts.begin();iO!=obsts.end();iO++ ){
        CObs* obst=*iO;
        int size=obst->getStateSize();
        CObs2D& geo=obst->getGeometry();
        for( int iI=0;iI<size;iI++ ){
            const CObsState& member=obst->getState(iI);
            obst->Configure(member);
            if( isCollision(geo, robot.getCDLine(), p1, p2) )
				return true;
        }//end iI
    }//end OIT

	return false;
}


/******************************************************************************

  OBST-level collision detection

******************************************************************************/
bool shCD::isCollision(CObs * obst, const CFlockState& s)
{
    const Point2d& p=s.getPos();
    return isCollision(obst,s.getType()->getGeometry(),p);
}

bool shCD::isCollision(CObs * obst, CRobot2D & robot, const Point2d & pos)
{
    int size=obst->getStateSize();
    CObs2D& geo=obst->getGeometry();
    for( int iI=0;iI<size;iI++ ){
        const CObsState& member=obst->getState(iI);
        obst->Configure(member);
        if( isCollision(geo,robot,pos,true) ){
            Last_ObState=&member;
            return true;
        }//end if
    }//end iI
    return false;
}

bool shCD::isCollision( CObs2D& obst, CRobot2D & robot, const Point2d & pos, bool gatherInfo)
{
    CDTriID.clear();
    const Matrix3x3 & Rot=obst.getRot();
    const Point3d& Pos=obst.getPos();

    //check roughly
    Point2d T(Pos[0],Pos[2]);
    if( robot.getRadius()+obst.getRadius()<(pos-T).norm() ) 
        return false;

    //check more precisely
    //go through each triangle
    const PtVector & Geo=obst.getGeo();
    const TriVector& Tri=obst.getTri();
    int size=Tri.size();
    bool in_collision=false;
    float R2=robot.getRadius()*robot.getRadius();
    for( int iT=0; iT<size;iT++ ){ //for each tri
        Point2d T[3]; //coordinate of corners of tri
        for(int i=0;i<3;i++)
            T[i]=toGlobal(Geo[Tri[iT][i]],Rot,Pos);
        bool r=isCollision(T[0],T[1],T[2],pos,R2);
        if(r && !gatherInfo) return true;
        if(r && gatherInfo){ CDTriID.push_back(iT); in_collision=true; }
    }

    return in_collision;
}


//check collision between the line and the obstacle
bool shCD::isCollision(CObs2D& obst, RAPID_model * line, 
const Point2d & pos1, const Point2d & pos2, bool gatherInfo)
{
    CDTriID.clear();

	//RAPID seems to have problem with model with no triangles
	//projected on the ground....
	if(	obst.getTri().empty() ) return false;

    const Matrix3x3 & Rot=obst.getRot();
    const Point3d& Pos=obst.getPos();

    static double robR[3][3]={ {1,0,0}, {0,1,0}, {0,0,1} };
    static double robT[3]={0,0,0};
    double obsR[3][3]={
    {Rot[0][0],Rot[0][1],Rot[0][2]}, 
    {Rot[1][0],Rot[1][1],Rot[1][2]}, 
    {Rot[2][0],Rot[2][1],Rot[2][2]} };
    double obsT[3]={Pos[0],0,Pos[2]};
	
	RAPIDres res;
    RAPID_Collide(res,robR,robT,line,obsR,obsT,&obst.getCDModel(),RAPID_ALL_CONTACTS);

    //gather info for later
    if( gatherInfo ){
        for( int iT=0;iT<res.RAPID_num_contacts;iT++ ){
            int added=false;
            for( int iCD=0;iCD<(int)CDTriID.size();iCD++ ){ //not sure that this is...
                if(CDTriID[iCD]==res.RAPID_contact[iT].id2){added=true; break;}
            }
            if( !added ) CDTriID.push_back(res.RAPID_contact[iT].id2);
        }
    }

    return (res.RAPID_num_contacts>0);
}


float shCD::getClearance
( CObs2D& obst, CRobot2D & robot, const Point2d & pos, Point2d & cd_pt, float clear )
{
    const Matrix3x3 & Rot=obst.getRot();
    const Point3d& Pos=obst.getPos();
    float min_Dis=FLT_MAX;

    //check roughly
    Point2d T(Pos[0],Pos[2]);
    if( ((pos-T).norm()-obst.getRadius())>clear ) return min_Dis;

    const PtVector & Geo=obst.getGeo();
    const TriVector& Tri=obst.getTri();
    const TriVector& Line=obst.getLine(); //indicate which edge is border
    int size=Tri.size();
    for( int iT=0; iT<size;iT++ ){ //for each tri
        for( int iD=0;iD<3;iD++ ){ //for each boundary edge of this tri
            if( Line[iT][iD]==0 ) continue; //not a border
            pair<Point2d,Point2d> e=getEdge(iD,Tri[iT],Geo,Rot,Pos);
            //find closest pt from pos to p1p2
            Point2d c;
            float dist=distsqr(pos,e.first,e.second,c);
            if( dist<min_Dis) { min_Dis=dist; cd_pt=c;}
        }
    }
    
    return sqrt(min_Dis);
}

void shCD::Push( CObs2D& obst, CRobot2D & robot, Point2d & pos )
{
    int size=CDTriID.size();
    if( size==0 ) return;
    const Matrix3x3 & Rot=obst.getRot();
    const Point3d& Pos=obst.getPos();
    const PtVector & Geo=obst.getGeo();
    const TriVector& Tri=obst.getTri();
    const TriVector& Line=obst.getLine();
    
    float min_Dis=1e10f; Point2d cloest;
    for( int iT=0; iT<size;iT++ ){
        int tri=CDTriID[iT];
        for( int iD=0;iD<3;iD++ ){
            if( Line[tri][iD]==0 ) continue;
            pair<Point2d,Point2d> e=getEdge(iD,Tri[tri],Geo,Rot,Pos);
            Point2d c; //used for closest points
            float dist=distsqr(pos,e.first,e.second,c);
            if( dist<min_Dis){min_Dis=dist;  cloest=c;}
        }
    }
	
	min_Dis=sqrt(min_Dis);
	Vector2d v=(pos-cloest).normalize();

	Point2d tmp=pos+v*(robot.getRadius()-min_Dis+1e-3);
	if( !isCollision(obst,robot,tmp) ){ pos=tmp; return; }
	pos=pos-v*(min_Dis+robot.getRadius()+1e-3);
}

void shCD::PushInDir( CObs2D& obst, CRobot2D & robot, const Vector2d& v, Point2d & pos )
{
    int size=CDTriID.size();
    if( size==0 ) return;
    const Matrix3x3 & Rot=obst.getRot();
    const Point3d& Pos=obst.getPos();
    const PtVector & Geo=obst.getGeo();
    const TriVector& Tri=obst.getTri();
    const TriVector& Line=obst.getLine();

    Point2d qpt=pos+v*10;
    pair<Point2d,Point2d> qe(pos,qpt);

    float min_Dis=1e10f; Point2d cloest;
    for( int iT=0; iT<size;iT++ ){
        int tri=CDTriID[iT];
        for( int iD=0;iD<3;iD++ ){
            if( Line[tri][iD]==0 ) continue;
            pair<Point2d,Point2d> e=getEdge(iD,Tri[tri],Geo,Rot,Pos);
            Point2d c=LineSegIntersect(e,qe);
            float dist=(pos-c).normsqr();
            if( dist<min_Dis){min_Dis=dist;  cloest=c;}
        }
    }

	min_Dis=sqrt(min_Dis); //(robot.getRadius()+1e-5); //push further
	Vector2d new_v=(cloest-pos).normalize();
	
	int try_num=100;
	while(try_num>0){
	    Point2d tmp_p1=pos+(new_v*min_Dis);
    	if( !isCollision(obst,robot,tmp_p1) ){ pos=tmp_p1; return; }
		try_num--;
		min_Dis+=1e-2;
	}
}

bool shCD::getCollisionInfo
( CObs2D& obst,
  CRobot2D & robot, const Point2d & oldPos, const Point2d & newPos,
  Vector2d & Normal/*out*/, float & dCP/*out, collision point*/ )
{
   // if( isCollision(obst,robot.getCDLine(),oldPos,newPos,true)==false ) 
   //     return false;
   
    if(!isCollision(obst,robot,newPos,true)) return false;

    //push the newPos further
    Vector2d u=(newPos-oldPos).normalize();
    Point2d newP=newPos+robot.getRadius()*u;

    //find collision point
    float min_T=dCP;
    Vector2d normal;
    const PtVector & Geo=obst.getGeo();
    const TriVector& Tri=obst.getTri();
    const TriVector& Line=obst.getLine();
    const Matrix3x3 & Rot=obst.getRot();
    const Point3d& Pos=obst.getPos();

    int size=CDTriID.size();
	
	bool found=false; //recording if we find something smaller

    for( int iT=0; iT<size;iT++ ){
        int tri=CDTriID[iT];
        for( int iD=0;iD<3;iD++ ){
            if( Line[tri][iD]==0 ) continue;
            pair<Point2d,Point2d> e=getEdge(iD,Tri[tri],Geo,Rot,Pos);
            const Point2d& p1=e.first;
            const Point2d& p2=e.second;
            //find closest pt from pos to p1p2
            Vector2d v=(p1-p2).normalize(); 
            Vector2d n(-v[1],v[0]);
			float c_b=n*(newP-oldPos);
			if(c_b>0) continue; //wrong direction!
            float a_b=n*(p1-oldPos);			
            float t=(a_b)/(c_b);

            if( t<0 ) continue;
            if( t<min_T ){ min_T=t; normal=n; found=true;}
        }

		if(found) break;

    }//end for iT

    if(min_T>=dCP) 
	   return false;

    //update dCP and normal
    dCP=min_T;
    Normal=normal;

    return true;
}

float shCD::getObstBoundary
(CObs2D& obst, list< pair<Point2d,Point2d> >& boundaries)
{
    float length=0;
    const PtVector & Geo=obst.getGeo();
    const TriVector& Tri=obst.getTri();
    const TriVector& Line=obst.getLine();
    const Matrix3x3 & Rot=obst.getRot();
    const Point3d& Pos=obst.getPos();
    int size=Tri.size();
    for( int iT=0; iT<size;iT++ ){
        for( int iD=0;iD<3;iD++ ){
            if( Line[iT][iD]==0 ) continue;
            pair<Point2d,Point2d> e=getEdge(iD,Tri[iT],Geo,Rot,Pos);
            length+=float((e.first-e.second).norm());
            boundaries.push_back(e);
        }
    }//end for iT
    return length;
}

//obst boundary length 
float shCD::getObstBoundary(CObs * obst,list< pair<Point2d,Point2d> >& boundaries)
{
    float length=0;
    //get total boundary length of obstacle
    int size=obst->getStateSize();
    for( int iI=0;iI<size;iI++ ){
        const CObsState& state=obst->getState(iI);
        obst->Configure(state);
        length+=getObstBoundary(obst->getGeometry(),boundaries);
    }//end iI

    return length;
}



