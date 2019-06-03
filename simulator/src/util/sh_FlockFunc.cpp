
#include "sh_FlockFunc.h"
#include "sh_FlockState.h"
#include "sh_CollisionDetection.h"

#ifdef _WIN32
#pragma warning(disable : 4786)
#endif

#include "graph/Graph.h"
#include "graph/GraphAlgo.h"
using namespace graph;

bool visible
(CFlockState& s, const Point2d& pt, CEnvironment* env)
{
    float R=s.getType()->getViewRadius();
    float A=s.getType()->getViewAngle();
    float R2=R*R;
    float  A2=float(A/2);
    Vector2d xD=s.getFacingDir();

    Vector2d dir=pt-s.getPos();
    float dist_sqr=dir.normsqr();
    if( dist_sqr>R2 ){
        return false; //too far
    }
    if( acos((dir/sqrt(dist_sqr))*xD)>A2 ){
        return false; //not in the range
    }
    if( env!=NULL ){
    	shCD cd(env);
        if( cd.isCollision(s.getType()->getGeometry(),s.getPos(),pt) ){
            return false;
        }
    }
    return true;
}

bool visible(CFlockState& s1, CFlockState& s2, CEnvironment* env)
{
    return visible(s1,s2.getPos(),env);
}

///////////////////////////////////////////////////////////////////////////////
struct _n{
    _n(){state=NULL;} 
    static _n InvalidData(){return _n();} 
    CFlockState* state;
};
struct _w{ float weight; };
typedef Graph<UG<_n,_w>,NMG<_n,_w>,WG<_n,_w>,_n,_w> GG;

list<FSLIST> 
getGroups(FSLIST& fslist,float dist,CEnvironment * env)
{
    GG g;
    list<FSLIST> fsl_list;

    _n node;
    _w edge; edge.weight=1;
    shCD cd(env);

	if(dist<0) dist=1e10;
	float dist_sqr=dist*dist;
    ///////////////////////////////////////////////////////////////////////////
    // Construct the graph

    //add nodes to the graph
    typedef FSLIST::iterator SIT;
    {for( SIT i=fslist.begin();i!=fslist.end();i++){
        node.state=*i;
        g.AddVertex(node);
    }}
    //add edge
    int size=fslist.size();
    {for(int i=0;i<size;i++){
        _n ni=g.GetData(i);
        for(int j=i+1;j<size;j++){
            _n nj=g.GetData(j);
			float d_sqr=(float)(ni.state->getPos()-nj.state->getPos()).normsqr();
			if( d_sqr>dist_sqr ) continue; //too far
            if( !visible(*ni.state,*nj.state) ) continue;
            if( !visible(*nj.state,*ni.state) ) continue;
            if( env!=NULL ) //check if in collision
                if( cd.isCollision(*ni.state,*nj.state) ) continue;
            g.AddEdge(i,j,edge);
        }
    }}

    ///////////////////////////////////////////////////////////////////////////
    vector< pair<int,VID> > ccs;
    GetCCStats(g,ccs);

    typedef vector< pair<int,VID> >::iterator CIT;
    {for(CIT i=ccs.begin();i!=ccs.end();i++){
        vector<VID> cc;
        GetCC (g,i->second,cc);
        typedef vector<VID>::iterator VIT;
        FSLIST group;
        for(VIT iv=cc.begin();iv!=cc.end();iv++)
            group.push_back(g.GetData(*iv).state);
        fsl_list.push_back(group);
    }}

    return fsl_list;
}

ostream& operator<<(ostream& o,const _n&n){return o;}
ostream& operator<<(ostream& o,const _w&w){return o;}
bool operator==(const _n& n1, const _n& n2){return n1.state==n2.state;}

///////////////////////////////////////////////////////////////////////////////

inline Point2d g_center(const FSLIST& g)
{
	Point2d center;
	int total=0;
	for(FSLIST::const_iterator i=g.begin();i!=g.end();i++){
		center[0]+=(*i)->getPos()[0];
		center[1]+=(*i)->getPos()[1];
		total++;
	}
	center[0]/=total;
	center[1]/=total;
	return center;
}

inline void divid_CA
(const FSLIST& g, const Point2d& center, float r_c,FSLIST& gin, FSLIST& gout)
{
	float r_c_2=r_c*r_c;
	for(FSLIST::const_iterator i=g.begin();i!=g.end();i++){
		CFlockState* s=*i;
		float dist=(s->getPos()-center).normsqr();
		if( dist<r_c_2 ) gin.push_back(s);
		else gout.push_back(s);
	}
}

inline CFlockState* closetState(const FSLIST& g,const Point2d& center)
{
	float min_d=1e10;
	CFlockState* min_s=NULL;

	for(FSLIST::const_iterator i=g.begin();i!=g.end();i++){
		CFlockState* s=*i;
		float dist=(s->getPos()-center).normsqr();
		if( dist<min_d ){
			min_d=dist;
			min_s=s;
		}
	}
	return min_s;
}

//get groups with compact zone
list<FSLIST> 
getGroups_CA(FSLIST& fslist, float scale, CEnvironment * env)
{
	list<FSLIST> groups=getGroups(fslist,1e10,env);
	list<FSLIST> groups_ca; //re-grouped using ca

	for(list<FSLIST>::iterator g=groups.begin();g!=groups.end();g++){
		int gsize=g->size();
		Point2d center=g_center(*g);
		float r_f=g->front()->getType()->getGeometry().getRadius();
		float r_c=r_f*sqrt(gsize*2.0f)*scale;

		FSLIST out,in;
		divid_CA(*g,center,r_c,in,out);

		if( !in.empty() )
			groups_ca.push_back(in);
		else{
			center=closetState(*g,center)->getPos();
			out.clear();
			divid_CA(*g,center,r_c,in,out);
			groups_ca.push_back(in);
		}

		if( !out.empty() ){
			if( !in.empty() ){
				list<FSLIST> out_groups=getGroups_CA(out,scale,env);
				groups_ca.insert(groups_ca.end(),out_groups.begin(),out_groups.end());
			}
			else{ //avoid infinite loop
				groups_ca.push_back(out); 
			}
		}
	}//end for
	return groups_ca;
}

///////////////////////////////////////////////////////////////////////////////

Point2d allGroupsSafeZone
(CFlockState& s, const Point2d& goal,
 const list<Point2d>& centers, const list<float>& radii)
{
	const Point2d& start=s.getPos();
	Vector2d vec=(goal-start).normalize();
	Vector2d n(-vec[1],vec[0]);
	//check if s->g interesects any circle
	bool intersect=false;
	typedef list<Point2d>::const_iterator CIT;
	typedef list<float>::const_iterator  RIT;
	CIT ic=centers.begin();
	RIT ir=radii.begin();
	for(;ic!=centers.end();ic++,ir++){
		float dist=fabs(n*((*ic)-start));
		if( dist<*ir ){
			intersect=true; 
			break;
		}
	}
	//
	if( !intersect ) return goal;
	//find the break point
	Point2d mid( (start[0]+goal[0])/2,(start[1]+goal[1])/2);
	ic=centers.begin();
	ir=radii.begin();
	float max_off=-1e10;
	float min_off=1e10;
	for(;ic!=centers.end();ic++,ir++){
		Vector2d cv=(*ic)-mid;
		float dist=fabs(vec*cv);
		if( dist<*ir ){
			float off=cv*n;
			if(off>0){
				off+=(*ir);
				if( off>max_off ) max_off=off;
			}
			else{
				off-=(*ir);
				if( off<min_off ) min_off=off;
			}
		}//end if
	}//end for
	//create now goal
	if( max_off<-min_off ) 
		return allGroupsSafeZone(s,mid+(n*max_off),centers,radii);
	else 
		return allGroupsSafeZone(s,mid+(n*min_off),centers,radii);
}

//smallest enclosing circle
inline pair<float, Point2d> 
SEC(const Point2d& p1, const Point2d& p2, const Point2d& p3)
{
	Vector2d v1=p1-p2;
	Vector2d v2=p2-p3;
	Vector2d v3=p3-p1;
	Point2d c;
	float r;

	float d=fabs(v1[0]*v2[1]-v1[1]*v2[0]);
	d=d*d;
	if(d==0){
		c.set( (p1[0]+p2[0]+p3[0])/3, (p1[1]+p2[1]+p3[1])/3);
		r=(c-p1).normsqr();
	}
	else{
		float v1_normsqr=v1.normsqr();
		float v2_normsqr=v2.normsqr();
		float v3_normsqr=v3.normsqr();
		r= (v1_normsqr*v2_normsqr*v3_normsqr)/(4*d);
		float alpha=-v2_normsqr*(v1*v3)/(2*d);
		float betta=-v3_normsqr*(v1*v2)/(2*d);
		float gamma=-v1_normsqr*(v2*v3)/(2*d);
		c.set(alpha*p1[0]+betta*p2[0]+gamma*p3[0], alpha*p1[1]+betta*p2[1]+gamma*p3[1]);
	}

	return pair<float, Point2d>(r,c);
}


//smallest enclosing circle
inline pair<float, Point2d> 
SEC(FSLIST::iterator s, FSLIST::iterator e, CFlockState* q1, CFlockState* q2)
{
	const Point2d& p1=q1->getPos();
	const Point2d& p2=q2->getPos();

	Point2d c( (p1[0]+p2[0])/2, (p1[1]+p2[1])/2);
	float r=(p1-c).normsqr();

	for(;s!=e;s++){
		const Point2d& pos=(*s)->getPos();
		float d=(pos-c).normsqr();
		if(d>r){ //get a new circle
			pair<float, Point2d> new_d=SEC(p1,p2,pos);
			c=new_d.second;
			r=new_d.first;
		}
	}//end for

	return pair<float, Point2d>(r,c);
}

//smallest enclosing circle
inline pair<float, Point2d> 
SEC(FSLIST::iterator s, FSLIST::iterator e, CFlockState* q)
{	
	FSLIST::iterator i=s;
	const Point2d& p1=(*i)->getPos(); i++; //first point
	const Point2d& p2=q->getPos();

	Point2d c( (p1[0]+p2[0])/2, (p1[1]+p2[1])/2);
	float r=(p1-c).normsqr();

	for(;i!=e;i++){
		const Point2d& pos=(*i)->getPos();
		float d=(pos-c).normsqr();
		if(d>r){ //get a new circle
			pair<float, Point2d> new_circle=SEC(s,i,q,*i);
			r=new_circle.first;
			c=new_circle.second;
		}
	}//end for

	return pair<float, Point2d>(r,c);
}


//compute the center and radius of a set of flock states
pair<float, Point2d> findEC(FSLIST& states)
{
	int size=states.size();
	if(size<2) return pair<float, Point2d>(2,states.front()->getPos());

	FSLIST::iterator i=states.begin();
	const Point2d& p1=(*i)->getPos(); i++; //first point
	const Point2d& p2=(*i)->getPos(); i++; //second point

	Point2d center( (p1[0]+p2[0])/2, (p1[1]+p2[1])/2);
	float radius=(p1-center).normsqr();

	for(;i!=states.end();i++){
		const Point2d& pos=(*i)->getPos();
		float d=(pos-center).normsqr();
		if(d>radius){ //get a new circle
			pair<float, Point2d> new_circle=SEC(states.begin(),i,*i);
			radius=new_circle.first;
			center=new_circle.second;
		}
	}//end i

	radius=sqrt(radius);
	if(radius<2) radius=2;
	return pair<float, Point2d>(radius,center);
}

