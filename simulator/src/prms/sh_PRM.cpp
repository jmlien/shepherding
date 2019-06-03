// PRMS.cpp: implementation of the PRMS class.
//
//////////////////////////////////////////////////////////////////////
#ifdef _WIN32
#pragma warning(disable : 4786)
#endif

#include "sh_PRM.h"
#include "sh_Environment.h"
#include "sh_BoundingBox.h"
#include "sh_FlockState.h"
#include "sh_ObstState.h"

#include <assert.h>

#include "graph/GraphAlgo.h"
#include "util/gettime.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

PRMS::PRMS(CEnvironment * env, RNG * rng)
:m_CD(env)
{
    m_pEnv=env;
    m_pRNG=rng;
    if( m_pEnv==NULL || m_pRNG==NULL ) throw "! Error : PRMS init error";
    m_create_good_loops=true;
    m_skip_same_cc=true;
}

PRMS::~PRMS()
{
    m_pEnv=NULL;
    m_pRNG=NULL;
}

//////////////////////////////////////////////////////////////////////
// Core

void PRMS::samplePRMNodes(CRoadMap& rmap, int n,float spacing)
{
	cout<<"- PRM ("<<n<<" nodes) "<<endl;

	double st=getTime();

    CFlock * m_F=rmap.getFlock();

    CBoundingBox& bbx=m_pEnv->getBBX();

    CRobot2D& geo=m_F->getGeometry();

    bbx.createCSPace(m_F->getGeometry().getRadius());

    for( int iN=0;iN<n;iN++ ){

        Point2d cfg=bbx.getRandomPoint(m_pRNG);
        //push if in collision
        if(m_CD.isCollision(geo,cfg)){
	        continue;
        }

        //put into roadmap
        CNode node; node.setPos(cfg); //node.setClearance(clear);
        rmap.addNode(node);
    }
    
	double et=getTime();    

    cout<<"- PRM done takes "<<et-st<<" msec"<<endl;
}

void PRMS::sampleMAPRMNodes(CRoadMap& rmap, int n,float spacing)
{
	cout<<"- MAPRM ("<<n<<" nodes): "<<flush;

    CFlock * m_F=rmap.getFlock();

    CBoundingBox& bbx=m_pEnv->getBBX();

    CRobot2D& geo=m_F->getGeometry();

    bbx.createCSPace(m_F->getGeometry().getRadius());

	//spacing
	float delta=0;
    if(spacing>0) delta=spacing;
	//spacing

	int attmpt=0;

	double st=getTime();

    for( int iN=0;iN<n;iN++ ){


        Point2d cfg=bbx.getRandomPoint(m_pRNG);
        //push if in collision
        if(m_CD.isCollision(geo,cfg)){
            m_CD.Push(geo,cfg);
            if(m_CD.isCollision(geo,cfg)){iN--; continue;} //still in m_CD
        }

        //put into roadmap
        m_CD.Push2Medial(geo,cfg); //push to the medial axis

		//may push out of bbx
		if(m_pEnv->getBBX().isCollision(cfg)){iN--; continue;} //become in m_CD

		attmpt++;
		if(attmpt>n) break; //attempted too many times....

		//check spacing
		if(delta>0)
		{
			float close=false;
			for( int i=0;i<iN;i++ ){
				if( (rmap.getNode(i)->getPos()-cfg).normsqr()<delta ){
					close=true; break;
				}
			}
			if( close ){ iN--; continue; }
		}

		attmpt=0;
        CNode node; node.setPos(cfg); //node.setClearance(clear);
        rmap.addNode(node);
        //report progress
        int percent1=(iN*100.0/n);
        int percent2=(iN-1)*100.0/n;
        if( percent1%10==0 && percent1!=percent2 && iN!=0) cout<<"=>"<<(percent1)<<"%"<<flush;
    }
    
	double et=getTime();    

    cout<<"\n- MAPRM takes "<<et-st<<" msec"<<endl;
}

void PRMS::sampleOBPRMNodes(CRoadMap& rmap, int n,float spacing)
{
    typedef pair<Point2d,Point2d> line;
	double st=getTime();
    list<line> boundaries;
    float length=m_CD.getObstBoundary(boundaries);
    float ratio=((float)n)/length;
    sampleOBPRMNodes_priviate(rmap,n,spacing,boundaries,length,ratio);
    double et=getTime();    
    cout<<"- OBPRM takes "<<et-st<<" msec"<<endl;
}

int PRMS::sampleOBPRMNodes_on_Obsts
(CObs& obst,CRoadMap& rmap,int n,float spacing)
{
    typedef pair<Point2d,Point2d> line;

    list<line> boundaries;
    float length=m_CD.getObstBoundary(&obst,boundaries);
    float ratio=((float)n)/length;
    return
    sampleOBPRMNodes_priviate(rmap,n,spacing,boundaries,length,ratio);
}

int PRMS::sampleOBPRMNodes_on_Obst
(CObs& obst,CObsState& state,CRoadMap& rmap,int n,float spacing)
{
    typedef pair<Point2d,Point2d> line;
    list<line> boundaries;
    obst.Configure(state);
    float length=m_CD.getObstBoundary(obst.getGeometry(),boundaries);
    float ratio=((float)n)/length;
    return sampleOBPRMNodes_priviate(rmap,n,spacing,boundaries,length,ratio);
}

int PRMS::sampleOBPRMNodes_priviate
(CRoadMap& rmap,int n,float spacing,
 list< pair<Point2d,Point2d> >& boundaries,float length, float ratio)
{
	cout<<"- OBPRM ("<<n<<" nodes)"<<endl;

    typedef pair<Point2d,Point2d> line;
    typedef list<line>::iterator LIT;

    CFlock * m_F=rmap.getFlock();
    float r=m_F->getGeometry().getRadius()*2;
    if(spacing>0) r=spacing;
    CRobot2D& geo=m_F->getGeometry();
    m_pEnv->getBBX().createCSPace(geo.getRadius());

    int new_node_size=0;
    for( LIT i=boundaries.begin();i!=boundaries.end();i++ ){
        line& l=*i;
        Vector2d v=(l.first-l.second);
        Vector2d nv(-v[1],v[0]);
        float vl=v.norm();
        nv=nv*(r/vl);
        int size=(int)ceil(ratio*vl);
        Vector2d dv=v/size;
        Point2d s=l.second+(-dv);
        for( int j=0;j<=size;j++ ){
            s=s+dv;
            Point2d cfg=s+nv;

            if( rmap.getNodeSize()>0 ){
                Point2d last_cfg=rmap.getNode(rmap.getNodeSize()-1)->getPos();
                if( (cfg-last_cfg).norm()<spacing ) continue;
            }
            if(m_CD.isCollision(geo,cfg)){
                //Push(*m_pEnv,geo,cfg);
                //if(isCollision(*m_pEnv,geo,cfg)){continue;} //still in m_CD
                continue;
            }
            CNode node; node.setPos(cfg);
            rmap.addNode(node);
            new_node_size++;
        }//end j
    }//end i

    return new_node_size;
}

void PRMS::connectNodes(CRoadMap& rmap, int k)
{
	double st=getTime(); 
	
    list< pair<int,int> > closest;

    //for each node find k cloest  
    vector<CNodeData*>& nodes=rmap.getNodes();
    sortedPairs(nodes,closest,k);
	connectNodes(rmap,closest);
	
	//done
	double et=getTime();    
    cout<<"\n- Connect takes "<<et-st<<" msec"<<endl;
}

//connect each node in rmap to nodes within radius distance
void PRMS::connectNodes(CRoadMap& rmap, float radius)
{
	double st=getTime(); 
	
    list< pair<int,int> > closest;

    //for each node find k cloest  
    vector<CNodeData*>& nodes=rmap.getNodes();
    nearPairs(nodes,closest,radius);
	connectNodes(rmap,closest);
	
	//done
	double et=getTime();    
    cout<<"\n- Connect takes "<<et-st<<" msec"<<endl;	
}

//connect pairs of nodes in closest  
void PRMS::connectNodes(CRoadMap& rmap, list< pair<int,int> >& closest)
{
    CRobot2D& geo=rmap.getFlock()->getGeometry();
    WDG& g=rmap.getGraph();
    vector<CNodeData*>& nodes=rmap.getNodes();
    
    list< pair<int,int> > loop; //store all found loops
    
	cout<<"- Connect ("<<closest.size()<<" pairs)"<<flush;
	int checked_n=0;
	int size=closest.size();
	typedef list< pair<int,int> >::iterator PIT;
    for( PIT i=closest.begin();i!=closest.end();i++, checked_n++)
    {
	    //report progress
        int percent1=(checked_n*100.0/size);
        int percent2=(checked_n-1)*100.0/size;
        if( percent1%10==0 && percent1!=percent2 && checked_n!=0) cout<<"=>"<<(percent1)<<"%"<<flush;    
    
	    //check the pair    
        int vid1=i->first;
        int vid2=i->second;
		
		if(g.IsEdge(vid1,vid2)) continue;
		//if(g.IsEdge(vid2,vid1)) continue;	//the graph is undirected	
		
		if(m_skip_same_cc||m_create_good_loops)
		{
			if(IsSameCC(g,vid1,vid2)){
				loop.push_back(*i);
				if(m_skip_same_cc) continue;
			}
		}//end if
		
        const Point2d& pos1=nodes[vid1]->getPos();
        const Point2d& pos2=nodes[vid2]->getPos();

        if( !m_CD.isCollision(geo,pos1,pos2) )
            rmap.addEdge(vid1,vid2);
    }
	
	//find good loops! (this is very slow...)
	if(m_create_good_loops && m_skip_same_cc) 
		findGoodLoops(rmap,nodes,loop);

    vector< pair<int,VID> > ccstats;
    int ccsize=GetCCStats(g,ccstats);
    cout<<"- CC#="<<ccsize<<"[";
    for(int i=0;i<ccsize;i++){
    	cout<<ccstats[i].first;
    	if(i!=ccsize-1) cout<<", ";
    }
	cout<<"]"<<endl;
}

  
//connect robot from pos to rmap
CNodeData * PRMS::connect2Map(CRoadMap& rmap, const Point2d& pos)
{
    //CFlock * m_F=rmap.getFlock();
    /////////////////////////////////////////////////////////////////
    //CRobot2D& geo=m_F->getGeometry();  //for collision detection
    //if( isCollision(*m_pEnv,geo,pos) ){
    //    return NULL;
    //}

    /////////////////////////////////////////////////////////////////
    WDG & graph = rmap.getGraph();
    vector<CNode> nodes;
    graph.GetVerticesData(nodes);

    list< pair<float,int> > sorted;
    typedef vector<CNode>::iterator NIT;
    {for( NIT i=nodes.begin();i!=nodes.end();i++ ){
        float dist = (i->getPos()-pos).normsqr();
        if( dist<1e-10 ) 
            return rmap.getNode(i->getID());
        sorted.push_back( pair<float,int>(dist,i-nodes.begin()) );
    }}//end for i

    sorted.sort();
    typedef list< pair<float,int> >::iterator LIT;
    {for( LIT i=sorted.begin();i!=sorted.end();i++ ){
        CNode & node=nodes[i->second];
        const Point2d & nodepos=node.getPos();
        CRobot2D dummy;
        if( !m_CD.isCollision(dummy,pos,nodepos) )
            return rmap.getNode(node.getID());
    }}//end for i

    return NULL;
}

void PRMS::findPath
(CRoadMap& rmap,const Point2d& s,const Point2d& g,list<Point2d>& path)
{
    static CNodeData * catchedN1=NULL;
    static CNodeData * catchedN2=NULL;
    static list<Point2d> catchedPath;
	CRobot2D& geo=rmap.getFlock()->getGeometry();

	//check direct connection
	if( !m_CD.isCollision(geo,s,g) ){
		calcPathCfg(rmap,s,g,path); 
		return;
	}

    //connect to roadmap...
    CNodeData * n1=connect2Map(rmap,s);  //start connect to goal
    CNodeData * n2=connect2Map(rmap,g);  //goal connect to goal
    if( n1==NULL || n2==NULL ){
        //cerr<<"! Warning : Start or/and Goal can not connect to the map"<<endl;
        return;
    }

    list<Point2d> mypath;
    mypath.push_back(s);
	//check catch
    if( catchedN1!=n1 || catchedN2!=n2 ){ //check cache
        //put to catch
        catchedPath.clear();
		//ok...not in catch, then we create a new path
        if( !findPathV1V2(rmap,n1->getID(),n2->getID(),catchedPath) ){
            cerr<<"! Warning : Start can not connect to Goal"<<endl;
            return;
        }
        catchedN1=n1; catchedN2=n2;
    }

    mypath.insert(mypath.end(),catchedPath.begin(),catchedPath.end());
    mypath.push_back(g);

    typedef list<Point2d>::iterator PIT;
    PIT last=mypath.end(); last--;
    for(PIT i=mypath.begin();i!=last;i++){
        PIT j=i; j++;
		//remove nodes that are directly visible
		if(!m_CD.isCollision(geo,s,*j)) continue;
        calcPathCfg(rmap,*i,*j,path); //start
    }
    path.push_back(mypath.back());

    return;
}


bool PRMS::findPathV1V2
(CRoadMap& rmap, int v1, int v2, list<Point2d>& path)
{
    if( v1==v2 ) return true;
    vector< pair<CNode,CEdge> > nodes;
    typedef vector< pair<CNode,CEdge> >::iterator NIT;
    int size=FindPathDijkstra<WDG>(rmap.getGraph(),v1,v2,nodes);
    if( size<=0 ) return false;

    for( NIT i=nodes.begin();i!=nodes.end();i++ ){
        path.push_back(i->first.getPos());
    }//end for

    return true;
}

//compute cfgs from p1 to p2
void PRMS::calcPathCfg
(CRoadMap& rmap,const Point2d& p1, const Point2d& p2, list<Point2d>& path)
{
    //compute path size and step
    float step_size=rmap.getFlock()->getViewRadius()/2;
    Vector2d v=p2-p1;
    float vn=v.norm();
    step_size=(step_size>vn)?vn:step_size;
    int size=(int)ceil(vn/step_size);
    if( size<2 ) size=2;
    Vector2d dir=v*(step_size/vn);

    //build path
    for( int i=0;i<size-1;i++ )
        path.push_back(p1+dir*i);
}

//////////////////////////////////////////////////////////////////////
// Find K

void PRMS::FindKClosest
(CNodeData * node, int K, vector<CNodeData*>& close, vector<CNodeData*>& nodes)
{
    list< pair<float,int> > sorted;

    //int full=0;
    int nSize=nodes.size();
    for( int iN=0;iN<nSize;iN++ ){
        if( node==nodes[iN] ) continue;
        float dist=(node->getPos()-nodes[iN]->getPos()).normsqr();
        sorted.push_back(pair<float,int>(dist,iN));
    }
    
    sorted.sort();
    typedef list< pair<float,int> >::iterator SIT;
    int j=0;
    for(SIT i=sorted.begin();j<K;i++,j++){
        close[j]=nodes[i->second];
    }
}


void PRMS::sortedPairs
(vector<CNodeData*>& nodes, list< pair<int,int> >& close, int k)
{
    typedef pair<int,int> VPAIR;
    list< pair<float,VPAIR> > sorted;
    typedef list< pair<float,VPAIR> >::iterator SIT;

    int nSize=nodes.size();
    for( int i=0;i<nSize;i++ ){
        list< pair<float,VPAIR> > sorted_i;
        for( int j=0;j<nSize;j++ ){
            VPAIR v(i,j);
            float dist=(nodes[i]->getPos()-nodes[j]->getPos()).normsqr();
            sorted_i.push_back(pair<float,VPAIR>(dist,v));
        }
        sorted_i.sort();
        SIT s=sorted_i.begin();
        for(int ik=0;ik<k&&s!=sorted_i.end();ik++,s++) sorted.push_back(*s);
    }
    
    sorted.sort();
    {
        for(SIT i=sorted.begin();i!=sorted.end();i++){
            close.push_back(i->second);
        }
    }//end for
}

void PRMS::nearPairs
(vector<CNodeData*>& nodes, list< pair<int,int> >& close, float d)
{
	double d_sqr=d*d;
    int nSize=nodes.size();
    for( int i=0;i<nSize;i++ )
    {
        for( int j=0;j<nSize;j++ )
        {
            float dist_sqr=(nodes[i]->getPos()-nodes[j]->getPos()).normsqr();
            if(dist_sqr<d_sqr) close.push_back(make_pair(i,j));
        }//end for j
    }//end for i
}

///////////////////////////////////////////////////////////////////////////////
// Path Smoothing

//This is very slow...

void PRMS::pathSmoothing
(CRoadMap& rmap,list<Point2d>& path, PRMS::PIT s)
{
    if(s==path.end()) return;
    {PIT tmp=s; tmp++; if( tmp==path.end() ) return;}

    CRobot2D& geo=rmap.getFlock()->getGeometry();//for collision detection
    PIT ns=s; ns++;
    
    //erase from the front
    PIT start=path.end(); start--;
    const Point2d& spt=*s;
    for( PIT i=start;i!=s; i-- ){
        if( !m_CD.isCollision(geo,spt,*i) ){         
            path.erase(ns, i);
            ns=i;
            break;
        }
    }//end for

    pathSmoothing(rmap,path,ns);
}

//---------------------------------------------------------------------
//indentify good loops
void PRMS::findGoodLoops
(CRoadMap& rmap, vector<CNodeData*>& nodes, const list< pair<int,int> >& loops)
{
	cout<<"- Find Good Loops"<<flush;
	double st=getTime();
	//try all pairs, brute force
    CRobot2D& geo=rmap.getFlock()->getGeometry();
	WDG& g=rmap.getGraph();
	typedef list< pair<int,int> >::const_iterator PIT;

	//int nSize=nodes.size();
	int checkedN=0;
	int size=loops.size();
	
	for(PIT i=loops.begin();i!=loops.end();i++,checkedN++)
	{
	    //report progress
        int percent1=(checkedN*100.0/size);
        int percent2=(checkedN-1)*100.0/size;
        if( percent1%10==0 && percent1!=percent2 && checkedN!=0) cout<<"=>"<<(percent1)<<"%"<<flush;    
    
    	//check the pair
		VID id1=i->first;
		VID id2=i->second;

		if(g.IsEdge(id1,id2)) continue;

		list<Point2d> path;
		findPathV1V2(rmap,id1,id2,path);
		float length=pathLength(path);

		const Point2d& pos1=nodes[id1]->getPos();
		const Point2d& pos2=nodes[id2]->getPos();
		float d=(pos1-pos2).norm();
		if(length>d*PI){ //there is a very long route connecting pos 1 and pos 2
			//try to connect!
			if( !m_CD.isCollision(geo,pos1,pos2) ) rmap.addEdge(id1,id2);
		}//end if
	}//i
	
	double et=getTime();
	cout<<"\n- Find Good Loops takes "<<et-st<<" msec"<<endl;
}


//---------------------------------------------------------------------
//simplify map
void PRMS::simplifyMap(CRoadMap& rmap)
{
	//get all degree-2 vertices
    typedef vector<CNodeData*>::iterator NIT;
    vector<CNodeData*>& nodes=rmap.getNodes();
    int NodeSize=nodes.size();
	CRobot2D& geo=rmap.getFlock()->getGeometry();
    WDG& g=rmap.getGraph();

	vector<int> deg2;
	for(int i=0;i<NodeSize;i++){
		int nei_size=g.GetVertexOutDegree(i);
		if(nei_size==2){
			deg2.push_back(i);
			vector<VID> nei;
			g.GetSuccessors(i,nei);
			//check if they can see each other
			const Point2d& p1=rmap.getNode(nei[0])->getPos();
			const Point2d& p2=rmap.getNode(nei[1])->getPos();
			if( !m_CD.isCollision(geo,p1,p2) ){
				rmap.delNode(i);
				rmap.addEdge(nei[0],nei[1]);
			}
		}//
		else if(nei_size==0) //just delete the node
			rmap.delNode(i);
	}//
}



