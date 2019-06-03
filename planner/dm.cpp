
#include "mp.h"

//create DM instance

DM * DM::create(list<string>& toks)
{
    if(toks.empty()) return NULL;
    string label = toks.front();
    toks.pop_front();

    DM * tmp=NULL;
    if(label=="Shepherd" || label=="shepherd") //Shepherd Only
        tmp=new DM_ShepherdOnly();
    else if(label=="Flock" || label=="flock") //Flock Only
        tmp=new DM_FlockOnly();
	else if(label=="FlockGeo" || label == "flockgeo" )
		tmp=new DM_FlockGeo();
    else{	
        cerr<<"! Error: Unknow Distance Metrics: "<<label<<endl;
        return false;
    }

    if(!tmp->initialize(toks)){
        delete tmp;
        return false;
    }

    //done
    return tmp;
}

//--------------
//
// shepherd only
//
//--------------

float DM_ShepherdOnly::distsqr(const Cfg& c1, const Cfg& c2)
{
    //compute shepherd distance (need a better way)
    typedef vector<Point2d>::const_iterator IT;
    IT i1=c1.m_shepherd_pos.begin();
    IT i2=c2.m_shepherd_pos.begin();
    
    float d=0;
    for(;i1!=c1.m_shepherd_pos.end();i1++,i2++)
        d+=((*i1)-(*i2)).norm();

    return d;
}

//--------------
//
// flock only
//
//--------------

float DM_FlockOnly::distsqr(const Cfg& c1, const Cfg& c2)
{
    //generate a set of target positions at random
    static const float * bbox=getEnvironment()->getBBX().getBBXValue();
    static float bbox_x=bbox[1]-bbox[0];
    static float bbox_z=bbox[5]-bbox[4];
    static float bbox_dia=sqrt(bbox_x*bbox_x+bbox_z*bbox_z);

    //which is which? 
    //m_shepherd_pos will be filled before adding to the tree
    const Cfg & tree_node=(c1.m_in_map)?c1:c2;
    const Cfg & rnd_node=(tree_node==c1)?c2:c1;

    //euclidean distance
    Vector2d vec=rnd_node.m_flock_tar-tree_node.m_flock_cen;
    float ed=vec.norm(); //normalized

    //angular distance
    float ad=1;
    if(tree_node.m_flock_dir.normsqr()!=0){
        ad=acos(vec*tree_node.m_flock_dir/ed)/PI;
    }
    
    return ed/bbox_dia; //+ad;
}

//--------------
//
// flock only
//
//--------------

float DM_FlockGeo::distsqr(const Cfg& c1, const Cfg& c2)
{
    //start to end distance
	static float MAX_GEO_DIST=-1;
	if(MAX_GEO_DIST<0) MAX_GEO_DIST=S2G_dist();

    //which is which? 
    //m_shepherd_pos will be filled before adding to the tree
    const Cfg & tree_node=(c1.m_in_map)?c1:c2;
    const Cfg & rnd_node=(tree_node==c1)?c2:c1;

    //euclidean distance
    Vector2d vec=rnd_node.m_flock_tar-tree_node.m_flock_cen;
    float ed=vec.norm(); //normalized
    float gd=pathLength(c1,c2)/MAX_GEO_DIST;

    //angular distance
    float ad=1;
    if(tree_node.m_flock_dir.normsqr()!=0){
        ad=acos(vec*tree_node.m_flock_dir/ed)/PI;
    }
    
    return gd; //+ad;
}


float DM_FlockGeo::S2G_dist()
{
	//get graph
	list<Point2d> path;
	CMapFlockState * shepherd=(CMapFlockState *)(getMP()->getShepherds().front());
	PRMS * prm=((CHerdingFlock*)shepherd->getType())->prm;
	prm->findPath(*shepherd->getMap(),getMP()->getStart(),getMP()->getGoal(),path); 
	if(path.empty()){
		cerr<<"! Error: DM_FlockGeo::S2G_dist cannot connect start to goal"<<endl;
		exit(1);
	}

	//find length
	return ::pathLength(path);
}

float DM_FlockGeo::pathLength(const Cfg& c1, const Cfg& c2)
{
	//get graph
	CMapFlockState * shepherd=(CMapFlockState *)getMP()->getShepherds().front();
	WDG& graph=shepherd->getMap()->getGraph();
	const VID& v1=c1.m_cls_vid;
	const VID& v2=c2.m_cls_vid;

//cout<<"v1="<<v1<<" v2="<<v2<<endl;

	//
	float c1v1=(c1.m_flock_cen-shepherd->getMap()->getNode(v1)->getPos()).norm();
	float c2v2=(c2.m_flock_cen-shepherd->getMap()->getNode(v2)->getPos()).norm();

    if( v1==v2 ) return c1v1+c2v2;

	//
    vector< pair<CNode,CEdge> > nodes;
    typedef vector< pair<CNode,CEdge> >::iterator NIT;
    int size=FindPathDijkstra<WDG>(graph,v1,v2,nodes);

	float length=0;
    for(NIT i=nodes.begin();i!=nodes.end();i++ ){
		length+=i->second.Weight();
    }//end for

	//done
    return length+c1v1+c2v2;	
}


