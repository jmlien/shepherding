#include <mp.h>


//create one of the tree-based planners
GraphPMP * GraphPMP::create(list<string>& toks)
{
    if(toks.empty()) return NULL;

    string label = toks.front();
    toks.pop_front();
    GraphPMP * tmp=NULL;

    if(label=="fuzzy")   
        tmp=new Graph_meta_fuzzy();
    else{
        cerr<<"! Error: Unknown Graph-based Planner: "<<label<<endl;
        return false;
    }

    if(!tmp->initialize(toks)){
        cout << "Could not initialize "<<label<<" planner" << endl;
        delete tmp;
        return false;
    }

    //done
    return tmp;
}


bool GraphPMP::initialize(list<string>& toks)
{
	m_sim_budget=UINT_MAX;
	for(list<string>::iterator i=toks.begin();i!=toks.end();i++) {
		const string& s=*i;
		if(s=="simsteps" || s=="SimSteps")
			m_sim_budget=atoi((++i)->c_str());
	}
	return true;
}

bool Graph_meta_fuzzy::initialize(list<string>& toks)
{	
	if(!GraphPMP::initialize(toks)) return false;

	m_n=100;
	m_l=20;
    m_k=20;
	delete m_tree; 
	m_tree=NULL;
	
    for(list<string>::iterator i=toks.begin();i!=toks.end();i++){
        const string& s=*i;
        if(s=="k" || s=="K")
            m_k=atoi((++i)->c_str());
		else if(s=="n" || s=="N")
            m_n=atoi((++i)->c_str());
		else if(s=="l" || s=="L")
            m_l=atoi((++i)->c_str());
		else if(s=="tree" || s=="Tree" || s=="TREE"){
			list<string> tmp(++i,toks.end());
			m_tree=TreePMP::create(tmp);
			break;
		}
    }

	if(m_tree==NULL) {
		cerr<<"! Error: Graph_meta_fuzzy::initialize: No tree defined"<<endl;
		return false;
	}
	
	m_tree->please_be_quiet(); //make tree stop talking

	return true;
}

bool Graph_meta_fuzzy::findPath(Path& path)
{
	sample();
	connect();
	return query(path);
}

//
//
// Sample
//
//
void Graph_meta_fuzzy::sample()
{
	MetaCfg metacfg;
	RNG * rng = getMP()->getRNG();

    //bounding box
    static const float * bbox=getEnvironment()->getBBX().getBBXValue();
    static float bbox_x=bbox[1]-bbox[0];
    static float bbox_z=bbox[5]-bbox[4];

	//compute the smallest circle
	float smallest_r=smallestCircleRadius()*1.5;
	float largest_r=smallest_r*2;
	
	//add n more nodes
	CFlockState * shepherd=getMP()->getShepherds().front();
	for(int i=0;i<m_n;i++){
		//create a random radius
		metacfg.m_flock_rad=(1+rng->uniform())*smallest_r;

		//create a random position
		do{
            shepherd->setPos(Point2d(rng->uniform()*bbox_x+bbox[0],rng->uniform()*bbox_z+bbox[4]));
        }
		while(isCollision(*getEnvironment(),*shepherd));
		//check the position
		Point2d closest_pt;
		float clearance=getClearance(*getEnvironment(), shepherd->getType()->getGeometry(), shepherd->getPos(), closest_pt);
		if(clearance<metacfg.m_flock_rad){
			i--;
			continue; //disc is too big 
		}
		else metacfg.m_flock_cen=shepherd->getPos();
		//create a random direction
		float rad=rng->uniform()*PI2;
		metacfg.m_flock_dir.set(cos(rad),sin(rad));
		//add to map
		meta_map.addnode(metacfg);
	}
}


//this approximate the radius of the smallest circle
float Graph_meta_fuzzy::smallestCircleRadius()
{
	int gsize=getMP()->getFlock().size();
	float r_f=getMP()->getFlock().front()->getType()->getGeometry().getRadius();
	return r_f*sqrt(gsize*2.0f);
}

//
//
// Connect
//
//
void Graph_meta_fuzzy::connect()
{
    MetaGraph& G=meta_map.getGraph();
    int total=G.GetVertexCount();
	vector<VID> nodes;
    nodes.reserve(total);
    G.GetVerticesVID(nodes);

	cout<<"- Connecting to the "<<m_k<<" closest cfgs"<<endl;
    for(int i=0;i<total;i++){
        vector<VID> knodes;
        knodes.reserve(m_k);
        k_closest(nodes[i],nodes,knodes);
		MetaCfg& meta_source=meta_map.getData(nodes[i]);
		for(int k=0;k<knodes.size();k++){
			MetaCfg& meta_goal=meta_map.getData(knodes[k]);

			//check straightline collision
			//check if the line between cfg and meta_goal collides with obstacle
			bool cd=mychecklinecollision(meta_goal.m_flock_cen, meta_source.m_flock_cen,*getMP()->getFlock().front());
			if(cd) continue;
			
			MetaRoadmap_Edge edge(meta_source.distance(meta_goal));
			G.AddEdge(nodes[i],knodes[k],edge);
		}
	}//end for i

	conncetStart2Graph();
	connectGoal2Graph();
	pruneGraph();
}
	
bool Graph_meta_fuzzy::localPlanner(const Cfg& cfg, MetaCfg& meta_goal)
{
	//check using simulation
	toFlockState(cfg);

	//clean the roadmap
	getMP()->getRM()->clear();

	//add cfg to the roadmap
	getMP()->getRM()->addnode(cfg);

	//setup the tree
	m_tree->setGoal(meta_goal.m_flock_cen,meta_goal.m_flock_rad);

	//search for a path to the position
	Path tmp;
	return m_tree->findPath(tmp); //) return false;
}

/*
void Graph_meta_fuzzy::createComformingFlock(Cfg& cfg, MetaCfg& meta_source)
{
	FSLIST& flock=getMP()->getFlock();
	RNG * rng = getMP()->getRNG();

    for(FSLIST::iterator iI=flock.begin();iI!=flock.end();iI++ ){
        CFlockState& s=**iI; //a flock member
        Point2d pos;
        do{
			float angle=rng->uniform()*PI2;
			Vector2d tmp(cos(angle),sin(angle));
			float dist=meta_source.m_flock_rad*rng->uniform();
			//cout<<"dist="<<dist<<" R="<<meta_source.m_flock_rad<<endl;
			tmp=tmp*dist;
			pos=meta_source.m_flock_cen+tmp;
        }while( isCollision(*getEnvironment(),s.getType()->getGeometry(),pos) );
        s.setPos(pos);
        s.setVelocity(meta_source.m_flock_dir*2); //how should we set the velocity?
    }//end for

	fromFlockState(cfg);

	//cout<<" R="<<cfg.m_flock_rad<<" r="<<meta_source.m_flock_rad<<" C="<<cfg.m_flock_cen<<" c="<<meta_source.m_flock_cen<<endl;
}

void Graph_meta_fuzzy::arrangeShepherd(Cfg& cfg)
{
	FSLIST& shepherds=getMP()->getShepherds();
	RNG * rng = getMP()->getRNG();
    int size=shepherds.size();

    Vector2d f_v = -cfg.m_flock_dir*cfg.m_flock_rad;
    if(size==1){
    	shepherds.front()->setPos(cfg.m_flock_cen+f_v);
    }
    else{
    	float angle=0.1*PI;
        float theta = ((PI-2*angle)/(size-1));
        int j=0;
        for(FSLIST::iterator i=shepherds.begin();i!=shepherds.end();i++,j++){
        	float phi = theta*j + angle;
            float cos_phi=cos(phi);
            float sin_phi=sin(phi);
            float x2 = cos_phi*f_v[0] - sin_phi*f_v[1] + cfg.m_flock_cen[0];
            float y2 = sin_phi*f_v[0] + cos_phi*f_v[1] + cfg.m_flock_cen[1];
            (*i)->setPos(Point2d(x2,y2));
        }//end for
    }
	fromFlockState(cfg);
}
*/

void Graph_meta_fuzzy::k_closest
(const VID& n, const vector<VID>& nodes, vector<VID>& knodes)
{
    MetaGraph& G=meta_map.getGraph();

    vector< pair<float,VID> > distdata;
    distdata.reserve(m_k);

    //compute distances
    MetaCfg& c1=meta_map.getData( G.GetData(n) );
    for(vector<VID>::const_iterator i=nodes.begin();i!=nodes.end();i++)
    {
        if(n==*i) continue; //same node
        MetaCfg& c2=meta_map.getData( G.GetData(*i) );
        pair<float,VID> tmp;
        tmp.first=c1.distance(c2);
        tmp.second=*i;

		if(tmp.first>=2) continue; //too far

        //distdata is full and its largest value is smaller than dist
        if( distdata.size()==m_k ){
            if( distdata.front().first<=tmp.first ){ //too big
                continue; //do nothing
            }
            else{ //pop the largest...
                pop_heap(distdata.begin(),distdata.end()); 
                distdata.pop_back();
            }
        }
        //add to distdata
        distdata.push_back(tmp);
        push_heap(distdata.begin(),distdata.end());
    }

    //get first k
    int size=distdata.size();
    for(short i=0;i<size;i++){
        knodes.push_back(distdata[i].second);
    }
}

void Graph_meta_fuzzy::conncetStart2Graph()
{
	const Cfg& start=getMP()->getQuery()->getQuery(0);	
	MetaCfg meta_start;
	meta_start.m_flock_cen=start.m_flock_cen;
	meta_start.m_flock_rad=start.m_flock_rad;
	m_sid=meta_map.addnode(meta_start);
	//
	MetaGraph& G=meta_map.getGraph();
    int total=G.GetVertexCount();
	vector<VID> nodes;
    nodes.reserve(total);
    G.GetVerticesVID(nodes);

	cout<<"- Connecting start meta cfg"<<endl;
    for(int i=0;i<total;i++){
		if(nodes[i]==m_sid) continue;
		MetaCfg& meta_i=meta_map.getData(nodes[i]);
		meta_start.m_flock_dir=meta_i.m_flock_dir;///maybe we should use flock's avg direction
		float dist=meta_start.distance(meta_i);
		if(dist>=2) continue; //too far
		bool cd=mychecklinecollision(meta_i.m_flock_cen, meta_start.m_flock_cen,*getMP()->getFlock().front());
		if(!cd)
			G.AddEdge(m_sid,nodes[i],MetaRoadmap_Edge(dist));
	}//end for i
}

void Graph_meta_fuzzy::connectGoal2Graph()
{
	//
	//compute the smallest circle
	float smallest_r=smallestCircleRadius();
	float largest_r=smallest_r*3;
	//
	MetaCfg meta_goal;
	meta_goal.m_flock_cen=getMP()->getGoal();
	meta_goal.m_flock_rad=largest_r;
	m_gid=meta_map.addnode(meta_goal);
	//
	MetaGraph& G=meta_map.getGraph();
    int total=G.GetVertexCount();
	vector<VID> nodes;
    nodes.reserve(total);
    G.GetVerticesVID(nodes);

	cout<<"- Connecting goal meta cfg"<<endl;
    for(int i=0;i<total;i++){
		if(nodes[i]==m_gid) continue;
		MetaCfg& meta_i=meta_map.getData(nodes[i]);
		meta_goal.m_flock_rad=meta_i.m_flock_rad;
		meta_goal.m_flock_dir=(meta_goal.m_flock_cen-meta_i.m_flock_cen).normalize();///maybe we should use flock's avg direction
		float dist=meta_i.distance(meta_goal);
		if(dist>=2) continue; //too far
		bool cd=mychecklinecollision(meta_i.m_flock_cen, meta_goal.m_flock_cen,*getMP()->getFlock().front());
		if(!cd) G.AddEdge(nodes[i],m_gid,MetaRoadmap_Edge(dist));
	}//end for i
}

//cut off those that cannot be connected from start
void Graph_meta_fuzzy::pruneGraph()
{
	cout<<"- Prune graph"<<endl;
	MetaGraph& G=meta_map.getGraph();
	G.GetReferenceofData(m_sid)->m_visited=true;
	G.GetReferenceofData(m_gid)->m_visited=true; //prevent from being deleted
	exploreGraph(m_sid);
	deleteUnvisitedNodes();
	
	//do it again from the goal
	G.SetPredecessors(); 
	G.GetReferenceofData(m_sid)->m_visited=true; //prevent from being deleted
	G.GetReferenceofData(m_gid)->m_visited=true;
	exploreGraphreverse(m_gid);
	deleteUnvisitedNodes();
}

void Graph_meta_fuzzy::exploreGraph(VID id)
{
	MetaGraph& G=meta_map.getGraph();
	vector<VID> succ;
	int size=G.GetSuccessors(id,succ);
	for(int i=0;i<size;i++){
		MetaRoadmap_Node * ptr=G.GetReferenceofData(succ[i]);
		if(!ptr->m_visited){
			ptr->m_visited=true;
			exploreGraph(succ[i]);
		}
	}//end for
}


void Graph_meta_fuzzy::exploreGraphreverse(VID id)
{
	MetaGraph& G=meta_map.getGraph();
	vector<VID> pred;
	int size=G.GetPredecessors(id,pred);
	for(int i=0;i<size;i++){
		MetaRoadmap_Node * ptr=G.GetReferenceofData(pred[i]);
		if(!ptr->m_visited){
			ptr->m_visited=true;
			exploreGraphreverse(pred[i]);
		}
	}//end for
}


void Graph_meta_fuzzy::deleteUnvisitedNodes()
{
	MetaGraph& G=meta_map.getGraph();
	MetaRoadmap new_map;
	//delete
    int total=G.GetVertexCount();
	vector<VID> nodes, mapping;
    nodes.reserve(total);
    G.GetVerticesVID(nodes);
	mapping=nodes;
	
	//add nodes
	for(int i=0;i<total;i++){
		MetaRoadmap_Node * n=G.GetReferenceofData(nodes[i]);
		if(n->m_visited){
			MetaCfg& cfg=meta_map.getData(nodes[i]);
			mapping[i]=new_map.addnode(cfg);
		}
		//if(!G.GetData(nodes[i]).m_visited) G.DeleteVertex(nodes[i]);
		//else ->m_visited=false; 
	}
	
	//add edges
	vector< pair< pair<VID,VID>, MetaRoadmap_Edge> > meta_edges;
	int esize=meta_map.getGraph().GetEdges(meta_edges);
	for(int i=0;i<esize;i++){
		VID v1=meta_edges[i].first.first;
		MetaRoadmap_Node * n1=G.GetReferenceofData(v1);
		if(!n1->m_visited) continue;
		VID v2=meta_edges[i].first.second;
		MetaRoadmap_Node * n2=G.GetReferenceofData(v2);
		if(!n2->m_visited) continue;
		new_map.addedge(mapping[v1],mapping[v2],meta_edges[i].second.Weight());
	}

	m_sid=mapping[m_sid];
	m_gid=mapping[m_gid];
	meta_map=new_map;
}

//
//
// Query
//
//
bool Graph_meta_fuzzy::query(Path& path)
{
	cout<<"- Query"<<endl;
	MetaGraph& G=meta_map.getGraph();
	MetaCfg& s=meta_map.getData(m_sid);

	while(true){

		//get a path from sid to gid
		vector< pair<MetaRoadmap_Node,MetaRoadmap_Edge> > inter_path;
		MetaRoadmap_Node snode=G.GetData(m_sid);
		MetaRoadmap_Node gnode=G.GetData(m_gid);
    	int size=FindPathDijkstra(G,snode,gnode,inter_path);
		if(size==0){
			break;
		}

		//evaluate the edges along the path one by one
		bool found_invalid_edge=false;
		for(int i=0;i<size-1;i++){
			//for each edge
			MetaRoadmap_Edge& e=inter_path[i].second;
			if(e.m_visited) continue;
			//simulate
			const Cfg& from=(i==0)?getMP()->getQuery()->getQuery(0):inter_path[i-1].second.m_local_map.getVector().back();
			MetaCfg& meta_goal=meta_map.getData(inter_path[i+1].first);
			bool found=localPlanner(from,meta_goal);
			VID v1=inter_path[i].first.m_id;
			VID v2=inter_path[i+1].first.m_id;
			G.DeleteEdge(v1,v2); //delete the edge
			if(found){
				e.m_visited=true;
				e.m_local_map=*getMP()->getRM(); //remember how the roadmap looks like
				G.AddEdge(v1,v2,e);
			}
			else{
				found_invalid_edge=true;
				break;
			}
		}//end for i
		
		cout<<"."<<flush;

		if(!found_invalid_edge){
			cout<<"\nYeah! Path found"<<endl;
			setup_roadmap_so_it_has_the_successful_path_to_the_goal(inter_path);
			return true; //found!!
		}
		
		//check if we run out of budget
		unsigned int sim_time=getMP()->getLPs().front()->getTotalSimulateStepCount();
		if(sim_time>=m_sim_budget){ //running out of time
			cout << endl << "- Time Budget ("<<sim_time<<"/"<<m_sim_budget<<") Exceeded!" << endl;
			break;
		}

	}//end while

	setup_roadmap_so_it_has_all_explored_paths(); //tmp
	cout<<"\nNo.... Path NOT found"<<endl;
	return false;
}

void Graph_meta_fuzzy::setup_roadmap_so_it_has_the_successful_path_to_the_goal
(vector< pair<MetaRoadmap_Node,MetaRoadmap_Edge> >& inter_path)
{
	int size=inter_path.size();
	Roadmap& joint_rm=*getMP()->getRM();
	joint_rm.clear();
	int last_vid=joint_rm.addnode(getMP()->getQuery()->getQuery(0));
	int vid=0;

	for(int i=0;i<size-1;i++){
		//for each edge
		MetaRoadmap_Edge& e=inter_path[i].second;
		
		//add the local roadmap to the joint roadmap
		int local_map_size=e.m_local_map.getVector().size();
		//add vertices first
		for(int j=1;j<local_map_size;j++)
			vid=joint_rm.addnode(e.m_local_map.getData(j));
		//add edges
		vector< pair< pair<VID,VID>, CEdge> > local_edges;
		int esize=e.m_local_map.getGraph().GetEdges(local_edges);
		for(int j=0;j<esize;j++){
			joint_rm.addedge(local_edges[j].first.first+last_vid, 
			                 local_edges[j].first.second+last_vid, 
							 local_edges[j].second.Weight());
		}
		last_vid=vid;
	} //end for i
}


//
//
// convert the explored  region to a roadmap
// this is for debug 
// not used in solving the problem
//
void Graph_meta_fuzzy::setup_roadmap_so_it_has_all_explored_paths()
{
	Roadmap& joint_rm=*getMP()->getRM();
	joint_rm.clear();
	
	vector< pair< pair<VID,VID>, MetaRoadmap_Edge> > meta_edges;
	int esize=meta_map.getGraph().GetEdges(meta_edges);
	int last_vid=0;
	int vid=0;

	for(int i=0;i<esize;i++){
		//for each edge
		MetaRoadmap_Edge& e=meta_edges[i].second;
		if(!e.m_visited) continue; //not visited

		//add the local roadmap to the joint roadmap
		int local_map_size=e.m_local_map.getVector().size();

		//add vertices first
		for(int j=0;j<local_map_size;j++){
			vid=joint_rm.addnode(e.m_local_map.getData(j));
		}
		//add edges
		vector< pair< pair<VID,VID>, CEdge> > local_edges;
		int esize=e.m_local_map.getGraph().GetEdges(local_edges);
		for(int j=0;j<esize;j++){
			joint_rm.addedge(local_edges[j].first.first+last_vid, 
			                 local_edges[j].first.second+last_vid, 
							 local_edges[j].second.Weight());
		}
		last_vid=vid+1;
	} //end for i

	if(joint_rm.getVector().empty())
		joint_rm.addnode(getMP()->getQuery()->getQuery(0));
}



