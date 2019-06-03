
#include "mp.h"

Query::Query()
{
    m_k=20;
}

//initialize itself
bool Query::initialize(list<string>& toks)
{
    return true;
}

bool Query::findPath(Path& path)
{
    Roadmap * rm = getMP()->getRM();
    MPGraph& graph = rm->getGraph();
	if(rm->getVector().empty()) return false;

    VID s=0, g=0;
	//find g, the closest node to the goal
	{
	/*
		float min = FLT_MAX;
		DM * dm = getMP()->getDM();

		//get all nodes in the roadmap
		vector<VID> all_nodes;
		graph.GetVerticesVID(all_nodes);

		// compute distances, choose max
		CFlockState * shepherd=getMP()->getShepherds().front();
		DM_FlockGeo mydm;
		Cfg tmpcfg;
		tmpcfg.m_flock_tar=getMP()->getGoal(); 
		tmpcfg.m_cls_vid=((CMapFlockState*)shepherd)->getMap()->closestNode(getMP()->getGoal()); 

		for(vector<VID>::iterator i=all_nodes.begin();i!=all_nodes.end();i++) {
			Cfg& c = rm->getData(graph.GetData(*i)); //getCfgFromVID(*i);
			//float tmp =  mydm.distsqr(c,tmpcfg);//
			float tmp=(c.m_flock_cen-getMP()->getGoal()).normsqr();
			if(tmp < min) {
				g = *i;
				min = tmp;
			}
		}
	*/
		g=rm->getVector().size()-1;
	}
    //VID g=graph.GetVertexCount()-1;

    vector< pair<Roadmap_Node,CEdge> > inter_path;
	Roadmap_Node snode=graph.GetData(s);
	Roadmap_Node gnode=graph.GetData(g);

	if(gnode==snode){
		path.getPath().push_back(rm->getData(0));
	}
	else{
		int size=FindPathDijkstra(graph,snode,gnode,inter_path);
    	vector<Cfg>& cfgs=path.getPath();
	    for(int i=0;i<size;i++){
    	    Cfg& c=rm->getData(inter_path[i].first);
        	cfgs.push_back(c);
    	}
	}

    return true;
}



