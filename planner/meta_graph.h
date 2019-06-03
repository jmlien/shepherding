#ifndef _PMP_PRM_H_
#define _PMP_PRM_H_

#include <list>
#include <string>
#include <float.h>
using namespace std;

class Path;
class TreePMP;

class GraphPMP //graph based probabilitic motion planning
{
public:
    //create one of the tree-based planners
    static GraphPMP * create(list<string>& toks);

    virtual bool initialize(list<string>& toks);
    virtual bool findPath(Path& path)=0;

protected:
	unsigned int m_sim_budget;       // max number of sim steps allowed
};

class Graph_meta_fuzzy: public GraphPMP
{
public:

	Graph_meta_fuzzy(){ m_tree=NULL; }
	virtual bool initialize(list<string>& toks);
    virtual bool findPath(Path& path);
	MetaRoadmap& getRoadmap(){ return meta_map; }

protected:
	
	void sample();
	void connect();
	bool query(Path& path);
	 
	//help functions for sampling
	float smallestCircleRadius();
	
	//help functions for connection
	bool localPlanner(const Cfg& cfg, MetaCfg& meta_goal);
	void createComformingFlock(Cfg& cfg, MetaCfg& meta_source);
	void arrangeShepherd(Cfg& cfg);
	void k_closest(const VID& n, const vector<VID>& nodes, vector<VID>& knodes);
	void conncetStart2Graph();
	void connectGoal2Graph();
	void pruneGraph();
	void exploreGraph(VID id);
	void exploreGraphreverse(VID id);
	void deleteUnvisitedNodes();

	//help functions for query
	void setup_roadmap_so_it_has_the_successful_path_to_the_goal
	(vector< pair<MetaRoadmap_Node,MetaRoadmap_Edge> >& inter_path);
	//for debugging only
	void setup_roadmap_so_it_has_all_explored_paths();

private:

	int m_n; //number of "meta" configurations
	int m_k; //connection between k closest configurations
	int m_l; //local planning trials

	TreePMP * m_tree;

	VID m_sid, m_gid; //meta start and goal id in the meta graph

	MetaRoadmap meta_map;

	struct query_results
	{
		bool found;
		pair<VID,VID> failed_edge;
	};
};

#endif //_PMP_PRM_H_


