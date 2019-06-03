#ifndef _JML_ROADMAP_H_
#define _JML_ROADMAP_H_

//--------------
//
// Roadmap
//
//--------------

#include "mp.h"

class Cfg; //defined in cfg.h
class MetaCfg; //defined in meta_cfg.h

//-----------------------------
//
// edge/node
//
//-----------------------------

struct Roadmap_Node
{
    Roadmap_Node(){ m_id=-1; }
    static Roadmap_Node InvalidData();
    bool operator==(const Roadmap_Node& n) const;
    int m_id;
};

inline ostream& operator<<(ostream& out,const Roadmap_Node& n)
{
    out<<n.m_id<<" ";
    return out;
}



//-----------------------------
//
// Define graph
//
//-----------------------------
typedef UG<Roadmap_Node,CEdge>  MPUG;
typedef NMG<Roadmap_Node,CEdge> MPNMG;
typedef WG<Roadmap_Node,CEdge>  MPWG;
typedef Graph<MPUG,MPNMG,MPWG,Roadmap_Node,CEdge> MPGraph;

//-----------------------------
//
// Roadmap
//
//-----------------------------

template<class CFGTYPE, class GRAPHTYPE>
class GenericRoadmap
{
public:
	
	typedef typename GRAPHTYPE::VERTEX_TYPE VERTEX_TYPE;

    VID addnode(const CFGTYPE& cfg)
	{
        m_data.push_back(cfg);
        VERTEX_TYPE node;
        node.m_id = m_data.size() - 1;
        return m_graph.AddVertex(node);
	}	

    bool addedge(VID v1, VID v2, float w)
	{
		return m_graph.AddEdge(v1, v2, w) == OK;
	}

	//make the roadmap empty
	void clear()
	{
		m_data.clear();
		m_graph=GRAPHTYPE();
	}

    const CFGTYPE& getData(const VERTEX_TYPE& node) const {return m_data[node.m_id];}
	const CFGTYPE& getData(VID t) const {return m_data[m_graph.GetData(t).m_id];}
    CFGTYPE& getData(const VERTEX_TYPE& node){return m_data[node.m_id];}
    CFGTYPE& getData(VID t){return m_data[m_graph.GetData(t).m_id];}

    const GRAPHTYPE& getGraph() const { return m_graph; }
    GRAPHTYPE& getGraph() { return m_graph; }

    vector<CFGTYPE>& getVector() { return m_data; }

private:

    GRAPHTYPE m_graph;

    //may want to store cfg data here instead of in the graph...
    vector<CFGTYPE> m_data;
};


//unweighted roadmap
typedef GenericRoadmap<Cfg,MPGraph> Roadmap;


//node used in metagraph
struct MetaRoadmap_Node
{
    MetaRoadmap_Node(){ m_id=-1; m_visited=false; }
	static MetaRoadmap_Node InvalidData();
    bool operator==(const MetaRoadmap_Node& n) const;
    int m_id;
	bool m_visited;
};


inline ostream& operator<<(ostream& out,const MetaRoadmap_Node& n)
{
    out<<n.m_id<<" ";
    return out;
}



//edge used in metagraph
struct MetaRoadmap_Edge: public CEdge
{
    MetaRoadmap_Edge():CEdge(){}
	MetaRoadmap_Edge(float w):CEdge(w){m_visited=false;}
	//this are used during the query
	bool m_visited;
	Roadmap m_local_map; //this is valid only when m_visited is true
};


//meta roadmap
typedef DG<MetaRoadmap_Node,MetaRoadmap_Edge>  MPDG;
typedef NMG<MetaRoadmap_Node,MetaRoadmap_Edge> MPNMG2;
typedef WG<MetaRoadmap_Node,MetaRoadmap_Edge>  MPWG2;
typedef Graph<MPDG,MPNMG2,MPWG2,MetaRoadmap_Node,MetaRoadmap_Edge> MetaGraph;
typedef GenericRoadmap<MetaCfg,MetaGraph> MetaRoadmap;

#endif//_JML_ROADMAP_H_


