#ifndef _JML_MP_H_
#define _JML_MP_H_

typedef unsigned int UINT;

///////////////////////////////////////////////////////////////////////
//shepherding stuff
#include "shepherding_base.h"
#include "shepherding_behaviors.h"

///////////////////////////////////////////////////////////////////////
//samp headers
#include "util.h"
#include "cfg.h"
#include "dm.h"
#include "lp.h"
#include "roadmap.h"
#include "path.h"
#include "query.h"
#include "state.h"
#include "tree.h"
#include "est.h"

///////////////////////////////////////////////////////////////////////
//meta
#include "meta_cfg.h"
#include "meta_graph.h"

///////////////////////////////////////////////////////////////////////
//mp
class MP
{
public:

	~MP();

    bool initialize(const string& file);
    bool initialize(istream& in);

    void createMap();
    bool queryMap();
    bool findPath();

    // ------
    //
    // Access
    //
    // ------

    const list<LP*>& getLPs() const { return m_lps; }

    DM * getDM() { return m_dm; }

    Roadmap * getRM() { return &m_map; }

    Query * getQuery() { return &m_query; }

    Path * getPath() { return &m_path; }

    const int& getDOF() const { return m_dof; }

    MPstate& getState() { return m_state; }

    FSLIST& getShepherds() { return m_shepherds; }

    FSLIST& getFlock() { return m_flock; }

    const Point2d& getGoal() const { return m_goal; }

	const Point2d& getStart() const { return m_start; }

	TreePMP *  getTree(){ return m_tree; }
	GraphPMP*  getGraph(){ return m_graph; }

	RNG * getRNG() { return m_rng; }

	void printStatistics();

private:

	void snapShepherdsAroundFlock();

	GraphPMP *  m_graph;//graph-based planner
    TreePMP *   m_tree; //tree-based planner

    list<LP*>   m_lps;
    DM *        m_dm;
    Roadmap     m_map;
    Path        m_path;
    Query       m_query;   //single or multiple query
    MPstate     m_state;   //recording device

    string m_in_mpfile;  //input motion planning filename
    string m_out_rmfile; //output roadmap file name

    RNG * m_rng;

    int m_dof; //degree of freedome
    FSLIST m_shepherds, m_flock; //list of shepherds, list of flocks
    Point2d m_start, m_goal; // 2d start and goal position
};

//singleton
MP * getMP();

#endif //_JML_MP_H_



