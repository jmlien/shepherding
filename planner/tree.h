#ifndef _PMP_TREE_H_
#define _PMP_TREE_H_

#include <list>
#include <string>
#include <limits>
#include <float.h>
using namespace std;

#include "cfg.h"
#include "multi_herding_rules.h"

class Path;

class TreePMP: public sh_stoppable
{
public:
	TreePMP(){m_quiet=false;}
	virtual ~TreePMP();

	//create one of the tree-based planners
	static TreePMP * create(list<string>& toks);

	virtual bool initialize(list<string>& toks);
	virtual bool findPath(Path& path)=0;
	virtual bool isGoal();
	void setGoal(const Point2d& pos, float radius){ m_goal_pos=pos; m_goal_rad=radius; }
	void please_be_quiet(){ m_quiet=true; }
	void printStatistics();


protected:

	Point2d m_goal_pos; // goal position; initial value is the global goal of the shepherd
	float   m_goal_rad; // goal radius; initial value is the view radius of the flock
	bool    m_quiet;    // make the tree quiet...
	unsigned int
	m_sim_budget;       // max number of sim steps allowed
};

//
// Modified from Chris Vo, 2008 Fall
// RRT
//

//RRT: random shepherd position
class RRT: public TreePMP
{
public:
	virtual bool initialize(list<string>& toks);
	virtual bool findPath(Path& path);

protected:

	virtual bool expanding()=0;
	virtual void randomize(Cfg& c)=0;

	int k_size; // the number of iterations
	float goal_bias; //0~1, default 0.5
};

//--------------
//
// no behavior (shepherd targets from milestone)
// &
// random targets
//
//--------------

class nb_rt_RRT: public RRT
{
public:

	virtual bool findPath(Path& path);
	virtual bool stop();

protected:

	//virtual bool initialize(list<string>& toks);
	virtual bool expanding();
	virtual void randomize(Cfg& c);
	virtual VID nearest(Cfg& q);
};

//--------------
//
// no behavior (shepherd targets from milestone)
// &
// random milestone (of the flock) biased toward a WS roadmap node
//
//--------------

class nb_rm_RRT: public nb_rt_RRT
{
public:
	virtual bool findPath(Path& path);
	virtual bool stop();
protected:
	virtual void randomize(Cfg& c);
};

//--------------
//
// deterministic behavior
// &
// random milestone (of the flock) biased toward a WS roadmap node
//
// implemetation: tree_db_rm_RRT.cpp
//
//--------------


class db_rm_RRT: public nb_rt_RRT
{
public:
	virtual bool findPath(Path& path);
	virtual bool stop();

protected:

	virtual void randomize(Cfg& c);
};

//--------------
//
// random behavior (parameters)
// &
// random milestone (of the flock) biased toward a WS roadmap node
//
//--------------

class rb_rm_RRT: public nb_rt_RRT
{
public:
	virtual bool findPath(Path& path);

protected:

	virtual void randomize(Cfg& c);

private:

	//deterministic shepherd behavior
	struct r_sh_behavior: public CBehaviorRule
	{
		virtual void applyRule(CFlockState& s)
		{
		}
	};
};

//--------------
//
// simulation-only
//
//--------------

class Simulation_Only: public TreePMP, public sh_statfunc
{
public:
	virtual bool findPath(Path& path);
	virtual bool stop();
	virtual void stat();
};


#endif //_PMP_TREE_H_
