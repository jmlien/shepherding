#ifndef EST_H_
#define EST_H_

/*
 * est.h
 *
 * Expansive-Spaces Trees Algorithm (Header)
 *
 * Based on the algorithm described in Choset [et al.]. Principles
 * of Robot Motion: Theory, Algorithms, and Implementation. 2005,
 * MIT Press. Pages 230--233.
 *
 * Created on: Jan 31, 2009
 * Author: Christopher Vo (cvo1@cs.gmu.edu)
 */

#include "mp.h"
#include "est_nhood.h"

/*------------------------------------------------------
 * EST
 *------------------------------------------------------*/

class EST: public TreePMP
{
public:
	virtual ~EST();
	virtual bool findPath(Path& path);
	virtual bool initialize(list<string>& toks);
	virtual bool stop();
protected:
	virtual VID pickRandomNode() = 0;
	virtual bool expandFrom(VID p_id);
	virtual UINT simulate(Cfg &p, Cfg& q);
	//virtual bool isGoal();

	int roadmap_size;
	int num_expansions;
	double nhood_size;
	NHood * neighborhood;
};

/*
 * The BasicEST uses a completely random method to choose the next
 * node to expand in the tree. This may lead to dense oversampling in
 * some areas of the space.
 */
class BasicEST: public EST
{
protected:
	virtual VID pickRandomNode();
};

/*
 * The NaiveEST algorithm attempts to limit the oversampling
 * by making it more likely to select nodes that have fewer
 * neighbors.
 */
class NaiveEST: public EST
{
public:
	virtual ~NaiveEST();
	virtual bool initialize(list<string>& toks);
protected:
	virtual bool expandFrom(VID p_id);
	virtual VID pickRandomNode();
	UINT * num_neighbors;
	UINT * prob_dist;
};

/*
 * The MinEST algorithm always deterministically picks the node
 * that has the fewest neighbors.
 */
class MinEST: public EST
{
public:
	virtual ~MinEST();
	virtual bool initialize(list<string>& toks);
protected:
	virtual bool expandFrom(VID p_id);
	virtual VID pickRandomNode();
	UINT * num_neighbors;
	int min_idx;
};

/*
 * HeuristicEST uses a variety of heuristics in picking a node
 */

class HeuristicEST: public EST
{
public:
	virtual ~HeuristicEST();
	virtual bool initialize(list<string>& toks);
	virtual bool findPath(Path& path);
protected:
	virtual bool expandFrom(VID p_id);
	virtual VID pickRandomNode();
	UINT * num_neighbors;
	UINT * out_degree;
	UINT * wavefront;
	double * dist_to_goal;
	int selected_index;
};

#endif /* EST_H_ */
