/*
 * est.cpp
 *
 * Expansive-Spaces Trees Algorithm
 *
 * Based on the algorithm described in Choset [et al.]. Principles
 * of Robot Motion: Theory, Algorithms, and Implementation. 2005,
 * MIT Press. Pages 230--233.
 *
 * Created on: Jan 31, 2009
 * Author: Christopher Vo (cvo1@cs.gmu.edu)
 */

#include "mp.h"

EST::~EST()
{
	if (neighborhood)
		delete neighborhood;
}

bool EST::initialize(list<string>& toks)
{
     if( !TreePMP::initialize(toks) ) return false;

	/*
	 * EST requires the following parameters:
	 * n: the number of attempts to expand the EST tree.
	 * nhood_size: the standard deviation of the EST neighborhood search.
	 */
	if (toks.empty()) {
		cerr << "EST requires parameters: n, nhood_size" << endl;
		return false;
	}
	string func = "";
	nhood_size = 3.0;
	num_expansions = 200;
	for (list<string>::iterator i = toks.begin(); i != toks.end(); i++) {
		const string& s = *i;
		if (s == "n" || s == "N")
			num_expansions = atoi((++i)->c_str());
		if (s == "nhood_size")
			nhood_size = atof((++i)->c_str());
		if (s == "nhood_func")
			func = *(++i);
	}

	// default neighborhood is gaussian
	if (func == "uniform") {
		neighborhood = new UniformTargetNHood(nhood_size);
	} else if (func == "gaussian") {
		neighborhood = new GaussianTargetNHood(nhood_size);
	} else if (func == "gauss_ring") {
		neighborhood = new GaussianRingTargetNHood(nhood_size);
	} else {
		neighborhood = new CircleTargetNHood(nhood_size);
	}

	// get class names
	const char* estClassName = typeid(*this).name();
	const char* nhoodClassName = typeid(*neighborhood).name();
	int i = -1, j = -1;
	while (isdigit(estClassName[++i]))
		;
	while (isdigit(nhoodClassName[++j]))
		;
	i *= sizeof(char);
	j *= sizeof(char);

	// print info
	printf("EST Algorithm (%s)\n", estClassName + i);
	printf("-- Expansions: %d\n", num_expansions);
	printf("-- Neighborhood size: %f\n", nhood_size);
	printf("-- Neighborhood function: %s\n", nhoodClassName + j);
	roadmap_size = 1;
	return true;
}

bool EST::stop()
{
	return false;
}

bool EST::findPath(Path& path)
{
	/*
	 * The basic operation of the EST is to repeatedly (n times)
	 * pick a cfg q in the tree with probability function p(q), and
	 * then try to expand into q's neighborhood.
	 *
	 * The choice of the function p(q) is very important in guiding
	 * the sampling.
	 */

	const int meter_width = 40;
	cout << "[";
	cout.fill('-');
	cout.width(meter_width);
	cout << "]" << endl << " ";
	int meter = (int) (num_expansions * 1.0 / meter_width);
	if (meter == 0)
		meter = 1;

	// local planner setup
	LP * lp = getMP()->getLPs().front();
	CFlockState * shepherd = getMP()->getShepherds().front();
	if (!isClass(lp, "LP_sb")) {
		cerr << "! ERROR: EST requires use of lp_sb." << endl;
		exit(1);
	}
	((LP_sb*) lp)->setBehavior(shepherd->getType()->getBehaviorRule());
	lp->setStopFunc(this);

	// expand the tree n times
	for (int i = 0; i < num_expansions; i++) {

		// pick a random config and expand tree from it.
		bool expanded = expandFrom(pickRandomNode());

		// check if we have reached goal
		if (expanded && isGoal()) {
			// solution found, quit
			cout << endl << "- Solution found!" << endl;
			return true;
		}

		getMP()->printStatistics();

		//check budget
		unsigned int sim_time=lp->getTotalSimulateStepCount();
		if(sim_time>=m_sim_budget){ //running out of time
			cout << endl << "- Time Budget ("<<sim_time<<"/"<<m_sim_budget<<") Exceeded!" << endl;
			break;
		}

		// progress meter
		if (i % meter == 0 && i != 0)
			cout << "*" << flush;
	}
	cout << endl;
	return false;
}

bool EST::expandFrom(VID p_id)
{
	/*
	 * This function attempts to connect from p
	 * to a new configuration q which is close to p.
	 */
	Roadmap * rm = getMP()->getRM();
	DM * dm = getMP()->getDM();
	Cfg p = rm->getData(p_id);
	Cfg q = p;

	// tweak q
	if (!neighborhood->tweak(q))
		return false;

	// simulate
	UINT steps = simulate(p, q);

	// could expand, add it to the tree.
	fromFlockState(q);
	q.m_sim_time_steps = steps;
	q.m_flock_dir = (q.m_flock_cen - p.m_flock_cen).normalize();
	q.m_in_map = true;
	float dist = dm->distsqr(q, p);
	VID q_id = rm->addnode(q);
	rm->addedge(p_id, q_id, dist);
	roadmap_size++;

	return true;
}

UINT EST::simulate(Cfg& p, Cfg& q)
{
	// set shepherd goals
	for (FSLIST::iterator i = getMP()->getShepherds().begin(); i
	        != getMP()->getShepherds().end(); i++)
		((CHerdingFlockState*) (*i))->goal = q.m_flock_tar;

	// attempt to connect p to q
	return getMP()->getLPs().front()->connect(p, q,0,true);
}

/*-------------------------------------------------------
 * Basic EST
 * The BasicEST uses a uniform random method to choose the next
 * node to expand in the tree. This may lead to dense oversampling in
 * some areas of the space.
 *-------------------------------------------------------*/

VID BasicEST::pickRandomNode()
{
	return int(getMP()->getRNG()->uniform(0.0, roadmap_size));
}

/*-------------------------------------------------------
 * Min EST
 * The MinEST algorithm always deterministically picks the node
 * that has the fewest neighbors.
 *-------------------------------------------------------*/

bool MinEST::initialize(list<string>& toks)
{
	bool test = EST::initialize(toks);
	if (test) {
		num_neighbors = (UINT*) calloc(num_expansions + 1, sizeof(UINT));
		min_idx = 0;
		return true;
	}
	return test;
}

MinEST::~MinEST()
{
	if (num_neighbors)
		free(num_neighbors);
}

VID MinEST::pickRandomNode()
{
	return min_idx;
}

bool MinEST::expandFrom(VID p_id)
{
	// expand as usual
	bool expanded = EST::expandFrom(p_id);

	// ...and update the num_neighbors list
	// for each node in roadmap, check if p is close.
	// if so, increment both ID's neighbor list.
	if (expanded) {
		Roadmap * rm = getMP()->getRM();
		vector<Cfg> & rm_data = rm->getVector();
		VID q_id = rm_data.size() - 1;
		Cfg & q = rm_data[q_id];
		//num_neighbors[p_id]+=5;
		// update neighbor list
		int min = num_expansions + 1, i = 0;
		for (i = 0; i < roadmap_size - 1; i++) {
			double dist = (rm_data[i].m_flock_tar - q.m_flock_tar).norm();
			if (dist < nhood_size) {
				num_neighbors[i]++;
				num_neighbors[q_id]++;
			}
		}
		for (i = 0; i < roadmap_size; i++) {
			if (num_neighbors[i] < min) {
				min = num_neighbors[i];
				min_idx = i;
			}
		}

	} else {
		return false;
	}
}

/*-------------------------------------------------------
 * Naive EST
 * The NaiveEST algorithm attempts to limit the oversampling
 * by making it more likely to select nodes that have fewer
 * neighbors.
 *-------------------------------------------------------*/

bool NaiveEST::initialize(list<string>& toks)
{
	bool test = EST::initialize(toks);
	if (test) {
		num_neighbors = (UINT*) calloc(num_expansions + 1, sizeof(UINT));
		prob_dist = (UINT*) calloc(num_expansions + 1, sizeof(UINT));
		return true;
	}
	return test;
}

NaiveEST::~NaiveEST()
{
	if (num_neighbors)
		free(num_neighbors);
	if (prob_dist)
		free(prob_dist);
}

VID NaiveEST::pickRandomNode()
{
	int rnd = int(getMP()->getRNG()->uniform(0.0, prob_dist[roadmap_size - 1]));

	int i;
	for (i = 0; i < roadmap_size - 1; i++)
		if (prob_dist[i + 1] >= rnd)
			break;

	return i;
}

bool NaiveEST::expandFrom(VID p_id)
{
	// expand as usual
	bool expanded = EST::expandFrom(p_id);

	// ...and update the num_neighbors list
	// for each node in roadmap, check if p is close.
	// if so, increment both ID's neighbor list.
	if (expanded) {
		Roadmap * rm = getMP()->getRM();
		vector<Cfg> & rm_data = rm->getVector();
		VID q_id = rm_data.size() - 1;
		Cfg & q = rm_data[q_id];
		// update neighbor list
		int max = 0, i = 0;
		for (i = 0; i < roadmap_size - 1; i++) {
			double dist = (rm_data[i].m_flock_tar - q.m_flock_tar).norm();
			if (dist < nhood_size) {
				num_neighbors[i]++;
				num_neighbors[q_id]++;
			}
		}
		for (i = 0; i < roadmap_size; i++)
			if (num_neighbors[i] > max)
				max = num_neighbors[i];
		// make prob_dist into a kind-of CDF
		prob_dist[0] = 0;
		for (i = 1; i < roadmap_size; i++)
			prob_dist[i] = prob_dist[i - 1] + (max - num_neighbors[i]);

	} else {
		return false;
	}
}

/*-------------------------------------------------------
 * Heuristic EST
 * The heuristic EST encorporates a variety of heuristics
 * in determining which node to expand.
 *-------------------------------------------------------*/

bool HeuristicEST::initialize(list<string>& toks)
{
	bool test = EST::initialize(toks);
	if (test) {
		num_neighbors = (UINT*) calloc(num_expansions + 1, sizeof(UINT));
		out_degree = (UINT*) calloc(num_expansions + 1, sizeof(UINT));
		wavefront = (UINT*) calloc(num_expansions + 1, sizeof(UINT));
		dist_to_goal = (double *) calloc(num_expansions + 1, sizeof(double));
		out_degree[0] = 1;
		selected_index = 0;
		return true;
	}
	return test;
}

HeuristicEST::~HeuristicEST()
{
	if (num_neighbors)
		free(num_neighbors);
	if (out_degree)
		free(out_degree);
	if (dist_to_goal)
		free(dist_to_goal);
	if (wavefront)
		free(wavefront);
}

bool HeuristicEST::findPath(Path& path)
{
	dist_to_goal[0] = (getMP()->getStart() - m_goal_pos).norm();
	return EST::findPath(path);
}

VID HeuristicEST::pickRandomNode()
{
	return selected_index;
}

bool HeuristicEST::expandFrom(VID p_id)
{
	/*
	 * This function attempts to connect from p
	 * to a new configuration q which is close to p.
	 */
	Roadmap * rm = getMP()->getRM();
	DM * dm = getMP()->getDM();
	vector<Cfg> & rm_data = rm->getVector();
	Cfg p = rm->getData(p_id);
	Cfg q = p;

	// tweak q
	if (!neighborhood->tweak(q))
		return false;

	// simulate
	UINT steps = simulate(p, q);

	// could expand, add it to the tree.
	fromFlockState(q);
	q.m_sim_time_steps = steps;
	q.m_flock_dir = (q.m_flock_cen - p.m_flock_cen).normalize();
	q.m_in_map = true;
	float dist = dm->distsqr(q, p);
	VID q_id = rm->addnode(q);
	rm->addedge(p_id, q_id, dist);
	roadmap_size++;
	wavefront[q_id] = wavefront[p_id]+1;

	// update num_neighbors
	int i;
	// for each node i except the one just inserted
	for (i = 0; i < roadmap_size - 1; i++) {
		// if dist between i and q is less than nhood_size
		// then i and q are neighbors
		double dist = (rm_data[i].m_flock_tar - q.m_flock_tar).norm();
		if (dist < nhood_size) {
			num_neighbors[i]++;
			num_neighbors[q_id]++;
		}
	}

	// update out_degree
	out_degree[p_id]++;

	// update dist_to_goal
	dist_to_goal[q_id] = (q.m_flock_tar - m_goal_pos).norm();

	// find the best (minimal) scoring goal
	double w_wfront = 0.2;
	double w_neighbors = 0.8;
	double w_out_degree = 0.0;
	double w_dist_to_goal = 0.2;

	double min = numeric_limits<double>::infinity();
	int min_idx = -1;
	for (i = 0; i < roadmap_size; i++) {
		double score = w_wfront * wavefront[i]
		               + w_neighbors * num_neighbors[i]
		               + w_out_degree * out_degree[i]
		               + w_dist_to_goal * dist_to_goal[i];
		if(score < min) {
			min = score;
			min_idx = i;
		}
	}
	selected_index = min_idx;
	return true;
}
