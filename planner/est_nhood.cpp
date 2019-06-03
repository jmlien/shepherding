/*
 * est_nhood.cpp
 *
 * Expansive-Spaces Trees Algorithm
 * Neighborhood Functions
 *
 * Based on the algorithm described in Choset [et al.]. Principles
 * of Robot Motion: Theory, Algorithms, and Implementation. 2005,
 * MIT Press. Pages 230--233.
 *
 * Created on: Feb 19, 2009
 * Author: Christopher Vo (cvo1@cs.gmu.edu)
 */

#include "mp.h"
#include "est_nhood.h"

bool GaussianTargetNHood::tweak(Cfg& q)
{
	/*
	 * This function takes q and tweaks the flock target
	 * to a gaussian random position with stdev nhood_size
	 * in x and y directions.
	 */
	RNG * rng = getMP()->getRNG();
	Vector2d delta;
	CFlockState * shepherd = getMP()->getShepherds().front();
	// grab one of the shepherds and use it
	// to determine whether a point is in collision
	int attempts = 0;
	do {
		delta[0] = rng->gauss(nhood_size);
		delta[1] = rng->gauss(nhood_size);
		shepherd->setPos(q.m_flock_tar + delta);
		// give up if cornered
		if (++attempts > 1000)
			return false;
	} while (isCollision(*getEnvironment(), *shepherd));
	q.m_flock_tar[0] = q.m_flock_tar[0] + delta[0];
	q.m_flock_tar[1] = q.m_flock_tar[1] + delta[1];
	return true;
}

bool UniformTargetNHood::tweak(Cfg& q)
{
	/*
	 * This function takes q and tweaks the flock target
	 * to a uniform random position with nhood_size
	 * in x and y directions.
	 */
	RNG * rng = getMP()->getRNG();
	Vector2d delta;
	CFlockState * shepherd = getMP()->getShepherds().front();
	// grab one of the shepherds and use it
	// to determine whether a point is in collision
	int attempts = 0;
	do {
		double half_nhood_size = nhood_size / 2;
		delta[0] = rng->uniform(-half_nhood_size, half_nhood_size);
		delta[1] = rng->uniform(-half_nhood_size, half_nhood_size);
		shepherd->setPos(q.m_flock_tar + delta);
		// give up if cornered
		if (++attempts > 1000)
			return false;
	} while (isCollision(*getEnvironment(), *shepherd));
	q.m_flock_tar[0] = q.m_flock_tar[0] + delta[0];
	q.m_flock_tar[1] = q.m_flock_tar[1] + delta[1];
	return true;
}

bool CircleTargetNHood::tweak(Cfg& q)
{
	/*
	 * This function takes q and tweaks the flock target
	 * to a uniform random position with nhood_size
	 * in a circle around q.
	 */
	RNG * rng = getMP()->getRNG();
	Vector2d delta;
	CFlockState * shepherd = getMP()->getShepherds().front();
	// grab one of the shepherds and use it
	// to determine whether a point is in collision
	int attempts = 0;
	double dir, len;
	do {
		dir = rng->uniform(0, 2 * M_PI);
		len = rng->uniform(0, nhood_size);
		delta[0] = len * cos(dir);
		delta[1] = len * sin(dir);
		shepherd->setPos(q.m_flock_tar + delta);
		// give up if cornered
		if (++attempts > 1000)
			return false;
	} while (isCollision(*getEnvironment(), *shepherd));
	q.m_flock_tar[0] = q.m_flock_tar[0] + delta[0];
	q.m_flock_tar[1] = q.m_flock_tar[1] + delta[1];
	return true;
}

bool GaussianRingTargetNHood::tweak(Cfg& q)
{
	/*
	 * This function takes q and tweaks the flock target
	 * to a uniform random position in a random direction
	 * and distance that is a gaussian with mean nhood_size
	 * away from q and standard deviation of nhood_size.
	 */
	RNG * rng = getMP()->getRNG();
	Vector2d delta;
	CFlockState * shepherd = getMP()->getShepherds().front();
	// grab one of the shepherds and use it
	// to determine whether a point is in collision
	int attempts = 0;
	double dir, len;
	do {
		dir = rng->uniform(0, 2 * M_PI);
		len = rng->gauss(nhood_size) + nhood_size;
		delta[0] = len * cos(dir);
		delta[1] = len * sin(dir);
		shepherd->setPos(q.m_flock_tar + delta);
		// give up if cornered
		if (++attempts > 1000)
			return false;
	} while (isCollision(*getEnvironment(), *shepherd));
	q.m_flock_tar[0] = q.m_flock_tar[0] + delta[0];
	q.m_flock_tar[1] = q.m_flock_tar[1] + delta[1];
	return true;
}
