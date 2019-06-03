/*
 * est_nhood.h
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

#ifndef EST_NHOOD_H_
#define EST_NHOOD_H_

/*---------------------------------------------------------
 * These functions are used to take configuration q and set it to
 * a new configuration q' which is in the neighborhood of q.
 *---------------------------------------------------------*/

class NHood
{
public:
	NHood(double size) :
		nhood_size(size)
	{
	}
	virtual bool tweak(Cfg& q) = 0;
protected:
	double nhood_size;
};

class GaussianTargetNHood: public NHood
{
public:
	GaussianTargetNHood(double size) :
		NHood(size)
	{
	}
	virtual bool tweak(Cfg &q);
};

class UniformTargetNHood: public NHood
{
public:
	UniformTargetNHood(double size) :
		NHood(size)
	{
	}
	virtual bool tweak(Cfg &q);
};

class CircleTargetNHood: public NHood
{
public:
	CircleTargetNHood(double size) :
		NHood(size)
	{
	}
	virtual bool tweak(Cfg &q);
};

class GaussianRingTargetNHood: public NHood
{
public:
	GaussianRingTargetNHood(double size) :
		NHood(size)
	{
	}
	virtual bool tweak(Cfg &q);
};

#endif /* EST_NHOOD_H_ */
