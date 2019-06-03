/*
 * Random Number Generator Metaclass
 *
 *  Created on: Feb 12, 2009
 *      Author: Christopher Vo (cvo1@cs.gmu.edu)
 */

#ifndef RNG_H_
#define RNG_H_

//#include "gsl/gsl_rng.h"
//#include "gsl/gsl_randist.h"
#include "Gauss.h"
#include "randomc.h"
#include "stocc.h"

// the interface to the RNG
class RNG {
public:
	RNG(long seed) {}
	virtual ~RNG() {}

	/* Generate a random number under a uniform distribution
	 * in range [0.0, 1.0) */
	virtual double uniform() = 0;

	/* Generate a random number under a uniform distribution
	 * in range [a,b) */
	virtual double uniform(double a, double b) = 0;

	/* Generate a random number under a gaussian distribution
	 * with mean 0.0 and stdev 1.0 */
	virtual double gauss() = 0;

	/* Generate a random number under a gaussian distribution
	 * with mean 0.0 and stdev sigma */
	virtual double gauss(double sigma) = 0;

protected:
	RNG() {}
};

/*
// GSL RNG
class GSL_RNG: public RNG {
public:
	GSL_RNG(long seed) {
		rng = gsl_rng_alloc(gsl_rng_mt19937);
		gsl_rng_set(rng, seed);
	}
	virtual ~GSL_RNG() {
		gsl_rng_free(rng);
	}
	virtual double uniform() {
		return gsl_ran_flat(rng, 0.0, 1.0);
	}
	virtual double uniform(double a, double b) {
		return gsl_ran_flat(rng, a, b);
	}
	virtual double gauss() {
		return gsl_ran_gaussian(rng, 1.0);
	}
	virtual double gauss(double sigma) {
		return gsl_ran_gaussian(rng, sigma);
	}
private:
	gsl_rng * rng;
};
*/

// Pseudo random number generators
// from http://www.agner.org/random/

class P_RNG: public RNG {
public:
	P_RNG(long seed):m_uniform(seed), m_non_uniform(seed){}
	
	virtual double uniform() {
		return m_uniform.Random();
	}
	//this needs to be checked...
	virtual double uniform(double a, double b) {
		return (double)m_uniform.IRandom((int)a, (int)b);
	}
	virtual double gauss() {
		return m_non_uniform.Normal(0,1.0f);
	}
	virtual double gauss(double sigma) {
		return m_non_uniform.Normal(0,(float)sigma);
	}
private:
	
	CRandomMother  m_uniform;
	StochasticLib1 m_non_uniform;
};

//C RNG
class STDLIB_RNG: public RNG {
public:
	STDLIB_RNG(long seed) {
		srand48(seed);
	}
	virtual double uniform() {
		return drand48();
	}
	virtual double uniform(double a, double b) {
		return (drand48() * (b - a)) + a;
	}
	virtual double gauss() {
		return mathtool::gauss(0.0f, 1.0f);
	}
	virtual double gauss(double sigma) {
		return mathtool::gauss(0.0f, (float)sigma);
	}
};

#endif /* RANDGEN_H_ */
