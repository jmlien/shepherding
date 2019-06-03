#ifndef _JML_DM_H_
#define _JML_DM_H_

//--------------
//
// Distance metrics
//
//--------------


#include <list>
#include <string>
using namespace std;

class Cfg; //defined in cfg.h

class DM
{
public:
    
    //create one of the cfg connector
    static DM * create(list<string>& toks);
    
    //initialize itself
    virtual bool initialize(list<string>& toks){ return true; }

    virtual float distsqr(const Cfg& c1, const Cfg& c2)=0;
};

//--------------
//
// Distance metrics
//
//--------------

class DM_ShepherdOnly : public DM
{
public:
    virtual float distsqr(const Cfg& c1, const Cfg& c2);
};

class DM_FlockOnly : public DM
{
public:
    virtual float distsqr(const Cfg& c1, const Cfg& c2);
};

class DM_FlockGeo : public DM
{
public:
    virtual float distsqr(const Cfg& c1, const Cfg& c2);

private:
	float S2G_dist();
	float pathLength(const Cfg& c1, const Cfg& c2);
};


#endif //_JML_DM_H_


