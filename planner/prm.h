#ifndef _PMP_PRM_H_
#define _PMP_PRM_H_

#include <list>
#include <string>
#include <float.h>
using namespace std;

#include "cfg.h"
#include "multi_herding_rules.h"

class Roadmap;
class Path;

class GraphPMP //graph based probabilitic motion planning
{
public:
    //create one of the tree-based planners
    static GraphPMP * create(list<string>& toks);

    virtual bool initialize(list<string>& toks)=0;
    virtual bool findPath(Path& path)=0;
};

class Graph_meta_fuzzy
{
public:
	
	virtual bool initialize(list<string>& toks);
    virtual bool findPath(Path& path);

private:
	
	void sample();
	void connect();
	
	int n; //number of "meta" configurations
	int k; //connection between k closest configurations
};

#endif //_PMP_PRM_H_


