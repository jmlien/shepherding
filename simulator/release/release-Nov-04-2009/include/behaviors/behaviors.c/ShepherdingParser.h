#ifndef SHEPHERDING_PARSER_H
#define SHEPHERDING_PARSER_H

#include <iostream>
#include <list>
#include <vector>
#include <cstdlib>
#include <cstring>
#include "Point.h"
using namespace mathtool;
using namespace std;

class CFlock;
class CBasicForceRule;
class ShepherdingInstance;

struct obst_raw_data{
	obst_raw_data(){ size=1; }
	int size;
	string geo_name;
	vector<Point3d> colors;
	vector<Point2d> positions;
	vector<float> angles;
};

struct flock_raw_data{

	flock_raw_data(){
		size=1;
		mass=scatteredness=1;
		view_range=10, view_angle=360;
	    separation=5, cohesion=5, alignment=5, obst_repulsion=5,
		damping=10, max_force=20;
	}

	string type;
	int size;
	string geo; //filename
	Point3d color;
	float   scatteredness;
	Point2d position;
	float mass;
	float view_range, view_angle;
	float separation, cohesion, alignment, obst_repulsion,
		  damping, max_force;
};

void setupBasicFlock(CFlock * boid, flock_raw_data& data);
void setupBasicForceRule(CBasicForceRule * frule, flock_raw_data& data);
bool  parseWS(string filename);

#endif //SHEPHERDING_PARSER_H
