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

//-----------------------------------------------------------------------------
struct obst_raw_data
{
	obst_raw_data(){ size=1; }
	int size;
	string geo_name;
	vector<Point3d> colors;
	vector<Point2d> positions;
	vector<float> angles;
};

struct flock_raw_data
{
	flock_raw_data(){
		size=1;
		mass=scatteredness=1;
		view_range=10, view_angle=360;
		separation=5, cohesion=5, alignment=5, obst_repulsion=5,
		damping=10, max_force=20;
	}
	
	void setupBasicFlock(CFlock * boid);
	void setupBasicForceRule(CBasicForceRule * frule);
		
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

//-----------------------------------------------------------------------------
//This class parses ws files
//and also sets up and initialize the simulation

class WSParser
{
public:
    //parsing the ws file
    bool parse(string filename);
    bool parse(istream& in);

protected:
    
    //obstacles
    bool readObstacles(list< list<string> >& tokens,obst_raw_data & data );
    bool createObstacles(obst_raw_data & data );
    
    //read basic flock stuff
    bool readFlock(list< list<string> >& tokens, flock_raw_data& data);
    //create flocks
    bool createFlock(list< list<string> >& tokens, flock_raw_data& data);
    //create shepherds
    bool createShepherds(list< list<string> >& tokens, flock_raw_data& data);
    void getAllLabels(istream& in, list< list<string> >& tokens);

    //convert a string to a list of tokens
    list<string> tokenize(char * tmp, const char * ignore);

};

//convert a string to lower case
inline string tolower(const string& s)
{
	string lower;
	for (string::const_iterator i = s.begin(); i != s.end(); i++)
	{
		char c = ::tolower(*i);
		lower.push_back(c);
	}
	return lower;
}

#endif //SHEPHERDING_PARSER_H
