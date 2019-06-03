#ifndef _JML_CFG_H_
#define _JML_CFG_H_

//--------------
//
// Cfg
//
//--------------

#include <list>
#include <iostream>
using namespace std;

class Cfg
{
public:

    Cfg();
    Cfg(const Cfg& c);
    ~Cfg(){delete [] m_value; m_value=NULL;}

    float operator[](const int& i) const { return m_value[i]; }
    float& operator[](const int& i) { return m_value[i]; }

    const Cfg& operator=(const Cfg& other);
    bool operator==(const Cfg& other) const;

    float * getValue() const { return m_value; }

private:

    //all flock state
    float * m_value;

public:

    //shepherd positions
	vector<Point2d> m_shepherd_tar; //target positions (maybe empty if not used)
    vector<Point2d> m_shepherd_pos; //shepherd positions
	vector<int>     m_shepherd_ss;  //shepherd swing step

    //flock radius/center
    float m_flock_rad;       //flock's radius
    Point2d m_flock_cen;     //flock's center
	Point2d m_flock_tar;     //flock's target (may be meaningless if not used)
	Vector2d m_flock_dir;    //flock target direction
	unsigned int m_sim_time_steps; //time steps from the last cfg to this cfg
	bool m_in_map; //true if this cfg is stored in the map
	VID m_cls_vid;   //closest node in ws graph from m_flock_cen
	//long m_rand_seed;		//seed for the random number generator
};

void fromFlockState(Cfg& c); //copy from flockstates
void toFlockState(const Cfg& c);   //copy to flockstates
Cfg operator+(const Cfg& c1, const Cfg& c2);
Cfg operator-(const Cfg& c1, const Cfg& c2);
Cfg operator*(const Cfg& c1, float s);
Cfg operator/(const Cfg& c1, float s);
ostream& operator<<(ostream& out, const Cfg& c);
float normsqr(Cfg& c);
float dot(Cfg& c1, Cfg& c2);

#endif //_JML_CFG_H_
