#ifndef _META_CFG_H_
#define _META_CFG_H_

//--------------
//
// MetaCfg
//
//--------------

#include <list>
#include <iostream>
using namespace std;

#include <Point.h>
using namespace std;

class MetaCfg
{
public:

    MetaCfg(){m_flock_rad=0;}
    MetaCfg(const MetaCfg& c)
	{ 
		*this=c; 
	}

    const MetaCfg& operator=(const MetaCfg& c)
	{
		m_flock_rad=c.m_flock_rad; m_flock_cen=c.m_flock_cen; 
		m_flock_dir=c.m_flock_dir;
	}

    bool operator==(const MetaCfg& c) const
	{
		if(m_flock_rad!=c.m_flock_rad) return false;
		if(!(m_flock_cen==c.m_flock_cen)) return false;
		if(!(m_flock_dir==c.m_flock_dir)) return false;
		return true;
	}

    //this distance is unsymmetric
	float distance(const MetaCfg& c) const
	{
    	//generate a set of target positions at random
	    static const float * bbox=getEnvironment()->getBBX().getBBXValue();
    	static float bbox_x=bbox[1]-bbox[0];
	    static float bbox_z=bbox[5]-bbox[4];
    	static float bbox_dia=sqrt(bbox_x*bbox_x+bbox_z*bbox_z);
		
		//c is the target
		//this enforeces at least pushing direction is the same as the
		//target facing direction
		Vector2d vec=(c.m_flock_cen-m_flock_cen).normalize();
		if(vec*c.m_flock_dir<0.5) //vec and c.m_flock_dir have angle larger than 60 degree
			return 2; //this is the max possible distance 
		if(vec*m_flock_dir<0.5) //same reason here
			return 2;
		float d1=vec.norm()/bbox_dia;                //max is 1
		float d2=acos(m_flock_dir*c.m_flock_dir)/PI; //max is 1

		return d1+d2;
	}

    //flock radius/center
    float    m_flock_rad;              //flock's radius
    Point2d  m_flock_cen;            //flock's center
	Vector2d m_flock_dir;           //flock's direction
};

inline ostream& operator<<(ostream& out, const MetaCfg& c)
{
	out<<c.m_flock_rad<<" "<<c.m_flock_cen;
}

#endif //_META_CFG_H_
