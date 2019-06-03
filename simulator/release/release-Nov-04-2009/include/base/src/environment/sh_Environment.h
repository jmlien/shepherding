#if !defined(AFX_ENVIRONMENT_H__899705AA_7F67_428B_944F_50BD6B117FFD__INCLUDED_)
#define AFX_ENVIRONMENT_H__899705AA_7F67_428B_944F_50BD6B117FFD__INCLUDED_

///////////////////////////////////////////////////////////////////////////////
#include <fstream>
#include <list>
using namespace std;

///////////////////////////////////////////////////////////////////////////////
#include <Vector.h>
using namespace mathtool;

///////////////////////////////////////////////////////////////////////////////
#include "sh_BoundingBox.h"
class CBoundingBox;
class CFlockState;
class CFlock;
class CObs;

///////////////////////////////////////////////////////////////////////////////
#include "RNG.h"

///////////////////////////////////////////////////////////////////////////////
// defined in sh_gui_main.cpp (may want to move to sh_Env.cpp
class CEnvironment;
void setEnvironment(CEnvironment * e);
CEnvironment * getEnvironment();

///////////////////////////////////////////////////////////////////////////////
class CEnvironment
{

public:

    //////////////////////////////////////////////////////////////////////
    //Constructors/Destructor
    //////////////////////////////////////////////////////////////////////
    CEnvironment();
    virtual ~CEnvironment();
    bool buildGLModels();

    //////////////////////////////////////////////////////////////////////
    //Access
    const CBoundingBox& getBBX() const { return m_BBX; }
    CBoundingBox& getBBX() { return m_BBX; }
    void setBBX
    (float minx,float maxx,float miny,float maxy,float minz,float maxz)
    {
        float bbx[6]={minx, maxx, miny, maxy, minz, maxz};
        setBBX(bbx);
    }
	void setBBX(float bbx[6]){m_BBX.setBBXValue(bbx);}

    void setGroudTexture(const std::string& name){ m_BBX.setTexture(name); }

    void addFlock( CFlock* f );
    std::list<CFlock*>& getFlocks() { return m_Flocks; }
    std::list<CFlockState*>& getFlockStates();

    void addObstacle( CObs* b ) { m_Obs.push_back(b); }
    std::list<CObs*>& getObstacles() { return m_Obs; }

    void setTargetPosition(float x, float y) {
                                           m_target_x = x;
                                           m_target_y = y;
                                               }
    float getTargetPositionX() { return m_target_x; }
    float getTargetPositionY() { return m_target_y; }

    RNG * getRNG() {return rng;}
    void setRNG(RNG * r) {
    	if(rng)
    		delete rng;
    	if(r)
    		rng = r;
    	else
    		rng = NULL;
    }


///////////////////////////////////////////////////////////////////////////
//Private Methods
private:

    //<Bouding Box (Obstacles). Each plane parallel to coordinate.
    CBoundingBox m_BBX;

    //////////////////////////////////////////////////////////////////////
    //Flock Info
    std::list<CFlock*> m_Flocks;   //<Information of Flock
    std::list<CFlockState*> m_FlockStates;

    //////////////////////////////////////////////////////////////////////
    //OBs Info
    std::list<CObs*> m_Obs;      //<Obstacles

    // Target position for where the shepherd robot will be.
    float m_target_x;
    float m_target_y;

    // random number generator
    RNG * rng;
};

#endif // !defined(AFX_ENVIRONMENT_H__899705AA_7F67_428B_944F_50BD6B117FFD__INCLUDED_)

