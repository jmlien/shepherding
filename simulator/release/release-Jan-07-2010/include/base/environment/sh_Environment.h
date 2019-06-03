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
class CEnvironment
{

public:

    //////////////////////////////////////////////////////////////////////
    //Constructors/Destructor
    //////////////////////////////////////////////////////////////////////
    CEnvironment();
    virtual ~CEnvironment();

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

    void addFlock( CFlock* f );
    list<CFlock*>& getFlocks() { return m_Flocks; }
    list<CFlockState*>& getFlockStates();

    void addObstacle( CObs* b ) { m_Obs.push_back(b); }
    list<CObs*>& getObstacles() { return m_Obs; }

///////////////////////////////////////////////////////////////////////////
//Private Methods
private:

    //<Bouding Box (Obstacles). Each plane parallel to coordinate.
    CBoundingBox m_BBX;

    //////////////////////////////////////////////////////////////////////
    //Flock Info
    list<CFlock*> m_Flocks;   //<Information of Flock
    list<CFlockState*> m_FlockStates;

    //////////////////////////////////////////////////////////////////////
    //OBs Info
    list<CObs*> m_Obs;      //<Obstacles
};

#endif // !defined(AFX_ENVIRONMENT_H__899705AA_7F67_428B_944F_50BD6B117FFD__INCLUDED_)

