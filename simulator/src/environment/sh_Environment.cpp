/*
 * gl_draw.h
 *
 * 
 * class glRenderer: Draw the simulation in OpenGL.
 * 
 * 
 * Last Major Modification : J-M Lien 12/28/2009
 *
 */
 
 
#include "sh_Environment.h"
#include "sh_FlockState.h"
#include "sh_ObstState.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CEnvironment::CEnvironment()
{
 //do nothing now
}

CEnvironment::~CEnvironment()
{
}

void CEnvironment::addFlock( CFlock* f )
{
    static float max_R=0;
    float r=f->getGeometry().getRadius();
    if( r>max_R ){
        max_R=r;
        m_BBX.createCSPace(max_R);
    }
    if( f->getStateSize()==0 )
        cerr<<"! Warning : Flock added without states."<<endl;
    m_Flocks.push_back(f);
}

list<CFlockState*>& CEnvironment::getFlockStates()
{
    typedef list<CFlock*>::iterator FIT;
    if( m_FlockStates.empty() ){
        // create the list
        for(FIT i2=m_Flocks.begin();i2!=m_Flocks.end();i2++){
            vector<CFlockState*>& states=(*i2)->getStates();
            m_FlockStates.insert(m_FlockStates.end(),states.begin(),states.end());
        }//end for
    }
    return m_FlockStates;
}



