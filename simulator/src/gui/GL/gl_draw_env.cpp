#include "gl_draw_env.h"

///////////////////////////////////////////////////////////////////////////////
//Includes
#include <GL/gliFont.h>
#include <GL/gliDump.h>
#include <cassert>

#ifdef _WIN32
#pragma warning(disable : 4786)
#endif

///////////////////////////////////////////////////////////////////////////////
//Contructor
glDrawEnvironment::glDrawEnvironment(CEnvironment* pEnv)
{
    //
    assert(pEnv);
    
    //create draw objects
    glBBox=new glDrawBoundingBox(pEnv->getBBX());
    assert(glBBox);
    addDrawObj(glBBox);
    
    glObst=new glDrawObsts(pEnv->getObstacles());
    assert(glObst);
    addDrawObj(glObst);
 
    glFlock=new glDrawFlocks(pEnv->getFlocks());
    assert(glFlock);
    addDrawObj(glFlock);
    
    m_pEnv=pEnv;
}


///////////////////////////////////////////////////////////////////////////////
void glDrawEnvironment::draw()
{
    drawDrawObject();
}




