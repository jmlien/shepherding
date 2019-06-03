#ifndef _SH_GL_DRAW_ENVIRONMENT_H_
#define _SH_GL_DRAW_ENVIRONMENT_H_

///////////////////////////////////////////////////////////////////////////////
#include "gl_draw.h"
#include "gl_draw_bbx.h"
#include "gl_draw_obst.h"
#include "gl_draw_flock.h"
#include "sh_Environment.h"

class glDrawEnvironment : public gl_Draw
{
public: 
    
    glDrawEnvironment(CEnvironment* pEnv);
    
    //draw the entire environment
    virtual void draw();

    //access
    glDrawBoundingBox * getBBox() { return glBBox;}
    glDrawObsts * getObstacles() { return glObst;}
    glDrawFlocks * getFlocks()  { return glFlock;} 

private:

    glDrawBoundingBox   * glBBox;
    glDrawObsts         * glObst;
    glDrawFlocks        * glFlock;
    
    CEnvironment* m_pEnv;
};

#endif //_SH_GL_DRAW_ENVIRONMENT_H_


