#ifndef _SH_GL_DRAW_OBST_H_
#define _SH_GL_DRAW_OBST_H_

#include "gl_draw.h"
#include "gl_draw_PolyhedronModel.h"
#include "sh_ObstState.h"
#include "sh_Obs2D.h"


//
// a draw class for drawing all obstacles
// 

class glDrawObsts : public gl_Draw
{
public:

    glDrawObsts(list<CObs*>& obsts, bool b_Show3D=true);
    void draw();
    
    //access
    void setDraw3D(bool flag);
    
protected:

    void drawObst( CObs* obst_type);
};


//
// a draw class for drawing individual obstacle
// 
class glDrawObst : public gl_draw_PolyhedronModel
{
public:
    //////////////////////////////////////////////////////////////////////
    // Constructor/Destructor
    glDrawObst(CObs* obst_type);

    //////////////////////////////////////////////////////////////////////
    // Core
    virtual void draw();
    
    //Access
    void setDraw3D(bool flag){ b_Show3D=flag; }

//////////////////////////////////////////////////////////////////////
// Protected
protected:

    virtual bool buildGLModel();
    
    ///////////////////////////////////////////////////////////////////////////
    bool buildGL2D();

//////////////////////////////////////////////////////////////////////
// Private
private:
    
    bool b_Show3D;
    CObs* m_obst_type; 
    int m_DisplayListID_2D;
};


#endif //_SH_DRAW_OBST_H_


