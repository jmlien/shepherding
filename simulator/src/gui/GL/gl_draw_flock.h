#ifndef _SH_GL_DRAW_FLOCK_H_
#define _SH_GL_DRAW_FLOCK_H_

#include "gl_draw.h"
#include "sh_FlockState.h"
#include "gl_draw_PolyhedronModel.h"
#include "sh_Robot2D.h"

//
// class for drawing all flock types
//
class glDrawFlocks : public gl_Draw
{
public:

    glDrawFlocks(list<CFlock*>& flock, bool b_Show3D=true);
    void draw();
    
    //access
    void toggleDrawID() {m_bDrawID=!m_bDrawID;}
    void toggleDrawDir() {m_bDrawDir=!m_bDrawDir;}
    void toggleDrawViewRange() {m_bDrawVR=!m_bDrawVR;}
    void setDraw3D(bool flag);

private:

    void drawID();
    void drawDirections();
    void drawViewRange();
    
    bool m_bDrawID;  //ID
    bool m_bDrawDir; //view dir
    bool m_bDrawVR;  //view range

    list<CFlock*>& m_flock;
};



//////////////////////////////////////////////////////////////////////////
//
// class for drawing each flock type
//
class glDrawFlock : public gl_draw_PolyhedronModel
{
public:
    
    glDrawFlock(CFlock * flock_type);
    
    //////////////////////////////////////////////////////////////////////
    // Core
    virtual void draw();
    virtual void castShadow();

    //construct open gl models
    virtual bool buildGLModel();
    
    //Access
    void setDraw3D(bool flag){ b_Show3D=flag; }

protected:

	bool buildGL2D();
	
    bool b_Show3D;
    int m_DisplayList; //projected display
    CFlock * m_flock_type;
};


#endif //_SH_DRAW_FLOCK_H_


