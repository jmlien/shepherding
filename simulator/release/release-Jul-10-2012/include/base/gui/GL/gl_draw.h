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
 
#ifndef _SH_GL_DRAW_H_
#define _SH_GL_DRAW_H_

///////////////////////////////////////////////////////////////////////////////
#include "gl_draw.h"
#include "sh_drawclass.h"
#include "sh_sim.h"


#include <GL/gli.h>
#include <GL/gliFont.h>

//Base class for OpenGL draw
class gl_Draw : public sh_Draw{ public: gl_Draw(){m_signature="OpenGL";} };

//forward declaration
class glDrawEnvironment; //defined in gl_draw_env.h

class glRenderer : public gl_Draw
{
public: 
    
    glRenderer();
    
    //
    void build(shSimulate* sim);
    
    //draw the entire environment
    virtual void draw();

    //access
    glDrawEnvironment * getEnvironment(){ return glEnv; }
    void toggleDrawText(){ b_ShowTxT=!b_ShowTxT; }
    void toggleDrawDone(){ b_Done=!b_Done; }
    void toggleSaveImage(){ b_SaveImg=!b_SaveImg; }
    
protected:
    
    void drawDone();
    
    //draw the given text with format "tag" "value"
    virtual void drawText(const char * tag, const char * value)
    {
        //////////////////////////////////////////////
        glTranslated(0,-0.5,0);
        glColor3f(0.2,0.2,0.5);
        drawstr(0.2,0,0,tag);
        int l=strlen(tag);
        glColor3f(1,0,0);
        drawstr(0.2*l+0.5,0,0,value);
    }

    //data
    glDrawEnvironment * glEnv;
    shSimulate* m_sim;
    
    ///////////////////////////////////////////////////////////////////////////////
    //Drawing States
    bool b_ShowTxT;
    bool b_Done;
    bool b_SaveImg;
    int current_imgID;
};

///////////////////////////////////////////////////////////////////////////////
// help functions
void drawCircle(float radius, float angle, bool fill=false);
void drawArrow(float radius);
void drawLine(const Point2d & p1,const Point2d & p2, float * color);
//draw text "s" on screen at pos 
//should this function be here???
inline void sh_drawString(const Point3d& pos,const std::string& s)
{ 
    drawstr(pos[0],pos[1],pos[2],s.c_str());
}


#endif //_SH_GL_DRAW_H_


