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

#ifdef _WIN32
#pragma warning(disable : 4786 4305 4244)
#endif

///////////////////////////////////////////////////////////////////////////////
#include "gl_draw.h"
#include "sh_drawclass.h"
#include "sh_sim.h"


#include <GL/gli.h>
#include <GL/gliFont.h>

//Base class for OpenGL draw
class gl_Draw : public sh_Draw
{
public:

    gl_Draw()
    {
        m_signature="OpenGL";
    }

    virtual ~gl_Draw(){}

    //cast shadow
    virtual void castShadow()
    {
        //cast shadow using the user inserted objects
        typedef list<sh_Draw*>::iterator DIT;
        for(DIT i=m_drawobjs.begin();i!=m_drawobjs.end();i++)
        {
            dynamic_cast<gl_Draw*>(*i)->castShadow();
        }
    }
};

//forward declaration
class glDrawEnvironment; //defined in gl_draw_env.h

class glRenderer : public gl_Draw
{
public: 
    
    glRenderer();
    virtual ~glRenderer(){}
    
    //
    void build(shSimulate* sim);
    
    //draw the entire environment
    virtual void draw();

	//draw text
	virtual void draw_textinfo();

    //overwrite cast shadow function
    virtual void castShadow()
    {
        if(b_CastShadow==false) return;
        gl_Draw::castShadow();
    }

    //access
    glDrawEnvironment * getEnvironment(){ return glEnv; }
    void toggleDrawText(){ b_ShowTxT=!b_ShowTxT; }
    void toggleDrawDone(){ b_Done=!b_Done; }
    void toggleCastShadow(){ b_CastShadow=!b_CastShadow; }
    bool isCastingShadow() const { return b_CastShadow; }
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
    bool b_CastShadow;
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


