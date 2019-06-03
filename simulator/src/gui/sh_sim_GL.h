/*
 * sh_sim_GL.h
 *
 * 
 * An OpenGL-based simulation
 * 
 * This class sets up glut and uses GL draw functoins in GL sub dir
 * This class has an instance of glRenderer to render everything.
 * 
 * TODO: move keyboard control to shepherding...
 * 
 * Last Major Modification : J-M Lien 12/28/2009
 *
 */
 
#ifndef _SH_SIM_GL_H_
#define _SH_SIM_GL_H_

#include "sh_sim.h"
#include "GL/gl_draw.h"

class shGLSimulate : public shSimulate
{
public:

    ///////////////////////////////////////////////////////////////////////
    shGLSimulate();
    virtual ~shGLSimulate(){}
    virtual bool initialize();
    virtual unsigned int simulate();
    virtual void stopsimulate();
    
    ///////////////////////////////////////////////////////////////////////
    //Access functions
    virtual sh_Draw * getRenderer() { return renderer; }
    void setWindowSize(int w, int h){ windowW=w; windowH=h; }

protected:
    
    ///////////////////////////////////////////////////////////////////////
    void startSim();
    bool InitGL(); //initialize openGL
    void fullScreen();
    void toggle2D3D();
    
    ///////////////////////////////////////////////////////////////////////
    //glut variables/callback functions
    static void Display( void );
    static void Reshape( int w, int h);
    static void Keyboard( unsigned char key, int x, int y );
    static void IdleFuncCallback();
    static void TimerCallback(int value);
    
    //point to the only instance of this class
    static shGLSimulate * gui;
    
    //Data
    glRenderer * renderer;
    bool Displayed;
    bool Simulating;//a flag that tells if it is currently simulating
    bool isGlutInitialized;
    bool fullscreen;
    bool b_Show3D;
    int drawWait;
    int windowW;
    int windowH;

    static Point3d light0_pos;
    static Point3d light1_pos;
};

#endif //_SH_SIM_GL_H_


