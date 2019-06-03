/*
 * shepherding_gui.h
 *
 * 
 * An OpenGL-based gui extended from shGLSimulate
 * 
 * This class mainly handles huamn-computer interaction stuff
 * using glut interface. 
 *
 * TODO: Mouse control does not seem to work anymore??
 * 
 */
 
#ifndef _SHEPHERDING_GUI_H_
#define _SHEPHERDING_GUI_H_

#include "shepherding_base.h"

class shepherding_gui : public shGLSimulate
{
public:

    ///////////////////////////////////////////////////////////////////////
    shepherding_gui(); 

    virtual unsigned int simulate();
        
protected:

    static void Mouse( int button, int state, int x, int y );
    static void Special( int key, int x, int y );
    static void PassiveMotion( int x, int y );
    static void Motion( int x, int y );
};

#endif //_SHEPHERDING_GUI_H_


