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
    virtual bool initialize();
    
    static const Point<int, 2>& getRawMousePosition() { return raw_mouse_position; }
    static const Point2d& getMousePosition() { return mouse_position; }
        
protected:

    static void Mouse( int button, int state, int x, int y );
    static void Special( int key, int x, int y );
    static void PassiveMotion( int x, int y );
    static void Motion( int x, int y );
	static void IdleFuncCallback();
    
private:
    static Point<int, 2> raw_mouse_position;
    static Point2d mouse_position;
};

#endif //_SHEPHERDING_GUI_H_


