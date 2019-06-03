/**
  * Interaction class for opengl.
  * OpenGL Interactor. (gliInteractor)
  *
  * 2001/3/19 Jyh-Ming
  */

#ifndef _GLI_INTERACTOR_H_
#define _GLI_INTERACTOR_H_

#if GL_ON
#ifdef MACOS
#include <GLUT/glut.h>
#else
#include <stdlib.h>
#include <GL/glut.h>
#endif

// Include GLM
#include "GL/glm/glm.hpp"
#include "GL/glm/gtc/matrix_transform.hpp"


class gli {

    ////////////////////////////////////////////////////////////////////////////////
    //
    //  TypeDefs
    //
    ////////////////////////////////////////////////////////////////////////////////

    typedef void (* GLI_DISPLAY_FUNC) ( void );
    typedef void (* GLI_MOUSE_FUNC)   ( int button, int state, int x, int y );
    typedef void (* GLI_MOTION_FUNC)  ( int x, int y );

public:

    ////////////////////////////////////////////////////////////////////////////////
    //
    //  Initializer
    //
    ////////////////////////////////////////////////////////////////////////////////

    static void gliInit();
    static void gliMainLoop() { glutMainLoop(); }

    ////////////////////////////////////////////////////////////////////////////////
    //
    //  Access Methods
    //
    ////////////////////////////////////////////////////////////////////////////////

    static void gliMouseFunc( GLI_MOUSE_FUNC mouse ){
        m_mouseFunc = mouse;
    }

    static void gliDisplayFunc( GLI_DISPLAY_FUNC display ){
        m_dislpayFunc = display;
    }

    static void gliMotionFunc( GLI_MOTION_FUNC motion ){
        m_motionFunc = motion;
    }

	static void gliDisableMouse() { m_DisalbeMouseControl=true; }
	static void gliEndMouse()     { m_DisalbeMouseControl=true; }

	static const float * getCameraPos() { return m_CameraPos;   }
	static const float * getWindowX()   { return m_WindowX;     }
	static const float * getWindowY()   { return m_WindowY;     }
    static float   getAzimuth()   { return m_currentAzim; }
    static float   getElevation() { return m_currentElev; }

    static void setAzimuth  (float azimuth)   { m_currentAzim = azimuth;   }
    static void setElevation(float elevation) { m_currentElev = elevation; }
    static void setCameraPos(float x, float y, float z)
    {
        m_CameraPos[0] = x;
        m_CameraPos[1] = y;
        m_CameraPos[2] = z;
    }

	static bool is3DView(){ return m_3D; } //check if 3d view
	static void enable3DView(){ m_3D=true; } //this will disable 2d view
	static void enable2DView(float left, float right, float bottom, float top)
	{ m_3D=false; m_2d_view[0]=left; m_2d_view[1]=right; m_2d_view[2]=bottom; m_2d_view[3]=top; }

	static glm::mat4 getViewMatrix();

protected:

    static void gliDisplay( void );
    static void gliMouse( int button, int state, int x, int y );
    static void gliMotion( int x, int y );
	static void gliRotateX(float v[3], float degree);
	static void gliRotateY(float v[3], float degree);

private:

    static GLI_DISPLAY_FUNC  m_dislpayFunc;
    static GLI_MOUSE_FUNC    m_mouseFunc;
    static GLI_MOTION_FUNC   m_motionFunc;

	static bool m_DisalbeMouseControl;

	static GLfloat m_CameraPos[3];
	static GLfloat m_deltaDis[3];

    static GLfloat m_currentAzim, m_deltaAzim;
    static GLfloat m_currentElev, m_deltaElev;

    static int m_StartX;
    static int m_StartY;
    static int m_PressedButton;

	static float m_WindowX[3];
	static float m_WindowY[3];

	static bool m_3D; //if true, perspective projection is used. Otherwise orthogonal view is used
	static float m_2d_view[4];
};

#endif 
#endif //_GLI_INTERACTOR_H_
