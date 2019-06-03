/*
 * sh_sim_GL.h
 *
 * 
 * implements sh_sim_GL.cpp
 * 
 */
///////////////////////////////////////////////////////////////////////
// main header file
#include <GL/glew.h>
#include "shepherding_base.h"
#include "GL/gl_draw_env.h"
#include "GL/shadowMap_glsl.h"

///////////////////////////////////////////////////////////////////////
// there can be only one gui
shGLSimulate * shGLSimulate::gui=NULL;

//set light positions
Point3d shGLSimulate::light0_pos;
Point3d shGLSimulate::light1_pos;

///////////////////////////////////////////////////////////////////////
shGLSimulate::shGLSimulate()
{
    //create renderer
    renderer=new glRenderer();
    assert(renderer);
  
    Displayed=false;
    Simulating=false;//a flag that tells if it is currently simulating
    fullscreen=false;
    b_Show3D=true;
    windowW=600;
    windowH=600;
    
    // isGlutInitialized solves issues of glutPostRedisplay() being called before glutInit,
    // which causes a runtime error
    isGlutInitialized=false;

    //this controls frame rate
    drawWait = 10;
    
    //set shGLSimulate::gui as this
    gui=this;
}

bool shGLSimulate::initialize()
{
    ///////////////////////////////////////////////////////////////////////
    //make sure precondition is right
    assert(checkOnce());

    //build renderer
    renderer->build(this);

    ///////////////////////////////////////////////////////////////////////
    //Init glut
    int argc=1;
    char * argv[1]={"flocking"};

    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_RGB|GLUT_DOUBLE|GLUT_DEPTH );
    glutInitWindowPosition(50,50);
    glutInitWindowSize( windowW, windowH );
    glutCreateWindow( "flocking" );

    //place the lights outside the bbox
    const float * bbox=pEnv->getBBX().getBBXValue();
    light0_pos.set(bbox[1]*2,bbox[3]*4,10);
    light1_pos.set(bbox[0]*2,bbox[3]*4,bbox[4]*2);

    ///////////////////////////////////////////////////////////////////////
    //Init SH displaying env.
    InitGL();
    gli::gliInit();
    glutReshapeFunc(Reshape);
    glutKeyboardFunc(Keyboard);
    gli::gliDisplayFunc(Display);
    isGlutInitialized = true; // solves issues with glutPostRedisplay() being
                              // called before glutInit, which causes a
                              // runtime error

    //disable cast shadow if openGL 2.0 is not available
    glewInit();
	GLboolean GL2=glewIsSupported("GL_VERSION_2_0");
    if(renderer->isCastingShadow() && GL2==false)
        renderer->toggleCastShadow();

    //create shadow map
    if(GL2)
    {
        ShadowMap_GLSL::generateShadowFBO(windowW, windowH, light0_pos[0], light0_pos[1], light0_pos[2]);
        ShadowMap_GLSL::loadShadowShader("shepherding.vert", "shepherding.frag");
    }

    return true;
}

unsigned int shGLSimulate::simulate()
{
    ///////////////////////////////////////////////////////////////////////
    gli::gliMainLoop();

    //done
    return getCurrentTimeStep();
}

void shGLSimulate::stopsimulate()
{
    shSimulate::stopsimulate();

    if( Simulating ){
        startSim(); //this will toggle "Simulating" flag
        renderer->toggleDrawDone();
        glutPostRedisplay();
    }
}


///////////////////////////////////////////////////////////////////////
bool shGLSimulate::InitGL()
{
    //Setup light and material properties
    GLfloat Ambient[] =  { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat Diffuse[] =  { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat Specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat SpecularExp[] = { 100 };

    glMaterialfv(GL_FRONT, GL_AMBIENT, Ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, Diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, Specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, SpecularExp);
    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

	GLfloat Light_Ambient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat Light_Diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat Light_Specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, Light_Ambient);
	//glLightfv(GL_LIGHT1, GL_AMBIENT, Light_Ambient);

	glLightfv(GL_LIGHT0, GL_SPECULAR, Light_Specular);
	//glLightfv(GL_LIGHT1, GL_SPECULAR, Light_Specular);

	glLightfv(GL_LIGHT0, GL_DIFFUSE, Light_Diffuse);
	//glLightfv(GL_LIGHT1, GL_DIFFUSE, Light_Diffuse);

    glEnable(GL_LIGHT0);
    //glEnable(GL_LIGHT1);

    glLightfv(GL_LIGHT0, GL_POSITION, light0_pos.get());
    //glLightfv(GL_LIGHT1, GL_POSITION, light1_pos.get());

    //Antialias
    glShadeModel( GL_SMOOTH );
    glEnable( GL_LINE_SMOOTH );
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glHint( GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST );

    //others
    glEnable( GL_DEPTH_TEST);
    glClearColor( 0,0,0, 1 );
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glPolygonMode( GL_FRONT, GL_FILL );
    glPointSize(5);

    return true;
}

/////////////////////////////////////////////////////////////////////
void shGLSimulate::startSim()
{
    if(!Simulating){ //if not start simu
        //Get time
        glutTimerFunc(0, TimerCallback, 1);
    }
    else{
        glutTimerFunc(0, TimerCallback, 0);
    }

    Simulating=!Simulating;
}


//toggle between 2D and 3D views
void shGLSimulate::toggle2D3D()
{
    if(b_Show3D){
        const float * bbox=pEnv->getBBX().getBBXValue();
        float w=bbox[1]-bbox[0];
        float h=bbox[5]-bbox[4];
        if(w==h)
            gli::enable2DView(bbox[0],bbox[1],bbox[4],bbox[5]);
        else if(w>h){
            float d=(w-h)/2;
            gli::enable2DView(bbox[0],bbox[1],bbox[4]-d,bbox[5]+d);
        }
        else{
            float d=(w-h)/2;
            gli::enable2DView(bbox[0]-d,bbox[1]+d,bbox[4],bbox[5]);
        }
    }
    else{ gli::enable3DView(); }
    
    b_Show3D=!b_Show3D;
    renderer->getEnvironment()->getObstacles()->setDraw3D(b_Show3D);
    renderer->getEnvironment()->getFlocks()->setDraw3D(b_Show3D);
}

//toggle full screen mode
void shGLSimulate::fullScreen()
{
	static int screen_pos_x = 0;
	static int screen_pos_y = 0;

    if(!fullscreen){
        fullscreen=true;
        //glutFullScreen();

        float fsW = glutGet(GLUT_SCREEN_WIDTH);
        float fsH = glutGet(GLUT_SCREEN_HEIGHT);
        glutPositionWindow(0,0);

		screen_pos_x = glutGet((GLenum)GLUT_WINDOW_X);
		screen_pos_y = glutGet((GLenum)GLUT_WINDOW_Y);

        glutReshapeWindow(fsW, fsH);
    }
    else{
        fullscreen=false;
		glutPositionWindow(screen_pos_x, screen_pos_y);
        glutReshapeWindow(windowW, windowH);
    }
}


///////////////////////////////////////////////////////////////////////////////
//
//
// the rest of this file contains static glut callback functions
// 
//
///////////////////////////////////////////////////////////////////////////////

void shGLSimulate::Display( void )
{
    //Draw scene
    if(gui->renderer!=NULL)
    {
        if(gui->renderer->isCastingShadow()) //so we can draw shadow map
        {
            //draw shadow map
            ShadowMap_GLSL::beginShadowMap();
            gui->renderer->castShadow();
            ShadowMap_GLSL::endShadowMap();

            //draw scene with shadows
            ShadowMap_GLSL::beginSceneRender();
            gui->renderer->draw();
            ShadowMap_GLSL::endSceneRender();
        }
        else{
            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
            glMatrixMode(GL_MODELVIEW);
            gui->renderer->draw();
        }
    }

	gui->renderer->draw_textinfo();

    gui->Displayed=true;
}

void shGLSimulate::Reshape( int w, int h)
{
    //const float * bbox=gui->pEnv->getBBX().getBBXValue();
    
    //if(h>w) h=w;
    //else w=h;

    glViewport( 0, 0, (GLsizei)w, (GLsizei)h );

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( 60, h*1.0/w, 0.1, 1500 );
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

    //if(gui->renderer->isCastingShadow())
    {
        // regenerate shadow map
        ShadowMap_GLSL::clean();
        ShadowMap_GLSL::generateShadowFBO(w, h, light0_pos[0], light0_pos[1], light0_pos[2]);
        //ShadowMap_GLSL::loadShadowShader();
    }
}


void shGLSimulate::Keyboard( unsigned char key, int x, int y )
{
    glRenderer * renderer=(glRenderer*)gui->getRenderer();
    glDrawEnvironment * env=renderer->getEnvironment();
    
    switch( key ){
        case 27 : exit(0);
        case ' ': gui->startSim();                          break;
        case '1': gui->toggle2D3D();                        break;
        case '2': env->getFlocks()->toggleDrawViewRange();  break;
        case '3': env->getFlocks()->toggleDrawDir();        break;
        case '4': env->getFlocks()->toggleDrawID();         break;
        case '5': env->getBBox()->toggleBoxGround();        break;
        case '6': gui->renderer->toggleCastShadow();        break;
        case 't': renderer->toggleDrawText();               break;
        case 'f': gui->fullScreen();                        break;
        case '0': renderer->toggleSaveImage();              break; //turn on/off image dumping
        case '=': if (gui->drawWait > 1) gui->drawWait--;   break;
        case '-': gui->drawWait++;                          break;
        default: break;
    }

    if (gui->isGlutInitialized)
        glutPostRedisplay();
}

void shGLSimulate::IdleFuncCallback()
{
	if(!gui->Simulating) return;
    if(!gui->Displayed) return; //not displayed yet
    gui->simulateOnce();
    gui->Displayed=false;
}


void shGLSimulate::TimerCallback(int value)
{
    //not in simuation state
    if( value==0 || !gui->Simulating ) return;
    //in simuation state
    glutPostRedisplay();
    glutTimerFunc(gui->drawWait, TimerCallback, value);
}

