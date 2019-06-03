/*
 * shepherding_gui.h
 *
 * 
 * implements shepherding_gui.cpp
 * 
 */
 
///////////////////////////////////////////////////////////////////////
// main header file
//#include <GL/glew.h>
#include "shepherding_gui.h"
#include "shepherding.h"

// instantiating static class variables used to store mouse position
Point<int, 2> shepherding_gui::raw_mouse_position;
Point2d shepherding_gui::mouse_position;


shepherding_gui::shepherding_gui(){}

bool shepherding_gui::initialize()
{
    shGLSimulate::initialize(); //call parent initialization function

    glutSpecialFunc(Special);
    glutPassiveMotionFunc(PassiveMotion);
    gli::gliMotionFunc(Motion);
    gli::gliMouseFunc(Mouse);
	glutIdleFunc(IdleFuncCallback);

    //let's show 2D first
    toggle2D3D();

	return true;
}

// global flag used to indicate which shepherd is selected
// (This should be moved to behavior.c
void shepherding_gui::Mouse( int button, int state, int x, int y )
{
    shepherding_gui::raw_mouse_position = Point<int, 2>(x, y);
    
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    const float* bounding_box = gui->getEnvironment()->getBBX().getBBXValue();
    float screen_width = viewport[2] - viewport[0], screen_height = viewport[3] - viewport[1];
    float environment_width = bounding_box[1] - bounding_box[0], environment_height = bounding_box[5] - bounding_box[4];
    float relative_x = (x - viewport[0])/screen_width;
    float relative_y = (y - viewport[1])/screen_height;
    
    shepherding_gui::mouse_position = Point2d(bounding_box[0] + relative_x*environment_width, bounding_box[4] + relative_y*environment_height);
}

//this should also be moved to behaviors.c
void shepherding_gui::PassiveMotion( int x, int y )
{
    shepherding_gui::raw_mouse_position = Point<int, 2>(x, y);
    
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    const float* bounding_box = gui->getEnvironment()->getBBX().getBBXValue();
    float screen_width = viewport[2] - viewport[0], screen_height = viewport[3] - viewport[1];
    float environment_width = bounding_box[1] - bounding_box[0], environment_height = bounding_box[5] - bounding_box[4];
    float relative_x = (x - viewport[0])/screen_width;
    float relative_y = (y - viewport[1])/screen_height;
    
    shepherding_gui::mouse_position = Point2d(bounding_box[0] + relative_x*environment_width, bounding_box[4] + relative_y*environment_height);
}

void shepherding_gui::Motion( int x, int y )
{
    //do nothing yet
}


//no sure if this will work
void shepherding_gui::IdleFuncCallback()
{
	if (getSI() == NULL || getSimulator() ==NULL) return;

	shepherding_gui * me = dynamic_cast<shepherding_gui*>(getSimulator());

	if (me->getCurrentTimeStep() >= getSI()->getSimBudget() && me->getCurrentTimeStep() >= 0)
		me->stopsimulate();
	else //keep simulating
		shGLSimulate::IdleFuncCallback();


#ifdef _WIN32
	//xbox controller

	CXBOXController* xbox_controller = getSI()->getXBoxController();

	if (xbox_controller == NULL) return;
	if (xbox_controller->IsConnected())
	{
		XINPUT_KEYSTROKE key = xbox_controller->GetKeyStroke();
		if (key.Flags == XINPUT_KEYSTROKE_KEYDOWN) //a key is pressed
		if (key.VirtualKey == VK_PAD_A ||
			key.VirtualKey == VK_PAD_START)
		{
			me->startSim();
		}
		else if (key.VirtualKey == VK_PAD_BACK)
		{
			exit(0);
		}
	}
#endif

}

//
// this is specialized to herding, should be moved to 
// shepherding code.
// todo
//
void shepherding_gui::Special( int key, int x, int y )
{
    // this is assuming GLUT_KEY_F1, F2, F3, etc are a series of incrementing integer values
    getSI()->setActiveShepherdID(key - GLUT_KEY_F1);
    const std::list<CFlockState*>& shepherds = gui->getEnvironment()->getFlockStates();
    std::list<CFlockState*>::const_iterator itr = shepherds.begin();
    
    // if not a herding type, clear active shepherd flag
    if(strstr(typeid(**itr).name(), "Herding") == 0)
    {
        getSI()->setActiveShepherdID(-1);
    }
}//end Special



