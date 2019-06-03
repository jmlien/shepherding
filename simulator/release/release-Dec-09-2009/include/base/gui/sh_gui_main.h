#ifndef _SD_MAIN_H_
#define _SD_MAIN_H_

#include "sh_main.h"

///////////////////////////////////////////////////////////////////////
int sh_start( void );
void sh_stop( void );
///////////////////////////////////////////////////////////////////////
void startSim();
void simulate();
void showViewingRange();
void showHeadingDir();
void showFlockID();
void showObstacle();
void showBBX();
void resetCamera();
void showTxt();
void showAxis();
void saveImg();
void saveEPS();
void lookthrough();

///////////////////////////////////////////////////////////////////////
// general header files
#include <GL/gli.h>
#include <string>

///////////////////////////////////////////////////////////////////////
//glut variables/callback functions
void Display( void );
void Reshape( int w, int h);
void Mouse( int button, int state, int x, int y );
void Keyboard( unsigned char key, int x, int y );
void Special( int key, int x, int y );
void PassiveMotion( int x, int y );
void Motion( int x, int y );

bool Init();
void SetLight();
void fullScreen();

void setConvertFile( const std::string& name );
long getCurrentTimeStep();

//Events
#define Simulation_Event    1
#define Show_Obs3D_Event    1
#define Show_OverHead_Event 2
#define Show_Range_Event    3
#define Show_BBX_Event      4
#define Show_ID_Event       5

#endif //_SD_MAIN_H_


