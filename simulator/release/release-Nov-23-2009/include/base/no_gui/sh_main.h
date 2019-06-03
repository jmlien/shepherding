#ifndef _SD_NO_GUI_MAIN_H_
#define _SD_NO_GUI_MAIN_H_

typedef bool(*stoppingFuncType)(void);

struct sh_stoppable {
	virtual bool stop(){ return false; }
};

struct sh_statfunc {
	virtual void stat(){ }
};


unsigned int sh_simulate(unsigned int time_steps, sh_stoppable * stopFunc, sh_statfunc * statFunc = 0);
void setTimeStep( float t );
void setRestitution( float r );

class CEnvironment; 
CEnvironment * getEnvironment();

#endif //_SD_NO_GUI_MAIN_H_


