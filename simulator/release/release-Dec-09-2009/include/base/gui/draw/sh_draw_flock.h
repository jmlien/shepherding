#ifndef _SH_DRAW_FLOCK_H_
#define _SH_DRAW_FLOCK_H_

#include "sh_FlockState.h"

void drawFlockID(list<CFlock*>& flock);
void drawFlockDirections(list<CFlock*>& flock);
void drawFlockViewRange( list<CFlock*>& flock );
void drawFlock( list<CFlock*>& flock, bool b_Show3D);

#endif //_SH_DRAW_FLOCK_H_


