#ifndef _SH_DRAW_H_
#define _SH_DRAW_H_

///////////////////////////////////////////////////////////////////////////////
//Drawing States
extern long CurTimeStep;
extern bool b_ShowViewRange;
extern bool b_ShowDirection;
extern bool b_ShowNumber;
extern bool b_Show3D;
extern bool b_ShowBBox;
extern bool b_ShowTxT;
extern bool b_Done;
///////////////////////////////////////////////////////////////////////////////
//Includes
class CEnvironment;
#include <string>

///////////////////////////////////////////////////////////////////////////////
void draw(CEnvironment& env);

///////////////////////////////////////////////////////////////////////////////
#include "sh_drawclass.h"
void addDrawObj(sh_Draw *v);
void addDrawInfo(const std::string& tag, const std::string& value);
void removeDrawObj( sh_Draw * v );
void removeDrawInfo( const std::string& tag );


///////////////////////////////////////////////////////////////////////////////
// help functions
void drawCircle(float radius, float angle, bool fill=false);
void drawArrow(float radius);

#endif //_SH_DRAW_H_


