#ifndef _SH_GL_DRAW_PATH_H_
#define _SH_GL_DRAW_PATH_H_

#include "shepherding_base.h"
#include "gl_draw.h"

class glDrawPath : public gl_Draw 
{
public:
	glDrawPath(const list<Point2d>& ptlist, float r=1, bool text=false);
	virtual void draw();

	list<Point2d> ptlist; //
	float radius; 
	bool text;
};

#endif //_SH_GL_DRAW_PATH_H_
