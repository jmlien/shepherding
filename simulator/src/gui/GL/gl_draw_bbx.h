#ifndef _SH_GL_DRAW_BBX_H_
#define _SH_GL_DRAW_BBX_H_

#include "gl_draw.h"
#include "sh_BoundingBox.h"

class glDrawBoundingBox : public gl_Draw
{
public:

	glDrawBoundingBox(const CBoundingBox& bbx);
	virtual void draw();

	void doClip();
	void unClip();
	void toggleBoxGround(){ m_drawbox=!m_drawbox; }
	
protected:

	void drawBBX();
	void drawGround();
	int BuildGroundModel(const float bbx[6],const string& text);
	int BuildBBXModel(const float bbx[6],const string& text);
	
	const CBoundingBox& m_bbx;
	unsigned int m_texture_id;
	bool m_drawbox;
};

#endif //_SH_GL_DRAW_BBX_H_


