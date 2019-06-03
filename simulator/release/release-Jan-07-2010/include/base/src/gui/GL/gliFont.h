#ifndef _GLI_FONT_H_
#define _GLI_FONT_H_
#include <GL/gli.h>

#if GL_ON
//copy from Mason Woo, 1999
void setfont(const char* name, int size);
void drawstr(GLfloat x, GLfloat y, GLfloat z, const char* format);
#endif

#endif //_GLI_FONT_H_


