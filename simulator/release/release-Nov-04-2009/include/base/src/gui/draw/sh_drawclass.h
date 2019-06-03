#ifndef _SH_DRAWCLASS_H_
#define _SH_DRAWCLASS_H_

#include <GL/gli.h>
#include <GL/gliFont.h>

#include <string>
using namespace std;

#include <Point.h>
using namespace mathtool;

struct sh_Draw{ 
    sh_Draw(){}
    virtual void draw()=0; 
};
void addDrawObj( sh_Draw * v );
void addDrawInfo( const std::string& tag, const std::string& value);
void removeDrawObj( sh_Draw * v );
void removeDrawInfo( const std::string& tag );

inline void sh_drawString(const Point3d& pos,const std::string& s){ 
#if GL_ON
    drawstr(pos[0],pos[1],pos[2],s.c_str());
#endif
}

/*
//gl
inline void sh_glTranslated(float x, float y, float z){ glTranslated(x,y,z); }
inline void sh_glRotated(float angle, float x, float y, float z)
{ glRotated(angle,x,y,z); }
inline void sh_glPushMatrix(){ glPushMatrix(); }
inline void sh_glPopMatrix(){ glPopMatrix(); }
inline void sh_glVertex3d(float x, float y,float z){ glVertex3d(x,y,z); }
inline void sh_glVertex2d(float x, float y){ glVertex3d(x,0,y); }
inline void sh_glNormal3d(float x, float y, float z){ glNormal3d(x,y,z); }
inline void sh_glColor3d(float x, float y, float z){ glColor3d(x,y,z); }
inline void sh_glBegin(GLenum i){ glBegin(i); }
inline void sh_glEnd(){ glEnd(); }
inline void sh_glDisable(GLenum i){ glDisable(i); }
inline void sh_glEnable(GLenum i){ glEnable(i); }
inline void sh_glLineWidth(float i){ glLineWidth(i); }
inline void sh_glPointSize(float i){ glPointSize(i); }
inline int  sh_glGenLists(int i){ return glGenLists(i); }
inline void sh_glNewList( int i, GLenum mode ){ glNewList(i,mode); }
inline void sh_glEndList(){ glEndList(); }
inline void sh_glCallList(int id){ glCallList(id); }

//glut
inline void sh_glutSolidSphere(float r,int hr,int vr){ glutSolidSphere(r,hr,vr); }
inline void sh_glutSolidCube(float r){ glutSolidCube(r); }
*/

#endif //_SH_DRAWCLASS_H_


