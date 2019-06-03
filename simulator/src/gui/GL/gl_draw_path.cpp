///////////////////////////////////////////////////////////////////////////////
#include "gl_draw_path.h"

#define HEIGHT -0.4

glDrawPath::glDrawPath(const list<Point2d>& ptlist, float r, bool text)
{
	this->ptlist=ptlist;
	this->radius=r;
	this->text=text;
}

void glDrawPath::draw()
{
    typedef list<Point2d>::const_iterator PIT;

    {//draw circles
        for( PIT ip=ptlist.begin();ip!=ptlist.end();ip++ ){
            const Point2d & p1=*ip;
            glPushMatrix();
            glTranslated(p1[0],HEIGHT,p1[1]);
            drawCircle(radius,PI2,true);
            glPopMatrix();
        }//end for
    }
 
    {//draw lines
        glBegin(GL_LINE_STRIP);
		glColor3f(0.7f,0.7f,0.2f);
        for( PIT ip=ptlist.begin();ip!=ptlist.end();ip++ ){
            const Point2d & p1=*ip;
            glVertex3d(p1[0],HEIGHT,p1[1]);
        }//end for
        glEnd();
    }
    
    //draw text
    if( !text ) return;
    glPushAttrib(GL_CURRENT_BIT);
    glColor3f(1,0,0);
    const Point2d & s=ptlist.front();
    drawstr(s[0],HEIGHT+1,s[1],"S");
    glColor3f(0,0,1);
    const Point2d & e=ptlist.back();
    drawstr(e[0],HEIGHT+1,e[1],"G");
    glPopAttrib();
}

