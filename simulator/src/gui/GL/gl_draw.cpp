#include "gl_draw.h"
#include "gl_draw_env.h"

///////////////////////////////////////////////////////////////////////////////
//Includes
#include <GL/gliFont.h>
#include <GL/gliDump.h>

#ifdef _WIN32
#pragma warning(disable : 4786)
#endif

///////////////////////////////////////////////////////////////////////////////
//Draw Objects
glRenderer::glRenderer()
{
    //init variables
    current_imgID=0; //id for dumping images
    b_ShowTxT=true;
    b_Done=false;
    b_SaveImg=false;
    b_CastShadow=false;
    m_sim=NULL;
}

void glRenderer::build(shSimulate * sim)
{
	assert(sim);
    assert(sim->getEnvironment()); //make sure environment is initialized
    m_sim=sim; //keep a pointer
    
    //create draw objects
    glEnv=new glDrawEnvironment(sim->getEnvironment());
    assert(glEnv);

    //I didn't call addDrawObj(glEnv) because it will put glEnv
    //at the end of the list, which makes rendering ugly
    m_drawobjs.push_front(glEnv);
}

///////////////////////////////////////////////////////////////////////////////
void glRenderer::draw()
{
    drawDrawObject();

    //dump to images
    if(b_SaveImg)
    {
        //dump
        char number[64];
        sprintf(number,"%08d.ppm",current_imgID++);
        string filename="shepherding_dump_";
        filename=filename+number;
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        dump(filename.c_str(),viewport[2],viewport[3]);
        cerr<<"- Save Image : "<<filename<<endl;
    }
}


//draw text
void glRenderer::draw_textinfo()
{
	if (b_ShowTxT){

		//text font
		setfont("helvetica", 12);

		//
		glPushAttrib(GL_CURRENT_BIT);

		//draw reference axis
		glMatrixMode(GL_PROJECTION); //change to Ortho view
		glPushMatrix();
		glLoadIdentity();
		gluOrtho2D(0, 20, 0, 20);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glDisable(GL_LIGHTING);
		glTranslated(0, 20, 0);

		//time info
		char value[32];
		sprintf(value, "%07ld", m_sim->getCurrentTimeStep());
		drawText("Simulation Time : ", value);

		drawTextInfo();

		glPopMatrix();

		//pop GL_PROJECTION
		glMatrixMode(GL_PROJECTION); //change to Pers view
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopAttrib();
	}

	if (b_Done) drawDone();
}


//this is called when the simulation is done
void glRenderer::drawDone()
{
    glPushAttrib(GL_CURRENT_BIT);

    //draw reference axis
    glMatrixMode(GL_PROJECTION); //change to Ortho view
    glPushMatrix(); 
    glLoadIdentity();
    gluOrtho2D(0,20,0,20);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_LIGHTING);

    glLoadIdentity();
    
    glColor4f(1,1,1,0.75f);
    glBegin(GL_POLYGON);
    glVertex3f(0,0,0);
    glVertex3f(20,0,0);
    glVertex3f(20,20,0);
    glVertex3f(0,20,0);
    glEnd();

    glColor3f(0.2,0.2,0);
	setfont("helvetica", 18);
    drawstr(10,10,1,"Done");

    //pop GL_PROJECTION
    glMatrixMode(GL_PROJECTION); //change to Pers view
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopAttrib();
}

///////////////////////////////////////////////////////////////////////////////
// help function

void drawCircle(float radius, float angle, bool fill)
{
#if GL_ON
    float delta=0.2f;
    angle=2*PI-angle;
    float s=PI+angle/2; float e=3*PI-angle/2;

    /////////////////////////////////////////////////////////////
    if(!fill)
        glBegin(GL_LINE_LOOP);
    else 
        glBegin(GL_POLYGON);

    {if( angle>0 ) glVertex3f( 0, 0.2f, 0 );
    for(float theta=s; theta<e ; theta += delta )
        glVertex3f( radius*sin(theta), 0.2f, radius*cos(theta) );
    glVertex3f( radius*sin(e), 0.2f, radius*cos(e) );}
    glEnd();
#endif
}

void drawArrow(float radius)
{
#if GL_ON
    glBegin(GL_TRIANGLES);
    glVertex3d(0.4,0.2,radius);
    glVertex3d(-0.4,0.2,radius);
    glVertex3d(0,0.2,0.6+radius);
    glEnd();

    glBegin(GL_LINES);
    glVertex3d(0,0.2,0);
    glVertex3d(0,0.2,radius);
    glEnd();
#endif//GL_ON
}

void drawLine(const Point2d & p1,const Point2d & p2, float * color)
{   
#if GL_ON
    glPushAttrib(GL_CURRENT_BIT);
    glBegin(GL_LINES);
    glColor3fv(color);
        glVertex3d(p1[0],0.2f,p1[1]);
        glVertex3d(p2[0],0.2f,p2[1]);
    glEnd();
    glPopAttrib();
#endif
}

