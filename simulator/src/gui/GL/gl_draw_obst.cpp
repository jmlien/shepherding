
#include "shadowMap_glsl.h"
#include "gl_draw_obst.h"
#include <GL/gli.h>
#include <cassert>


//
// glDrawObsts
//

//draw a list of all obstacles in the env
glDrawObsts::glDrawObsts(list<CObs*>& obsts, bool b_Show3D)
{
	//create and add a draw object for each obst
	typedef list<CObs*>::iterator OIT;
    for( OIT io=obsts.begin();io!=obsts.end();io++ ){
        CObs* obst_type=*io;
        glDrawObst * tmp=new glDrawObst(obst_type);
        assert(tmp);
        tmp->setDraw3D(b_Show3D);
        addDrawObj(tmp);
    }
}

void glDrawObsts::draw()
{
    //for each type of obst
    glEnable(GL_LIGHTING);
    sh_Draw::draw();
}

void glDrawObsts::setDraw3D(bool flag) 
{
    typedef list<sh_Draw*>::iterator DIT;
    for(DIT i=m_drawobjs.begin();i!=m_drawobjs.end();i++)
        ((glDrawObst*)(*i))->setDraw3D(flag);
}


//
// glDrawObst
//

glDrawObst::glDrawObst(CObs* obst_type)
{
    m_DisplayListID_2D=-1;
    m_obst_type=obst_type;
}

//////////////////////////////////////////////////////////////////////
// Core

void glDrawObst::draw()
{
    if(m_DisplayListID_2D<0){
        assert(m_obst_type);
        buildGLModel();
    }
    
    int size=m_obst_type->getStateSize(); 
    for(int i=0;i<size;i++)
    {
        CObsState & state = m_obst_type->getState(i);
        const Point2d & pos=state.getPos();
        //float * rot=state.getRot();
        float rot=state.getRotRadian();
        const Point3d& color=state.getColor();
		setColor(color);
        glColor3fv(color.get());

        glPushMatrix();
        
        glTranslatef( pos[0], 0, pos[1]);
		glRotated(rot * 180 / PI, 0, 1, 0);
		/*
		double r[16]={ rot[0],rot[3],rot[6],0,
		rot[1],rot[4],rot[7],0,
		rot[2],rot[5],rot[8],0, 0,0,0,1};
		glMultMatrixd(r);
		*/
		{
			float * rot = state.getRot();
			glm::mat4 M(rot[0], rot[3], rot[6], 0,
				rot[1], rot[4], rot[7], 0,
				rot[2], rot[5], rot[8], 0, 0, 0, 0, 1);

			M=glm::translate(M, glm::vec3(pos[0], -0.6, pos[1]));
			ShadowMap_GLSL::setM(&M[0][0]);
		}

        glPushMatrix();
        glTranslatef(0,-0.6,0);

        glDisable(GL_LIGHTING);
        glCallList(m_DisplayListID_2D);
        glEnable(GL_LIGHTING);

        glPopMatrix();
        if(b_Show3D)
		{
            gl_draw_PolyhedronModel::draw();
        }
        glPopMatrix();

    }//end for i

	{
		glm::mat4 M;
		ShadowMap_GLSL::setM(&M[0][0]);
	}

}

void glDrawObst::castShadow()
{
    if( b_Show3D ==false ) return; //only cast shadow when 3d is shown

    if(m_DisplayListID_2D<0){
        assert(m_obst_type);
        buildGLModel();
    }

    int size=m_obst_type->getStateSize();
    for(int i=0;i<size;i++)
    {
        CObsState & state = m_obst_type->getState(i);
        const Point2d & pos=state.getPos();
        float * rot=state.getRot();
        glPushMatrix();
        glTranslatef( pos[0], 0, pos[1]);
        double r[16]={ rot[0],rot[3],rot[6],0,
                       rot[1],rot[4],rot[7],0,
                       rot[2],rot[5],rot[8],0, 0,0,0,1};
        glMultMatrixd(r);
	
        gl_draw_PolyhedronModel::draw();
        glPopMatrix();
    }//end for i

}


bool glDrawObst::buildGLModel()
{
    IModel * raw_model = m_obst_type->getRawModel();
    if( gl_draw_PolyhedronModel::buildGLModel(*raw_model)==false )
        return false;

    if( buildGL2D()==false ) return false;
    return true;
}

bool glDrawObst::buildGL2D()
{

    CObs2D & obs2d = m_obst_type->getGeometry();
    
    const PtVector& geo=obs2d.getGeo();
    const TriVector& tri=obs2d.getTri();
    const TriVector& line=obs2d.getLine();
    
    
    typedef TriVector::const_iterator TIT;
    
    //Build display list
    m_DisplayListID_2D = glGenLists(1);
    glNewList(m_DisplayListID_2D, GL_COMPILE);
    glBegin(GL_TRIANGLES);
    for( TIT iT=tri.begin(); iT!=tri.end(); iT++ )
    {
        const Tri& t=*iT;
        int vid1=(int)t[0], vid2=(int)t[1], vid3=(int)t[2];
        float p1[3]={geo[vid1][0],0,geo[vid1][2]};
        float p2[3]={geo[vid2][0],0,geo[vid2][2]};
        float p3[3]={geo[vid3][0],0,geo[vid3][2]};
        
        //glBegin(GL_TRIANGLES);
        glVertex3fv(p3);
        glVertex3fv(p2);
        glVertex3fv(p1);
        //glEnd();
    }
    glEnd();

    //draw boundary
    glBegin(GL_LINES);
    glColor3f(0,0,0);
    int tid=0;
    for( TIT iT=tri.begin(); iT!=tri.end(); iT++,tid++ )
    {
        const Tri& t=*iT;
        int vid1=(int)t[0], vid2=(int)t[1], vid3=(int)t[2];
        float p1[3]={geo[vid1][0],0,geo[vid1][2]};
        float p2[3]={geo[vid2][0],0,geo[vid2][2]};
        float p3[3]={geo[vid3][0],0,geo[vid3][2]};
        
        if(line[tid][0]!=0) {
            glVertex3fv(p1);
            glVertex3fv(p2);
        }
        
        if(line[tid][1]!=0) {
            glVertex3fv(p2);
            glVertex3fv(p3);
        }       
        
        if(line[tid][2]!=0) {
            glVertex3fv(p3);
            glVertex3fv(p1);
        }
    }
    glEnd();

    glEndList();

    return true;
}
