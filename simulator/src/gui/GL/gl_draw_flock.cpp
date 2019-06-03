#include "shadowMap_glsl.h"
#include "gl_draw_flock.h"
#include "gl_draw.h" //use help function in there...


//
// draw All flock types in the environment
//


glDrawFlocks::glDrawFlocks(list<CFlock*>& flock, bool b_Show3D)
:m_flock(flock)
{
	m_bDrawID=false;  //ID
	m_bDrawDir=false; //view dir
	m_bDrawVR=false;  //view range
	
	//create and add a draw object for each obst
	typedef list<CFlock*>::iterator IT;
    for( IT i=flock.begin();i!=flock.end();i++ ){
        CFlock* flock_type=*i;
        glDrawFlock * tmp=new glDrawFlock(flock_type);
        assert(tmp);
        tmp->setDraw3D(b_Show3D);
        addDrawObj(tmp);
    }
}	
	
void glDrawFlocks::draw()
{
    glEnable(GL_LIGHTING);
    sh_Draw::draw();
    if(m_bDrawID)  drawID();
	if(m_bDrawDir) drawDirections();
	if(m_bDrawVR)  drawViewRange();
}


void glDrawFlocks::drawID()
{
    glDisable(GL_LIGHTING);
    glColor3d(0.2,0.2,0.1);
    typedef list<CFlock*>::iterator FIT;
    char value[32];
    //for each type of flock
    for( FIT i=m_flock.begin();i!=m_flock.end();i++ ){
        CFlock* flock_type=*i;
        int size=flock_type->getStateSize();
        float height=1.2*flock_type->getGeometry().getHeight();
        //for each state
        for( int i=0;i<size;i++ ){
            CFlockState & state = flock_type->getState(i);
            const Point2d& pos=state.getPos();
            sprintf(value,"%d",state.getID());
            drawstr(pos[0],height,pos[1],value);
        }
    }//end OIT
}

void glDrawFlocks::drawDirections()
{
    glDisable(GL_LIGHTING);
    typedef list<CFlock*>::iterator FIT;
    //for each type of obst
    for( FIT i=m_flock.begin();i!=m_flock.end();i++ ){
        CFlock* flock_type=*i;
        float radius=flock_type->getGeometry().getRadius();
        int size=flock_type->getStateSize();
        //for each obst
        for( int i=0;i<size;i++ ){
            CFlockState & state = flock_type->getState(i);
            const Point2d& pos=state.getPos();
            float y=state.getRot();
			const Point3d& color=state.getColor();
            glPushMatrix();
            glTranslated(pos[0],0,pos[1]);
            glRotated(y,0,1,0);
			glColor3d(color[0]/4,color[1]/4,color[2]/4);
            drawArrow(radius*2);
            glPopMatrix();
        }
    }//end OIT
}

void glDrawFlocks::drawViewRange()
{
    glDisable(GL_LIGHTING);
    glLineWidth(0.5);
    typedef list<CFlock*>::iterator FIT;
    //for each type of obst
    for( FIT i=m_flock.begin();i!=m_flock.end();i++ ){
        CFlock* flock_type=*i;
        float radius=flock_type->getViewRadius();
        float angle=flock_type->getViewAngle();
        int size=flock_type->getStateSize();
        //for each obst
        for( int i=0;i<size;i++ ){
            CFlockState & state = flock_type->getState(i);
            const Point2d& pos=state.getPos();
            float y=state.getRot();
			const Point3d& color=state.getColor();
            glPushMatrix();
            glTranslated(pos[0],0,pos[1]);
            glRotated(y,0,1,0);
			glColor3d(color[0]/4,color[1]/4,color[2]/4);
            drawCircle(radius,angle);
            glPopMatrix();
        }
    }//end OIT
}

void glDrawFlocks::setDraw3D(bool flag) 
{
    typedef list<sh_Draw*>::iterator DIT;
    for(DIT i=m_drawobjs.begin();i!=m_drawobjs.end();i++)
        ((glDrawFlock*)(*i))->setDraw3D(flag);
}

//
// draw individual flock type
//


glDrawFlock::glDrawFlock(CFlock * flock_type)
{
	m_flock_type=flock_type;
    m_DisplayList=-1;
}

//////////////////////////////////////////////////////////////////////
// Core
void glDrawFlock::draw()
{
	if(m_DisplayList<0){
		assert(m_flock_type);
		buildGLModel();
	}
	    
	//CRobot2D & geo = m_flock_type->getGeometry();
	int size=m_flock_type->getStateSize();

    //for each m_flock
    for( int i=0;i<size;i++ ){
		CFlockState & state = m_flock_type->getState(i);
		

		const Point3d& color=state.getColor();
		setColor(color);

		glPushMatrix();
        const Point2d & pos=state.getPos();
        float * rot=state.getRotM();
		glTranslatef( pos[0], 0, pos[1]);
		double r[16]={ rot[0],rot[3],rot[6],0, 
					   rot[1],rot[4],rot[7],0,
					   rot[2],rot[5],rot[8],0, 0,0,0,1};
		glMultMatrixd(r);
	
		{
			glm::mat4 M(rot[0], rot[3], rot[6], 0,
				        rot[1], rot[4], rot[7], 0,
				        rot[2], rot[5], rot[8], 0, 0, 0, 0, 1);
			M = glm::translate(M, glm::vec3(pos[0], -0.5, pos[1]));
			ShadowMap_GLSL::setM(&M[0][0]);
		}

		////////////////////////////////////////////////////////////////////////////////
	
		if(b_Show3D==false) glColor3fv(color.get());
		else glColor3f(0.45f,0.45f,0.45f);
		glPushMatrix();
		glDisable(GL_LIGHTING);
		glTranslatef(0,-0.5f,0);
		glCallList(m_DisplayList);
		glEnable(GL_LIGHTING);

		////////////////////////////////////////////////////////////////////////////////
		glPopMatrix();
		if(b_Show3D){

		    //{//for shadow map
      //          glMatrixMode(GL_TEXTURE);
      //          glActiveTextureARB(GL_TEXTURE0);
      //          glPushMatrix();
      //          glTranslatef( pos[0], 0, pos[1]);
      //          glMultMatrixd(r);
      //      }

		    gl_draw_PolyhedronModel::draw();

            //{//for shadow map
            //    glPopMatrix();
            //    glMatrixMode(GL_MODELVIEW); //back to normal matrix mode
            //}
		}
		glPopMatrix();

	}//end for i

	{
		glm::mat4 M;
		ShadowMap_GLSL::setM(&M[0][0]);
	}
}

void glDrawFlock::castShadow()
{
    if(b_Show3D==false) return;

    if(m_DisplayList<0){
        assert(m_flock_type);
        buildGLModel();
    }

    //CRobot2D & geo = m_flock_type->getGeometry();
    int size=m_flock_type->getStateSize();

    //for each m_flock
    for( int i=0;i<size;i++ ){
        CFlockState & state = m_flock_type->getState(i);

        glPushMatrix();
        const Point2d & pos=state.getPos();
        float * rot=state.getRotM();
        glTranslatef( pos[0], 0, pos[1]);
        double r[16]={ rot[0],rot[3],rot[6],0,
                       rot[1],rot[4],rot[7],0,
                       rot[2],rot[5],rot[8],0, 0,0,0,1};
        glMultMatrixd(r);
        gl_draw_PolyhedronModel::draw();
        glPopMatrix();
    }//end for i
}

bool glDrawFlock::buildGLModel()
{
	IModel * raw_model = m_flock_type->getRawModel();
    if( gl_draw_PolyhedronModel::buildGLModel(*raw_model)==false )
        return false;

    //Build projected GL model
    if( buildGL2D()==false ) return false;
    return true;
}

bool glDrawFlock::buildGL2D()
{
    CRobot2D & robot = m_flock_type->getGeometry();
    
    const PtVector& geo=robot.getGeo();
    const TriVector& tri=robot.getTri();
    
    //Build display list
    m_DisplayList = glGenLists(1);
    glNewList(m_DisplayList, GL_COMPILE);
    glBegin(GL_TRIANGLES);

    typedef TriVector::const_iterator TIT;
    for( TIT iT=tri.begin(); iT!=tri.end(); iT++ )
    {
        const Tri& t=*iT;
        int vid1=(int)t[0], vid2=(int)t[1], vid3=(int)t[2];
        float p1[3]={geo[vid1][0],0,geo[vid1][2]};
        float p2[3]={geo[vid2][0],0,geo[vid2][2]};
        float p3[3]={geo[vid3][0],0,geo[vid3][2]};
        
        glVertex3fv(p3);
        glVertex3fv(p2);
        glVertex3fv(p1);
    }

    glEnd();
    glEndList();

    return true;
}

