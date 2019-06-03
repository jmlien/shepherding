#include "gl_draw_coverage.h"
#include <GL/gli.h>

CDrawCoverage::CDrawCoverage
(CEnvironment * env, CFlock * flock, int xsize, int zsize, float H,
 float R,float G,float B,float A)
{
	assert(env);
	assert(flock);
	
    m_pEnv=env;
    m_pFlock=flock;
    m_xsize=xsize;
    m_zsize=zsize;

    const float * bbx=m_pEnv->getBBX().getBBXValue();
    float xw=float(bbx[1]-bbx[0]);
    float zw=float(bbx[5]-bbx[4]);
    float xres=xw/(xsize-1);
    float zres=zw/(xsize-1);
    float xmin=bbx[0];
    float zmin=bbx[4];
    m_xres=xres;
    m_zres=zres;

    CRobot2D& geo=flock->getGeometry();
    shCD cd(m_pEnv);

    m_Colors=new GLfloat[xsize*zsize*4];
    m_Points=new GLfloat[xsize*zsize*3];
    m_TotalPoints=xsize*zsize*A;

    //init color and pos
    memset(m_Colors,0,xsize*zsize*4*sizeof(GLfloat));
    memset(m_Points,0,xsize*zsize*3*sizeof(GLfloat));
    {for(int i=0;i<xsize;i++){
        for(int j=0;j<zsize;j++){
            int index=i*zsize+j;
            m_Points[index*3]=xmin+i*xres;
            m_Points[index*3+1]=H;
            m_Points[index*3+2]=zmin+j*zres;
            m_Colors[index*4+1]=0.2;
            m_Colors[index*4]=R;
            m_Colors[index*4+1]=G;
            m_Colors[index*4+2]=B;
            m_Colors[index*4+3]=A;
            Point2d pt(m_Points[index*3],m_Points[index*3+2]);
            if( cd.isCollision(geo,pt) ) m_Colors[index*4+3]=0;
        }
    }}

    //build the index
    int xsize_1=xsize-1;
    int zsize_1=zsize-1;
    m_Index=new GLuint[xsize_1*zsize_1*4];
    {for(int i=0;i<xsize_1;i++){
        for(int j=0;j<zsize_1;j++){
            int index=(i*zsize_1+j)*4;
            m_Index[index]=(i*zsize+j);
            m_Index[index+1]=m_Index[index]+1;
            m_Index[index+2]=((i+1)*zsize+j)+1;
            m_Index[index+3]=m_Index[index+2]-1;
        }//end for
    }}//end for
}

void CDrawCoverage::draw()
{
#if GL_ON
    glDisable(GL_LIGHTING);
    //glEnableClientState(GL_VERTEX_ARRAY);
    //glEnableClientState(GL_COLOR_ARRAY);
    //sent to gl
    //glVertexPointer(3, GL_FLOAT, 0, m_Points);
    //glColorPointer(4, GL_FLOAT, 0, m_Colors);

    //glDrawElements(GL_QUADS,(m_xsize-1)*(m_zsize-1)*4,GL_UNSIGNED_INT,m_Index);
    glBegin(GL_QUADS);
    int size=(m_xsize-1)*(m_zsize-1)*4;
    for(int i=0;i<size;i++){
        glColor4fv(m_Colors+(m_Index[i]*4));
        glVertex3fv(m_Points+(m_Index[i]*3));
    }
    glEnd();
    //glDisableClientState(GL_VERTEX_ARRAY);
    //glDisableClientState(GL_COLOR_ARRAY);
#endif
}

void CDrawCoverage::update(const Point2d& pos, float dvalue, float vr)
{
    if( vr<1e-10 )  
        vr=m_pFlock->getViewRadius();

	shCD cd(m_pEnv);
    float vr2=vr*vr;
    const float * bbx=m_pEnv->getBBX().getBBXValue();
    float dxs=pos[0]-vr-bbx[0];
    float dxe=dxs+vr*2;
    float dzs=pos[1]-vr-bbx[4];
    float dze=dzs+vr*2;

    int sx=(int)ceil(dxs/m_xres);
    if(sx<0) sx=0;
    int ex=(int)floor(dxe/m_xres);
    if(ex>=m_xsize) ex=m_xsize-1;
    int sz=(int)ceil(dzs/m_zres);
    if(sz<0) sz=0;
    int ez=(int)floor(dze/m_zres);
    if(ez>=m_zsize) ez=m_zsize-1;
    for(int i=sx;i<=ex;i++){
        for(int j=sz;j<=ez;j++){
            int index=(i*m_zsize+j);
            Point2d cellpos(m_Points[index*3],m_Points[index*3+2]);
            Vector2d dir=(cellpos-pos);
            if( dir.normsqr()>vr2 ) continue;
            if( cd.isCollision(m_pFlock->getGeometry(),pos,cellpos) )
                continue;
            int cid=index*4+3;
            m_Colors[cid]+=dvalue;
            if( m_Colors[cid]<0 ) m_Colors[cid]=0;
            else if (m_Colors[cid]>1) m_Colors[cid]=1;
        }//end for
    }//end for
}

float CDrawCoverage::coverageRate()
{
    int size=m_xsize*m_zsize;
    float current=0;
    for( int i=0;i<size;i++ ){
        current+=m_Colors[i*4+3];
    }
    return 1-current/m_TotalPoints;
}

