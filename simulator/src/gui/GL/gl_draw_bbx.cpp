#include <GL/gli.h>
#include <GL/gliTexture.h>
#include "gl_draw_bbx.h"


glDrawBoundingBox::glDrawBoundingBox(const CBoundingBox& bbx)
:m_bbx(bbx)
{
    m_texture_id=0;
    m_drawbox=true;
}

void glDrawBoundingBox::draw()
{
    //drawGround();

    if(m_drawbox) drawBBX();
    else drawGround();
}

void glDrawBoundingBox::drawBBX()
{
#if GL_ON
    static int did=-1;
    if( did==-1 ){
        did=BuildBBXModel(m_bbx.getBBXValue(),m_bbx.getTexture());
    }

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glDisable(GL_LIGHTING);
    glLineWidth(1);
    glCallList(did);
    glDisable(GL_CULL_FACE);
#endif
}

void glDrawBoundingBox::drawGround()
{
#if GL_ON
    static int did=-1;
    if( did==-1 ){
        did=BuildGroundModel(m_bbx.getBBXValue(),m_bbx.getTexture());
    }

    glDisable(GL_LIGHTING);
    glLineWidth(1);
    glCallList(did);
#endif
}


void glDrawBoundingBox::doClip()
{
#if GL_ON
    static const float * bbx=m_bbx.getBBXValue();

    //clip
    static GLdouble leftClip[] =   { 1,  0 , 0, -bbx[0]+0.05};
    static GLdouble rightClip[] =  {-1,  0 , 0,  bbx[1]+0.05};
    static GLdouble bottomClip[] = { 0,  1 , 0, -bbx[2]+0.05};
    static GLdouble topClip[] =    { 0, -1 , 0,  bbx[3]+0.05};
    static GLdouble backClip[] =   { 0,  0 , 1, -bbx[4]+0.05};
    static GLdouble frontClip[] =  { 0,  0 ,-1,  bbx[5]+0.05};

    glClipPlane( GL_CLIP_PLANE0, leftClip);
    glClipPlane( GL_CLIP_PLANE1, rightClip);
    glClipPlane( GL_CLIP_PLANE2, bottomClip);
    glClipPlane( GL_CLIP_PLANE3, topClip);
    glClipPlane( GL_CLIP_PLANE4, backClip);
    glClipPlane( GL_CLIP_PLANE5, frontClip);
    
    glEnable(GL_CLIP_PLANE0);
    glEnable(GL_CLIP_PLANE1);
    glEnable(GL_CLIP_PLANE2);
    glEnable(GL_CLIP_PLANE3);
    glEnable(GL_CLIP_PLANE4);
    glEnable(GL_CLIP_PLANE5);
#endif
}

void glDrawBoundingBox::unClip()
{
#if GL_ON
    glDisable(GL_CLIP_PLANE0);
    glDisable(GL_CLIP_PLANE1);
    glDisable(GL_CLIP_PLANE2);
    glDisable(GL_CLIP_PLANE3);
    glDisable(GL_CLIP_PLANE4);
    glDisable(GL_CLIP_PLANE5);
#endif
}


int glDrawBoundingBox::BuildBBXModel(const float bbx[6],const string& text)
{
#if GL_ON
    if( !text.empty() && m_texture_id<=0 ){
        m_texture_id = CreateTexture(text);
    }

    GLfloat vertice[]=
    { bbx[0], bbx[2], bbx[4],
      bbx[1], bbx[2], bbx[4],
      bbx[1], bbx[2], bbx[5],
      bbx[0], bbx[2], bbx[5],
      bbx[0], bbx[3], bbx[4],
      bbx[1], bbx[3], bbx[4],
      bbx[1], bbx[3], bbx[5],
      bbx[0], bbx[3], bbx[5]};

    //Face index
    GLubyte id1[] = { 3, 2, 1, 0 }; //buttom
    //GLubyte id2[] = { 4, 5, 6, 7 }; //top
    GLubyte id3[] = { 2, 6, 5, 1 }; //left
    GLubyte id4[] = { 0, 4, 7, 3 }; //right
    GLubyte id5[] = { 1, 5, 4, 0 }; //back
    GLubyte id6[] = { 7, 6, 2, 3 }; //front
    //line index
    /*
    GLubyte lineid[] = 
    { 0, 1, 1, 2, 2, 3, 3, 0,
      4, 5, 5, 6, 6, 7, 7, 4,
      0, 4, 1, 5, 2, 6, 3, 7};
	*/
	
    //texture
    GLfloat txtcoord[]=
    { 0,1, 1,1 ,1,0, 0,0};

    //set properties for this box
    int id = glGenLists(1);
    glNewList(id, GL_COMPILE);

    if(m_texture_id>0){
        glBindTexture(GL_TEXTURE_2D,m_texture_id);
        glEnable(GL_TEXTURE_2D);
    }
    else glDisable(GL_TEXTURE_2D);

#ifdef _WIN32
    glEnable( GL_POLYGON_OFFSET_FILL );
    glPolygonOffset( 2.0, 2.0 );
#endif

    //setup points
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    glVertexPointer(3, GL_FLOAT, 0, vertice);
    glTexCoordPointer(2, GL_FLOAT, 0, txtcoord);

    if(m_texture_id>0) glColor3d( 1,1,1 );
    else glColor3f( 0.8f, 0.8f, 0.6f );

    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glDrawElements( GL_QUADS, 4, GL_UNSIGNED_BYTE, id1 );
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

//    glColor3d( 0.76, 0.76, 0.56 );
//    glDrawElements( GL_QUADS, 4, GL_UNSIGNED_BYTE, id2 );

    glColor3d( 0.65, 0.65, 0.45 );
    glDrawElements( GL_QUADS, 4, GL_UNSIGNED_BYTE, id3 );
    glDrawElements( GL_QUADS, 4, GL_UNSIGNED_BYTE, id4 );

    glColor3d( 0.5, 0.5, 0.3 );
    glDrawElements( GL_QUADS, 4, GL_UNSIGNED_BYTE, id5 );
    glDrawElements( GL_QUADS, 4, GL_UNSIGNED_BYTE, id6 );

#ifdef _WIN32
    glDisable( GL_POLYGON_OFFSET_FILL );
    
    //Draw lines
    glColor3d( 0.4, 0.4, 0.2 );
    //glDrawElements( GL_LINES, 24, GL_UNSIGNED_BYTE, lineid );
#endif
    glEndList();

    return id;
#endif
    return 0;
}

int glDrawBoundingBox::BuildGroundModel(const float bbx[6],const string& text)
{
#if GL_ON
    if( !text.empty() && m_texture_id<=0 ){
        m_texture_id = CreateTexture(text);
    }

    GLfloat vertice[]=
    { bbx[0], bbx[2], bbx[4],
      bbx[1], bbx[2], bbx[4],
      bbx[1], bbx[2], bbx[5],
      bbx[0], bbx[2], bbx[5]};

    //Face index
    GLubyte id1[] = { 3, 2, 1, 0 }; //buttom

    //line index
    //GLubyte lineid[] = { 0, 1, 1, 2, 2, 3, 3, 0};

    //texture
    GLfloat txtcoord[]=
    { 0,1, 1,1 ,1,0, 0,0};

    //set properties for this box
    int id = glGenLists(1);
    glNewList(id, GL_COMPILE);

    if(m_texture_id>0){
        glBindTexture(GL_TEXTURE_2D,m_texture_id);
        glEnable(GL_TEXTURE_2D);
    }
    else glDisable(GL_TEXTURE_2D);

    glEnable( GL_POLYGON_OFFSET_FILL );
    glPolygonOffset( 2.0, 2.0 );

    //setup points
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, vertice);
    glTexCoordPointer(2, GL_FLOAT, 0, txtcoord);

    if(m_texture_id>0) glColor3d( 1,1,1 );
    else glColor3d( 0.6, 0.6, 0.6 );
    glDrawElements( GL_QUADS, 4, GL_UNSIGNED_BYTE, id1 );
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);


    glDisable( GL_POLYGON_OFFSET_FILL );
    
    //Draw lines
    //glColor3f( 0.4f, 0.4f, 0.2f );
    //glDrawElements( GL_LINES, 8, GL_UNSIGNED_BYTE, lineid );




    //draw a larger ground
    float scale=10;
    float x_mid=(bbx[0]+bbx[1])/2;
    float z_mid=(bbx[4]+bbx[5])/2;
    float x_min=(bbx[0]-x_mid)*scale+x_mid;
    float x_max=(bbx[1]-x_mid)*scale+x_mid;
    float z_min=(bbx[4]-z_mid)*scale+z_mid;
    float z_max=(bbx[5]-z_mid)*scale+z_mid;

    glColor3f(0.90f,0.90f,0.90f);
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(x_min, bbx[2]-1, z_min);
    glNormal3f(0, 1, 0);
    glVertex3f(x_min, bbx[2]-1, z_max);
    glNormal3f(0, 1, 0);
    glVertex3f(x_max, bbx[2]-1, z_max);
    glNormal3f(0, 1, 0);
    glVertex3f(x_max, bbx[2]-1, z_min);
    glEnd();


    glEndList();

    return id;
#endif
    return 0;
}

