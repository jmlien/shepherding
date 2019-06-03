// PolyhedronModel.cpp: implementation of the gl_draw_PolyhedronModel class.
//
//////////////////////////////////////////////////////////////////////

#include "gl_draw_PolyhedronModel.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////

#include <modelgraph/ModelGraph.h>
using namespace modelgraph;

#include "GL/gli.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
gl_draw_PolyhedronModel::gl_draw_PolyhedronModel()
{
    m_TextureID=m_WiredID=m_DisplayListID=-1;
    //m_Scale=1;
}

//////////////////////////////////////////////////////////////////////
// Core
//////////////////////////////////////////////////////////////////////
void gl_draw_PolyhedronModel::draw()
{
    glColor3fv( m_color.get() );
    if(m_TextureID>0)
        glBindTexture(GL_TEXTURE_2D,m_TextureID);
    else 
        glDisable(GL_TEXTURE_2D);
    glCallList(m_DisplayListID);
}

//////////////////////////////////////////////////////////////////////
// Protected Methods
//
// Build GL & PQP models
//////////////////////////////////////////////////////////////////////

bool 
gl_draw_PolyhedronModel::buildGLModel(IModel& model)
{
    //////////////////////////////////////////////////////////////////
    //Build display list
    m_DisplayListID = glGenLists(1);
    glNewList(m_DisplayListID, GL_COMPILE);
    bool bResult=buildGLModel_solid(model);
    glEndList();
    if( bResult==false ) return false;
    return bResult;
}

bool gl_draw_PolyhedronModel::buildGLModel_solid(IModel& model)
{
    const PtVector& pts=model.GetVertices();
    const TriVector& triP=model.GetTriP();
    const TriVector& triN=model.GetTriN();
    const TriVector& triT=model.GetTriT();
    const V2Vcetor & texure=model.GetTextureCoords();
    V3Vcetor & normals=model.GetNormals();
    bool texture=!texure.empty()&&!triT.empty();
    if(m_TextureID>0 && texture){
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D,m_TextureID);
    }

    glBegin(GL_TRIANGLES);
    int size=triP.size();
    for( int it=0;it<size;it++ ){
        const Tri& tv=triP[it];
        const Tri& tn=triN[it];
        Tri tt;
        if(texture) tt=triT[it];

        for( int i=0;i<3;i++ ){
            if(texture) glTexCoord2f(texure[tt[i]][0],texure[tt[i]][1]);
            glNormal3f(normals[tn[i]][0],normals[tn[i]][1],normals[tn[i]][2]);
            glVertex3f(pts[tv[i]][0],pts[tv[i]][1],pts[tv[i]][2]);
        }
    }

    glEnd();

    return true;
}

//build wire frame
bool gl_draw_PolyhedronModel::buildGLModel_wire(IModel& model)
{   
    const PtVector& pts=model.GetVertices();
    const TriVector& triP=model.GetTriP();

    vector<Vector3d> n; n.reserve(triP.size());
    typedef TriVector::const_iterator TIT;
    for( TIT it=triP.begin();it!=triP.end();it++ ){
        const Tri& tri=*it;

        const Point3d& p1=pts[tri[0]];
        const Point3d& p2=pts[tri[1]];
        const Point3d& p3=pts[tri[2]];
        Vector3d normal=((p2-p1)%(p3-p1)).normalize();
        n.push_back(normal);
    }

    CModelGraph mg;
    if( mg.doInit(pts,triP)==false ) return false;
    
    //build model    
    glBegin(GL_LINES);
    const CModelEdge * edge=mg.getEdges();
    while(edge!=NULL){
        int tri_l=edge->getLeftTri();
        int tri_r=edge->getRightTri();
        
        bool draw=false;
        if( tri_l==-1 || tri_r==-1 ) draw=true;
        else if( (1-fabs(n[tri_l]*n[tri_r]))>1e-3 ) draw=true;
        
        if( draw ){
            Vector3d normal=(n[tri_l]+n[tri_r]).normalize();
            glNormal3f(normal[0],normal[1],normal[2]);
            const Point3d& p1=pts[edge->getStartPt()];
            const Point3d& p2=pts[edge->getEndPt()];
            glVertex3f(p1[0],p1[1],p1[2]);
            glVertex3f(p2[0],p2[1],p2[2]);
        }
        edge=edge->getNext();
    }
    glEnd();
 
    return true;
}


