// PolyhedronModel.h: interface for the CPolyhedronModel class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _GL_DRAW_POLYHEDRAONMODEL_H_
#define _GL_DRAW_POLYHEDRAONMODEL_H_

#include "gl_draw.h"
#include "model/IModel.h"


//a class construct a polyhedron body
class gl_draw_PolyhedronModel: public gl_Draw
{
public:

	gl_draw_PolyhedronModel();
	
    //////////////////////////////////////////////////////////////////////
    // Core
    virtual void draw();

    //methods for building GL models
    virtual bool buildGLModel(IModel& model);

    //////////////////////////////////////////////////////////////////////
    // Access
    int getDisplayID() const { return m_DisplayListID; }
    void setTexture( int texid ){ m_TextureID=texid; }
	void setColor(const Point3d& c){ m_color=c; }
	
//////////////////////////////////////////////////////////////////////
// Protected and Private
protected:
    
    typedef IModel::PtVector  PtVector;
    typedef IModel::TriVector TriVector;
    typedef IModel::Tri       Tri;
    typedef IModel::V3Vcetor  V3Vcetor;
    typedef IModel::V2Vcetor  V2Vcetor;

    bool buildGLModel_solid(IModel& model);
    bool buildGLModel_wire(IModel& model);

    //////////////////////////////////////////////////////////////////////
    //Data for OpenGL model
    int m_DisplayListID;       //display id for GL
    int m_WiredID;             //display id for GL in wire frame
    int m_TextureID;           //id of the texture
    Point3d m_color;
};

#endif // _GL_DRAW_POLYHEDRAONMODEL_H_

