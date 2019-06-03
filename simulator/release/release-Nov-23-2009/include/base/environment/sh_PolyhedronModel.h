// PolyhedronModel.h: interface for the CPolyhedronModel class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(_MULTIPOLYHEDRONMODEL_H_)
#define _MULTIPOLYHEDRONMODEL_H_

#include "model/IModel.h"

#include <Vector.h>
#include <Matrix.h>
using namespace mathtool;

//a class construct a polyhedron body
class CPolyhedronModel  
{
public:

    //////////////////////////////////////////////////////////////////////
    // Constructor/Destructor
    CPolyhedronModel();
    virtual ~CPolyhedronModel();

    //////////////////////////////////////////////////////////////////////
    // Core
    virtual void doDraw(Vector3d & color);

    //methods for building GL models
    virtual bool buildGLModel(IModel& model);

    //////////////////////////////////////////////////////////////////////
    // Access
    int getDisplayID() const { return m_DisplayListID; }
    void setPos(float X, float Y, float Z);
    const Point3d& getPos() const { return m_Pos; }

    void setRot(float rX, float rY, float rZ);
    void setRot(float * R);
    const Matrix3x3 & getRot() const { return m_Rot; }

    void setTexture( int texid ){ m_TextureID=texid; }
    void setScale( float scale ){ m_Scale=scale; }
//////////////////////////////////////////////////////////////////////
// Protected and Private
protected:
    
    typedef IModel::PtVector  PtVector;
    typedef IModel::TriVector TriVector;
    typedef IModel::Tri       Tri;
    typedef IModel::V3Vcetor  V3Vcetor;
    typedef IModel::V2Vcetor  V2Vcetor;

    bool buildGLModel_solid( IModel& model );
    bool buildGLModel_wire(IModel& model);

    //////////////////////////////////////////////////////////////////////
    //Data for OpenGL model
    int m_DisplayListID;       //display id for GL
    int m_WiredID;             //display id for GL in wire frame
    int m_TextureID;           //id of the texture
    //////////////////////////////////////////////////////////////////////
    //Data for polyhedron state.
    Point3d m_Pos;
    Matrix3x3 m_Rot;
    float m_Scale;
};

#endif // !defined(_MULTIPOLYHEDRONMODEL_H_)

