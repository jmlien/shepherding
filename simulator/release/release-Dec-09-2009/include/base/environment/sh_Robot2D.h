#ifndef _ROBOT2D_H_
#define _ROBOT2D_H_

#include "sh_PolyhedronModel.h"
#include <Point.h>

#include "RAPID.H"

//////////////////////////////////////////////////////////////////////////
class CRobot2D : public CPolyhedronModel
{
public:
    //////////////////////////////////////////////////////////////////////
    // Constructor/Destructor
    CRobot2D();
    virtual ~CRobot2D();

    //////////////////////////////////////////////////////////////////////
    // Core
    void doDraw(bool b_Show3D);

    //construct collision detection models
    virtual bool buildCDModel(IModel& model);
    virtual bool buildGLModel(IModel& model);
    bool buildCDLine( const Point2d& p1, const Point2d& p2);

    //////////////////////////////////////////////////////////////////////
    // Access
    float getHeight(){ return m_Height; }
    float getRadius(){ return m_Radius; }
    void setColor( float R, float G, float B ){ m_Color.set(R,G,B); }
    const Vector3d & getColor( ){ return m_Color; }

    RAPID_model & getCDModel(){ return m_Poly2D; }
    RAPID_model * getCDLine() { return m_Line2D; }

protected:

    void extractInfo(IModel& model);
    bool buildGL2D();
    bool buildPQP2D();

private:
    float m_Height;
    float m_Radius;
    Vector2d m_Center;
    int m_DisplayList; //its for ground
    Vector3d m_Color;

    RAPID_model m_Poly2D;
    RAPID_model * m_Line2D;

    //catched projected geo/topo data
    PtVector  m_Geo;
    TriVector m_Tri;
};

#endif


