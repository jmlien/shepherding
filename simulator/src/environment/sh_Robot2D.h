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
    //construct collision detection models
    bool buildCDModel(IModel& model); //Create enclosing circle as the projection
    bool buildTrueCDModel(IModel& model); //Create true projection
    bool buildCompleteCDModel(IModel& model);//Create 3D model without projection..
    bool buildCDLine( const Point2d& p1, const Point2d& p2);

    //////////////////////////////////////////////////////////////////////
    // Access
    float getHeight(){ return m_Height; }
    float getRadius(){ return m_Radius; }
    void setColor( float R, float G, float B ){ m_Color.set(R,G,B); }
    const Vector3d & getColor( ){ return m_Color; }
    
    PtVector& getGeo(){ return m_Geo; }
    TriVector& getTri(){ return m_Tri;}
    
    RAPID_model & getCDModel(){ return m_Poly2D; }
    RAPID_model * getCDLine() { return m_Line2D; }

protected:

    void extractInfo(IModel& model);
    void findUsefulTri(PtVector& pts, TriVector& tris);
    bool buildPQP2D();

private:

    float m_Height;
    float m_Radius;
    Vector2d m_Center;
    Vector3d m_Color;

    RAPID_model m_Poly2D;
    RAPID_model * m_Line2D;

    //catched projected geo/topo data
    PtVector  m_Geo;
    TriVector m_Tri;
};

#endif


