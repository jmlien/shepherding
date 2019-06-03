#ifndef _OBS2D_H_
#define _OBS2D_H_

#include "sh_PolyhedronModel.h"
#include "RAPID.H"

#include <vector>
using namespace std;

//////////////////////////////////////////////////////////////////////////
class CRobot2D;
class CObs2D : public CPolyhedronModel
{
public:
    //////////////////////////////////////////////////////////////////////
    // Constructor/Destructor
    CObs2D();
    virtual ~CObs2D();

    //////////////////////////////////////////////////////////////////////
    // Core
    //construct collision detection models
    virtual bool buildCDModel(IModel& model);

    //////////////////////////////////////////////////////////////////////
    // Access
    void setColor( float R, float G, float B ){ m_Color.set(R,G,B); }

    //////////////////////////////////////////////////////////////////////
    RAPID_model & getCDModel(){ return m_Poly2D; }
    const PtVector& getGeo() const { return m_Geo; }
    const TriVector& getTri() const { return m_Tri; }
    const TriVector& getLine() const { return m_Line; }

    //////////////////////////////////////////////////////////////////////
    float getRadius() const { return m_Radius; }
    const Vector2d & getCenter() const { return m_Center; }
    float getHeight(){ return m_Height; }
    //////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Protected
protected:

    ///////////////////////////////////////////////////////////////////////////
    bool buildPQP2D();
    ///////////////////////////////////////////////////////////////////////////
    void extractInfo(IModel& model);
    void findUsefulTri(PtVector& pts, TriVector& tri);

//////////////////////////////////////////////////////////////////////
// Private
private:
    float m_Height;
    float m_Radius;
    Vector2d m_Center;
    Vector3d m_Color;

    RAPID_model m_Poly2D;

    //catched projected geo/topo data
    PtVector  m_Geo;
    TriVector m_Line;
    TriVector m_Tri;
};

#endif

