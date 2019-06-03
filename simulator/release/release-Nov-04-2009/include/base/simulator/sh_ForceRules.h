#ifndef _SH_FORCE_RULES_H_
#define _SH_FORCE_RULES_H_

#include <Vector.h>
using namespace mathtool;
class CFlockState;

///////////////////////////////////////////////////////////////////////////////
class CForceRule
{
public:
    virtual Vector2d getForce(CFlockState& s)=0;
};

///////////////////////////////////////////////////////////////////////////////
class CBasicForceRule : public CForceRule
{
public:
    ///////////////////////////////////////////////////////////////////////////
    CBasicForceRule();
    CBasicForceRule(const CBasicForceRule& other);

    ///////////////////////////////////////////////////////////////////////////
    virtual Vector2d getForce(CFlockState& s);

    ///////////////////////////////////////////////////////////////////////////
    void setCohesion(float v){ m_Cohesion=v; }
    void setSeparation(float v){ m_Separation=v; }
    void setAlignment(float v){ m_Alignment=v; }
    void setObstRepulsion(float v){ m_ObstRepulsion=v; }
    void setMaxForce(float v){ m_Max_Force=v; }
    void setDampingFactor(float v){ m_Dampling=v; }
    float getCohesion() const { return m_Cohesion; }
    float getSeparation() const { return m_Separation; }
    float getAlignment() const { return m_Alignment; }
    float getObstRepulsion() const { return m_ObstRepulsion; }
    float getMaxForce() const { return m_Max_Force; }
    float getDampingFactor() const { return m_Dampling; }
///////////////////////////////////////////////////////////////////////////////
private:
    bool truncate(Vector2d& f1, const Vector2d& f2);

    float m_Cohesion;
    float m_Separation;
    float m_Alignment;
    float m_ObstRepulsion;
    float m_Max_Force;
    float m_Dampling;
};
///////////////////////////////////////////////////////////////////////////////

#endif //_SH_FORCE_RULES_H_


