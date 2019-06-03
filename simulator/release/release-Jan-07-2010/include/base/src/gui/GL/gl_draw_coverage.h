#ifndef _SH_GL_DRAW_COVERAGE_H_
#define _SH_GL_DRAW_COVERAGE_H_

#include "gl_draw.h"
#include "shepherding_base.h"

//draw the coverage
class CDrawCoverage : public gl_Draw
{
public:

    CDrawCoverage
    (CEnvironment * env, CFlock * flock, int xsize, int zsize,
     float H=1,float R=0,float G=0,float B=0,float A=0.5);

    ~CDrawCoverage()
    {delete [] m_Colors; delete [] m_Points; delete [] m_Index;}

    virtual void draw();
    void update(const Point2d& pos, float value,float vr=0);
    float coverageRate(); //0~1

private:
    
    CEnvironment * m_pEnv;
    CFlock * m_pFlock;
    int m_xsize;
    int m_zsize;
    float * m_Colors;
    float * m_Points;
    unsigned int * m_Index;
    float m_xres;
    float m_zres;
    float m_TotalPoints;
};

#endif //_SH_DRAW_COVERAGE_H_
