#ifndef _SH_GL_DRAW_MAP_H_
#define _SH_GL_DRAW_MAP_H_

#include "gl_draw.h"
#include "sh_Roadmap.h"


class glDrawRoadMap : public gl_Draw 
{
public:

    glDrawRoadMap(CRoadMap& map, bool b_ShowID=false):m_rmap(map){ 
        m_b_ShowID=b_ShowID; 
        m_color.set(0,0,0);
        m_height=-0.99;
    }
    virtual void draw();
    
    void setColor(const Point3d& c) { m_color=c; }
    void setHeight(int h) { m_height=h; } //height in Y axis
    
protected:

    void drawID(const Point2d& pos,int id);
    
    CRoadMap& m_rmap;
    bool m_b_ShowID;
    float m_height;
    Point3d m_color;
};

#endif //_SH_GL_DRAW_MAP_H_
