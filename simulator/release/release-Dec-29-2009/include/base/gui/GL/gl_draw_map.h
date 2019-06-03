#ifndef _SH_GL_DRAW_MAP_H_
#define _SH_GL_DRAW_MAP_H_

#include "gl_draw.h"
#include "sh_Roadmap.h"


class glDrawRoadMap : public gl_Draw 
{
public:

    glDrawRoadMap(CRoadMap& map, bool b_ShowID=false):m_rmap(map){ m_b_ShowID=b_ShowID; }
    virtual void draw();
    
protected:

    void drawID(const Point2d& pos,int id);
    
    CRoadMap& m_rmap;
    bool m_b_ShowID;
};

#endif //_SH_GL_DRAW_MAP_H_
