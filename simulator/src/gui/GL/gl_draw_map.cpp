#include "gl_draw_map.h"
#include "sh_FlockState.h"

void glDrawRoadMap::draw()
{
    float radius=1;
    if( m_rmap.getFlock()!=NULL ) 
        radius=m_rmap.getFlock()->getGeometry().getRadius();
        
    glDisable(GL_LIGHTING);

    ///////////////////////////////////////////////////////////////////////////
    glLineWidth(2);
    glColor3fv(m_color.get());
    ///////////////////////////////////////////////////////////////////////////
    //draw nodes
    vector<CNodeData*>& nodes=m_rmap.getNodes();
    typedef vector<CNodeData*>::iterator NIT;
    {for(NIT i=nodes.begin();i!=nodes.end();i++){
        if( *i==NULL ) continue;
        const Point2d& pos=(*i)->getPos();
        glPushMatrix();
        glTranslated(pos[0],m_height,pos[1]);
        drawCircle(0.25,PI2,true);
        glPopMatrix();
        if( m_b_ShowID ) drawID(pos,(*i)->getID());
    }}

    ///////////////////////////////////////////////////////////////////////////
    //draw edges
    glPushAttrib(GL_CURRENT_BIT);
    vector< pair<VID,VID> > edges;
    typedef vector< pair<VID,VID> >::iterator EIT;
    m_rmap.getGraph().GetEdges(edges);
    glBegin(GL_LINES);
    {for(EIT i=edges.begin();i!=edges.end();i++){
        const Point2d& p1=m_rmap.getNode(i->first)->getPos();
        const Point2d& p2=m_rmap.getNode(i->second)->getPos();
        glVertex3d(p1[0],m_height,p1[1]);
        glVertex3d(p2[0],m_height,p2[1]);
    }}//end for
    glEnd();
}

void glDrawRoadMap::drawID(const Point2d& pos,int id)
{
    char value[32];
    sprintf(value,"%d",id);
    glPushAttrib(GL_CURRENT_BIT);
    glColor3d(0.2,0.1,0.1);
    drawstr(pos[0],m_height+1,pos[1],value);
    glPopAttrib();
}

