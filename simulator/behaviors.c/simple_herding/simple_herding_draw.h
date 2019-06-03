#ifndef _SIMPLE_HERDING_DRAW_H_
#define _SIMPLE_HERDING_DRAW_H_

#include "shepherding.h"
#include "shepherding_base.h"

#include "simple_herding.h"
#include "herding_goal.h"

#include "GL/gl_draw.h"
#include "GL/gl_draw_map.h"
#include "GL/gl_draw_path.h"

//
// draw goal position
//
//

class glDrawGoal : public gl_Draw
{

public:

    CSimpleHerdingGoal goal;
    string message;
    float r,g,b;

    glDrawGoal(){
        r=1;
        g=b=0;
        //message="Steer Flock Here";
    }

    void draw()
    {   
        //draw goal region
        glEnable(GL_LIGHTING);
        
        const vector<Point2d>&  pts=goal.getPoints();
        const IModel::TriVector & tri=goal.getTriangles();
        const Point2d& center=goal.getPosition();
        float radius=goal.getRadius();
        
        if(tri.empty())
        {           
            if(radius>0){
                glPushMatrix();
                glTranslated(center[0], 1, center[1]);
                glColor3f(r,g,b); 
                drawCircle(radius, PI2, false); //empty circle
                glPopMatrix();
            }
        }
        else
        {   
            glBegin(GL_TRIANGLES);
            glColor3f(r,g,b); 
            glNormal3f(0,1,0);
            int tsize=(int)tri.size();
            
            for( int iT =0; iT<tsize ; iT ++ )
            {
                int vid1=(int)tri[iT][0], 
                    vid2=(int)tri[iT][1], 
                    vid3=(int)tri[iT][2];
                 glVertex3f(pts[vid1][0],-1,pts[vid1][1]);
                 glVertex3f(pts[vid2][0],-1,pts[vid2][1]);
                 glVertex3f(pts[vid3][0],-1,pts[vid3][1]);
             }
             glEnd();
        }

        //draw a ball on a bar 
        glPushMatrix();
            glTranslated(center[0], 3, center[1]);
            glColor3d(r,g,b); 
            glutSolidSphere(0.5,10,10);
        glPopMatrix();
        
        glDisable(GL_LIGHTING);        
        glBegin(GL_LINES);
            glVertex3d(center[0], 3,center[1]);
            glVertex3d(center[0], -0.9, center[1]);
        glEnd();

        sh_drawString(Point3d(center[0],5.5,center[1]),message);
    }
    //end draw
};


//
// draw groups
//

class glDrawGroups : public gl_Draw
{
public:

    CSimpleHerdingFlock * flock;
    
    glDrawGroups(CSimpleHerdingFlock * f)
    {
        flock=f;
    }
    
    void draw()
    {
        if(flock==NULL) return;
        
        //compute center and radius
        vector<Point2d> centers;
        vector<float>  radius;
        list<FSLIST>::iterator i=flock->groups.begin();
        for(;i!=flock->groups.end();i++) { 
            pair<float,Point2d> disc=findEC(*i);
            radius.push_back(disc.first);
            centers.push_back(disc.second);
        }
        
        glDisable(GL_LIGHTING);
        int size=centers.size();
        for(int i=0;i<size;i++){
            glPushMatrix();
            glColor3d(1,1,0);
            glLineWidth(1);
            glTranslated(centers[i][0],0,centers[i][1]);
            drawCircle(radius[i], PI2);
            glColor3d(1,1,0);
            drawCircle(radius[i]/50, PI2,true);
            glPopMatrix();
        }
    } //end draw
};


//
// draw roadmap
//
/*
class glDrawMap : public gl_Draw
{
public:

    CRoadMap * rmap;
    int gid;
    Point3d color;

    glDrawMap(){
        rmap=NULL;
        gid=-1;
        color.set(0.3f,0.2f,0);
    }
    

    void draw(){
#if GL_ON
        #ifdef NO_DRAW_GUIDES
        return;
        #endif
        glLineWidth(1);
        glDisable(GL_LIGHTING);
        glColor3fv(color.get());
        drawMap(*rmap);
#endif
    }
};
*/


//
// draw controlled group's path
//

class glDrawMovingPath : public gl_Draw
{
public:

    CSimpleHerdingFlock * flock;
    Point3d color;
    
    glDrawMovingPath(CSimpleHerdingFlock * f)
    {
        color.set(0.5,0,0);
        flock=f;
    }

    void draw()
    {
        if(flock==NULL) return;
        glDisable(GL_LIGHTING);
        glColor3fv(color.get());
        glDrawPath drawpath(flock->path,0.2f);
        drawpath.draw();
    }
};

//
// draw milestones
//

class glDrawMilestones : public gl_Draw
{
public:

    CSimpleHerdingFlock * flock;
    vector< list<Point2d> > catched_targets;
    const unsigned int max_catch_size;
    
    glDrawMilestones(CSimpleHerdingFlock * f)
    : max_catch_size(50)
    {
        flock=f;
    }
    
    void draw()
    {
        if(flock==NULL) return;
        int stat_size=flock->getStateSize();
        for(int k=0;k<stat_size;k++)
        {
            if((int)catched_targets.size()<=k) catched_targets.push_back(list<Point2d>());
            list<Point2d> & path=catched_targets[k];
            CHerdingFlockState& s=(CHerdingFlockState&)flock->getState(k);;
            path.push_back(s.target);
            while( path.size()>max_catch_size ) path.pop_front();
    
            glDisable(GL_LIGHTING);
            glColor3d(0,0.5,0);
    
            int count=0;
            for(list<Point2d>::reverse_iterator i=path.rbegin();i!=path.rend();i++,count++){
                const Point2d& pt=*i;
                glPushMatrix();
                glColor4f(0,1,0,1-(count*1.0)/max_catch_size);
                glLineWidth(1);
                glTranslated(pt[0],0,pt[1]);
                drawCircle(0.2, PI2, true); //filled circle
                glPopMatrix();
            }//end for i
        }//end for k
    }

};

//
// draw trace
//
/*
class glDrawTrace : public gl_Draw
{
    list<Point2d> trace;
    Point3d color;

    glDrawTrace()
    {
        color.set(0.2f,0.2f,0.2f);
    }

    void draw()
    {
#if GL_ON
        #ifdef NO_DRAW_GUIDES
        return;
        #endif
        if( trace.empty() ) return; //nothing to show

        glDisable(GL_LIGHTING);
        glLineWidth(1);

        Point2d last_pt=trace.front();
        glColor3fv(color.get());
        glVertex2d(last_pt[0],last_pt[1]);
        
        //draw line
        glBegin(GL_LINE_STRIP);
        for(list<Point2d>::iterator i=++trace.begin();i!=trace.end();i++)
        {
            const Point2d& pt=*i;
            glColor3fv(color.get());
            if((last_pt-pt).normsqr()>1) glColor3d(1,0,0);
            last_pt=pt;
            glVertex2d(pt[0],pt[1]);
        }
        glEnd();
#endif
    }//end draw
};

//
//draw vector
//


class glDrawDir(glglDraw):

    dir=Vector2d(1,1);

    pos=Point2d(-1000,-1000);

    def draw(self):

        glDisable(GLenum.GL_LIGHTING);

        glLineWidth(2);

        glBegin(GLenum.GL_LINE_STRIP);

        glColor3d(0.1,0.1,0.5);

        glVertex2d(self.pos.x,self.pos.y);

        glVertex2d(self.pos.x+self.dir.x,self.pos.y+self.dir.y);

        glEnd();
*/



#endif //_SIMPLE_HERDING_DRAW_H_

