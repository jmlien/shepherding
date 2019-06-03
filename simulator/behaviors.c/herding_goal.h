/*
 * simple_herding_goal.h
 *
 * Define herding goal 
 *  - this can be a cirle (created via build_circle)
 *  - this can also be an arbitrary polygon (create via build)
 * 
 * Created on: Mar 08, 2010
 *
 */
 
 
#ifndef SIMPLE_HERDING_GOAL_H
#define SIMPLE_HERDING_GOAL_H

#include "shepherding_base.h"

class CSimpleHerdingGoal
{
    typedef IModel::PtVector  PtVector;
    typedef IModel::TriVector TriVector;
    typedef IModel::Tri       Tri;
    
public:
    
    void build(const string& filename);
    
    bool isInGoal(const Point2d& pos);
    
    //access
    const vector<Point2d>&  getPoints() const { return m_Geo; }
    const TriVector& getTriangles() const { return m_Tri; } 
    const Point2d& getPosition() const { return m_Position; }
    float getRadius() {return m_Radius;}
    
    void setPosition(const Point2d& p) {m_Position=p;} 
    void setCenter(const Point2d& c) {m_Center=c;} 
    void setRadius(float r) {m_Radius=r;}
    
private:

    //catched projected geo/topo data
    vector<Point2d>  m_Geo;
    TriVector        m_Tri;
    
    //
    float m_Radius;
    Point2d m_Center;
    Point2d m_Position;
};


#endif //SIMPLE_HERDING_GOAL_H


