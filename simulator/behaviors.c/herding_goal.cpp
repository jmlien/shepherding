/*
 * simple_herding_goal.cpp
 *
 * implements simple_herding_goal.h
 * 
 * Created on: Mar 08, 2010
 *
 */
 

#include "herding_goal.h"

void CSimpleHerdingGoal::build(const string& filename)
{
    //read in data
    IModel * model=CreateModelLoader(filename);

    PtVector& pts=model->GetVertices();
    m_Tri=model->GetTriP();
	
    //projected into xz plane & find center in xz plane
	m_Center.set(0,0);
    typedef PtVector::iterator PIT;
    {for( PIT ip=pts.begin();ip!=pts.end();ip++ ){
        const Point3d& pt=*ip;
        m_Center[0]+=pt[0]; m_Center[1]+=pt[2]; 
		m_Geo.push_back(Point2d(pt[0],pt[2]));
    }}
    for(int i=0;i<3;i++) m_Center[i]/=pts.size();
	
    //compute radius
    m_Radius=0;
    {for( PIT ip=pts.begin();ip!=pts.end();ip++ ){
        Point3d& pt3d=*ip;
        Point2d pt(pt3d[0],pt3d[2]);
        float r=(pt-m_Center).normsqr();
        if(r>m_Radius) m_Radius=r;
    }}
    m_Radius=sqrt(m_Radius);
	
	//done
	delete model;
}
    
bool CSimpleHerdingGoal::isInGoal(const Point2d& pos)
{
    
    if( (m_Center-pos).normsqr()>m_Radius*m_Radius ){
        return false; //too far
    }
    
    if(m_Tri.empty() || m_Geo.empty()){ return true; }
    
    //for each polygon
    int tsize=(int)m_Tri.size();
    for( int iT =0; iT<tsize ; iT ++ )
    {
        int vid1=(int)m_Tri[iT][0], 
            vid2=(int)m_Tri[iT][1], 
            vid3=(int)m_Tri[iT][2];

        Point2d& p1=m_Geo[vid1];
        Point2d& p2=m_Geo[vid2];
        Point2d& p3=m_Geo[vid3];
        
        //check if pos is in this triangle
        Vector2d v1=p2-p1; 
        Vector2d v2=p3-p2; 
        Vector2d v3=p1-p3; 
        
        Vector2d u1=pos-p1;
        Vector2d u2=pos-p2; 
        Vector2d u3=pos-p3;
        
        float dot1=u1[1]*v1[0]-u1[0]*v1[1];
        float dot2=u2[1]*v2[0]-u2[0]*v2[1];
        float dot3=u3[1]*v3[0]-u3[0]*v3[1];
        
        if(dot1*dot2>0 && dot1*dot3>0) return true;
    }
    
    return false;
}

