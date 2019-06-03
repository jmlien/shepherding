#include "GL/gli.h"
#include "sh_Obs2D.h"
#include <cassert>
#include <Point.h>

CObs2D::CObs2D()
{
    m_Height=0;
    m_Radius=0;
}

CObs2D::~CObs2D()
{
}

//////////////////////////////////////////////////////////////////////
// Core

bool CObs2D::buildCDModel(IModel& model)
{
    if( m_Geo.empty() )
        extractInfo(model);

    return  buildPQP2D();
}

//////////////////////////////////////////////////////////////////////
// Protected:

void CObs2D::extractInfo(IModel& model)
{
    PtVector& pts=model.GetVertices();
    TriVector& tri=model.GetTriP();

    //projected into xz plane & find center in xz plane
    m_Height=0;
    typedef PtVector::iterator PIT;
    {for( PIT ip=pts.begin();ip!=pts.end();ip++ ){
        const Point3d& pt=*ip;
        m_Center[0]+=pt[0]; m_Center[1]+=pt[2]; 
        if(m_Height<pt[1]) m_Height=pt[1];
    }}
    m_Center=m_Center/pts.size();

    //compute radius
    m_Radius=0;
    {for( PIT ip=pts.begin();ip!=pts.end();ip++ ){
        Point3d& pt3d=*ip;
        Vector2d pt(pt3d[0],pt3d[2]);
        float r=(pt-m_Center).normsqr();
        if(r>m_Radius) m_Radius=r;
        pt3d[0]-=m_Center[0];
        pt3d[2]-=m_Center[1];
    }}
    m_Radius=sqrt(m_Radius);
    m_Geo=pts;

    //Build projected PQP and GL model
    findUsefulTri(pts,tri);
}

void CObs2D::findUsefulTri(PtVector& pts, TriVector& tris)
{
    typedef TriVector::iterator TIT;
    m_Tri.reserve(tris.size());
    {for(TIT iT=tris.begin(); iT!=tris.end();iT++){
        const Tri& tri=*iT;
        
        if(tri[0]==tri[1] && tri[0]==tri[2]) continue; //??
        
        const Point3d& p1=pts[tri[0]];
        const Point3d& p2=pts[tri[1]];
        const Point3d& p3=pts[tri[2]];

        if( p1[1]>0.1 && p2[1]>0.1 && p3[1]>0.1 )
            continue; //too high

        //compute normal, backface cull
        Vector3d v=((p2-p1)%(p3-p1));
        if( v[1]>=0 ) continue;

        //compute the area of projected triangle
        Vector2d v_p2p1(p2[0]-p1[0],p2[2]-p1[2]);
        Vector2d v_p3p1(p3[0]-p1[0],p3[2]-p1[2]);
        float area=fabs(v_p2p1[0]*v_p3p1[1]-v_p2p1[1]*v_p3p1[0]);

        if( area<1e-3 ) continue; //too small

        m_Tri.push_back(tri);
    }}

    //Get Boundary lines

    vector< Vector<int,2> > lines; lines.reserve(m_Tri.size()*3);
    {for(TIT iT=m_Tri.begin(); iT!=m_Tri.end();iT++){
        const Tri& tri=*iT;
        lines.push_back(Vector<int,2>(tri[0],tri[1]));
        lines.push_back(Vector<int,2>(tri[1],tri[2]));
        lines.push_back(Vector<int,2>(tri[2],tri[0]));
    }}


    typedef vector< Vector<int,2> >::iterator LIT;
    {for(TIT iT=m_Tri.begin(); iT!=m_Tri.end();iT++){
        Tri tri_line;
        Vector<int,2> line;
        const Tri& tri=*iT;

        for( int iD=0; iD<3; iD++ ){
            switch(iD){
                case 0: line.set(tri[0],tri[1]); break;
                case 1: line.set(tri[1],tri[2]); break;
                case 2: line.set(tri[2],tri[0]); break;
            }
            int count=0;
            
            for( LIT iL=lines.begin(); iL!=lines.end(); iL++ ){
                Vector<int,2>& l=*iL;
                if((l[0]==line[0]&&l[1]==line[1])||(l[0]==line[1]&&l[1]==line[0]))
                    count++;
            }
            if( count==1 ) tri_line[iD]=1;
            else tri_line[iD]=0;
        }
        m_Line.push_back(tri_line);
    }}
}

bool CObs2D::buildPQP2D()
{
    //create PQP MODEL
    m_Poly2D.BeginModel();

    //for each polygon
    int size=m_Tri.size();
    for( int iT=0; iT<size; iT ++ )
    {
        int vid1=(int)m_Tri[iT][0], vid2=(int)m_Tri[iT][1], vid3=(int)m_Tri[iT][2];
        double p1[3]={m_Geo[vid1][0],0,m_Geo[vid1][2]};
        double p2[3]={m_Geo[vid2][0],0,m_Geo[vid2][2]};
        double p3[3]={m_Geo[vid3][0],0,m_Geo[vid3][2]};
        
        //copy to PQP_REAL
        m_Poly2D.AddTri(p1, p2, p3, iT);
    }

    m_Poly2D.EndModel();
    return true;
}


