#include "sh_Robot2D.h"

CRobot2D::CRobot2D():CPolyhedronModel()
{
    m_Line2D=NULL;
    m_Height=0;
    m_Radius=0.001f;
}

CRobot2D::~CRobot2D()
{
    delete m_Line2D;
    m_Line2D=NULL;
}

//////////////////////////////////////////////////////////////////////
// Core
bool CRobot2D::buildCDModel(IModel& model)
{
    if( m_Geo.empty() )
        extractInfo(model);

    return  buildPQP2D();
}

bool CRobot2D::buildTrueCDModel(IModel& model)
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
    
	findUsefulTri(pts,tri);
	return this->buildPQP2D();
}

bool CRobot2D::buildCompleteCDModel(IModel& model)
{
    this->m_Geo = model.GetVertices();
    this->m_Tri = model.GetTriP();
    return this->buildPQP2D();
}

bool CRobot2D::buildCDLine
( const Point2d& p1, const Point2d& p2)
{
    //buld very "thin" line
	float R=m_Radius; //(p1-p2).norm()*1e-3;
	if(R<1e-3) R=1e-3;
    Vector2d u=(p2-p1).normalize()*R;
    Vector2d n(u[1], -u[0]);

    //make 4 pts
    double p11[3]={p1[0]+n[0]-u[0],0,p1[1]+n[1]-u[1]};
    double p12[3]={p1[0]-n[0]-u[0],0,p1[1]-n[1]-u[1]};
    double p21[3]={p2[0]+n[0]+u[0],0,p2[1]+n[1]+u[1]};
    double p22[3]={p2[0]-n[0]+u[0],0,p2[1]-n[1]+u[1]};
    
    //make 2 triangles
    if( m_Line2D!=NULL ){ delete m_Line2D; m_Line2D=NULL; }
    
    m_Line2D=new RAPID_model();
    if( m_Line2D==NULL ) return false;
    
    //fill these 2 tris
    m_Line2D->BeginModel();
    m_Line2D->AddTri(p11, p12, p21, 0);
    m_Line2D->AddTri(p21, p22, p12, 1);
    m_Line2D->EndModel();
    return true;
}

//////////////////////////////////////////////////////////////////////
// Protected:

void CRobot2D::extractInfo(IModel& model)
{
    PtVector& pts=model.GetVertices();

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

	//build a bounding circle
	float delta=0.2;
	int id=1;
	m_Geo.push_back(Point3d(m_Center[0],0,m_Center[1]));
	for(float d=delta;d<PI2;d+=delta,id++){
		float cos_d=cos(d);
		float sin_d=sin(d);
		Point2d pt(cos_d*m_Radius,sin_d*m_Radius);
		m_Geo.push_back(Point3d(pt[0],0,pt[1]));
		Tri tri(0,id,id+1);
		m_Tri.push_back(tri);
	}
	m_Tri.back()[2]=1; //wrap around
}

void CRobot2D::findUsefulTri(PtVector& pts, TriVector& tris)
{
    typedef TriVector::iterator TIT;
    
    m_Tri.clear();
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
}


bool CRobot2D::buildPQP2D()
{
    //create PQP MODEL
    m_Poly2D.BeginModel();
    //for each polygon
    for( int iT =0; iT< (int)m_Tri.size(); iT ++ )
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




