#include "ModelTool.h"
#include "IModel.h"

void ComputeFaceNormal(IModel& model)
{
    const IModel::PtVector& points=model.GetVertices();
    const IModel::TriVector& triP=model.GetTriP();
    IModel::TriVector& triN=model.GetTriN();
    IModel::V3Vcetor & normals=model.GetNormals();
    
    triN.reserve(triP.size());
    normals.reserve(triP.size());

    typedef IModel::TriVector::const_iterator TIT;
    int index=0;
    for( TIT it=triP.begin();it!=triP.end();it++,index++ ){
        const IModel::Tri& tri=*it;
        
        const Point3d& p1=points[tri[0]];
        const Point3d& p2=points[tri[1]];
        const Point3d& p3=points[tri[2]];
        
        Vector3d n=((p2-p1)%(p3-p1)).normalize();
        normals.push_back(n);
        triN.push_back(IModel::Tri(index,index,index));
    }
}
