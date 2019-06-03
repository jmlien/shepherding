// PolyhedronModel.cpp: implementation of the CPolyhedronModel class.
//
//////////////////////////////////////////////////////////////////////

#include "sh_PolyhedronModel.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CPolyhedronModel::CPolyhedronModel()
{
    m_Scale=1;
}

//////////////////////////////////////////////////////////////////////
//Access
void CPolyhedronModel::setPos(float X, float Y, float Z)
{
    m_Pos.set(X,Y,Z); 
}

void CPolyhedronModel::setRot(float * R )
{
    m_Rot[0].set(R);
    m_Rot[1].set(R+3);
    m_Rot[2].set(R+6);
}

void CPolyhedronModel::setRot(float RX, float RY, float RZ)
{
    float sin_x=sin(RX); float cos_x=cos(RX);
    float sin_y=sin(RY); float cos_y=cos(RY);
    float sin_z=sin(RZ); float cos_z=cos(RZ);

    Matrix3x3 rx(1,0,0,0,cos_x,-sin_x,0,sin_x,cos_x); 
    Matrix3x3 ry(cos_y,0,sin_y,0,1,0,-sin_y,0,cos_y); 
    Matrix3x3 rz(cos_z,-sin_z,0,sin_z,cos_z,0,0,0,1); 
    m_Rot=rz*ry*rx;
}

