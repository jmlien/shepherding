#ifndef _IPOLYGONAL_H_
#define _IPOLYGONAL_H_

//////////////////////////////////////////////////////////////////////
//  This is an interface for all classes which constians
//  polygonal data. For example, deformable object, sculpture deformer
//  and evene model loader
//////////////////////////////////////////////////////////////////////

#include <Point.h>
#include <Vector.h>
using namespace mathtool;

#include <vector>
using namespace std;

class IModel  
{
public:

//////////////////////////////////////////////////////////////////////
//  Interface for Retrive general polgons
//////////////////////////////////////////////////////////////////////
    typedef Vector<int, 3> Tri;

    typedef vector<Point3d>  PtVector;
    typedef vector<Tri>      TriVector;
    typedef vector<Vector3d> V3Vcetor;
    typedef vector<Vector2d> V2Vcetor;
//////////////////////////////////////////////////////////////////////
    virtual const PtVector & GetVertices() const =0;
    virtual const TriVector & GetTriP() const =0;  //triangle points
    virtual const TriVector & GetTriN() const =0;  //triangle normals
    virtual const TriVector & GetTriT() const =0;  //triangle texture

    virtual const V2Vcetor & GetTextureCoords() const =0;
    virtual const V3Vcetor & GetNormals() const =0;
//////////////////////////////////////////////////////////////////////  
    virtual PtVector & GetVertices() =0;
    virtual TriVector & GetTriP() =0;  //triangle points
    virtual TriVector & GetTriN() =0;  //triangle normals
    virtual TriVector & GetTriT() =0;  //triangle texture
    
    virtual V2Vcetor & GetTextureCoords() =0;
    virtual V3Vcetor & GetNormals() =0;
};

#endif // !defined(_IPOLYGONAL_H_)
