///////////////////////////////////////////////////////////////////////////////////////////
// This file defines a Movie.BYU format fileloader


#ifndef _MOVIEBYULOADER_H_
#define _MOVIEBYULOADER_H_

#include "ILoadable.h"
#include <fstream>
using namespace std;

class CMovieBYULoader : public ILoadable
{
public:

    //////////////////////////////////////////////////////////////////////////////////////
    CMovieBYULoader()
    {
        m_PartsSize=m_VertexSize=m_PolygonSize=m_EdgeSize=0;
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // Implemetation of ILoadable interface
    //////////////////////////////////////////////////////////////////////////////////////
    virtual bool ParseFile(bool silent=false);

    //////////////////////////////////////////////////////////////////////////////////////
    // Implemetation of IModel interface
    //////////////////////////////////////////////////////////////////////////////////////
    const PtVector & GetVertices() const{ return points; }
    const TriVector & GetTriP() const{ return triP; } 
    const TriVector & GetTriN() const{ return triN; }  //triangle normals
    const TriVector & GetTriT() const{ return triT; }  //triangle texture
    const V2Vcetor & GetTextureCoords() const { return textures; }
    const V3Vcetor & GetNormals() const { return normals; }

    PtVector & GetVertices() { return points; }
    TriVector & GetTriP(){ return triP; }
    TriVector & GetTriN(){ return triN; }
    TriVector & GetTriT(){ return triT; }
    V2Vcetor & GetTextureCoords() { return textures; }
    V3Vcetor & GetNormals() { return normals; }

////////////////////////////////////////////////////////////////////////////////////////////
//
//  Protected Methods and data members
//
////////////////////////////////////////////////////////////////////////////////////////////
protected:
    virtual bool CheckCurrentStatus(bool silent=false);

////////////////////////////////////////////////////////////////////////////////////////////
//
//  Private Methods and data members
//
////////////////////////////////////////////////////////////////////////////////////////////
private:
    bool ParseSection1(ifstream & in);
    bool ParseSection2(ifstream & in);
    bool ParseSection3(ifstream & in);

    int m_PartsSize,m_VertexSize,m_PolygonSize,
        m_EdgeSize;

    vector< pair<int,int> > parts;
    PtVector points;
    TriVector triP;
    TriVector triN;    //not support
    TriVector triT;    //not support
    V2Vcetor  textures; //not support
    V3Vcetor  normals;  //not support
};

#endif
