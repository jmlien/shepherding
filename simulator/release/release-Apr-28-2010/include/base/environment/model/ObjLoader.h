///////////////////////////////////////////////////////////////////////////////////////////
// This file defines a A/W obj format fileloader


#ifndef _AW_OBJLOADER_H_
#define _AW_OBJLOADER_H_

#include "ILoadable.h"
#include <fstream>
using namespace std;

struct CObjmaterial
{
  string  name;                 // name of material 
  float diffuse[4];           // diffuse component
  float ambient[4];           // ambient component
  float specular[4];          // specular component
  float emmissive[4];         // emmissive component
  float shininess;            // specular exponent
};

struct  CObjGroup {
  string            name;       //name of this group
  unsigned int      tris_start;
  unsigned int      tris_end;
  unsigned int      material;   //index to material for group
};

class CObjLoader : public ILoadable
{
public:

    //////////////////////////////////////////////////////////////////////////////////////
    CObjLoader()
    {
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
    TriVector & GetTriN(){ return triN; }  //triangle normals
    TriVector & GetTriT(){ return triT; }  //triangle texture
    V2Vcetor & GetTextureCoords() { return textures; }
    V3Vcetor & GetNormals() { return normals; }

////////////////////////////////////////////////////////////////////////////////////////////
//
//  Protected Methods and data members
//
////////////////////////////////////////////////////////////////////////////////////////////
protected:
    virtual bool CheckCurrentStatus(bool silent=false);

    bool ReadOBJ(istream& in);
    bool FirstPass(istream& in);
    bool SecondPass(istream& in);
    //CObjGroup& FindGroup(const string& name);
    //CObjGroup& AddGroup(const string& name);

////////////////////////////////////////////////////////////////////////////////////////////
//
//  Private Methods and data members
//
////////////////////////////////////////////////////////////////////////////////////////////
private:

    PtVector points;
    TriVector triP;  //points index
    TriVector triN;  //normal index
    TriVector triT;  //texture index

    V2Vcetor  textures;
    V3Vcetor  normals;
    //vector<CObjGroup>  groups;
    //vector<CObjmaterial> material;
};

#endif
