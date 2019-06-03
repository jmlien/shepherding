#include "ObjLoader.h"
#include "ModelTool.h"
//#include <strstream>

//////////////////////////////////////////////////////////////////////////////////////
// Global
//////////////////////////////////////////////////////////////////////////////////////

inline bool v_n(string& s,int& v, int& n){

    if( sscanf(s.c_str(), "%d//%d", &v, &n)==2 ){
        v--; n--;
        return true;
    }
    return false;
}

inline bool vtn(string& s,int& v, int& t, int& n){

    if( sscanf(s.c_str(), "%d/%d/%d", &v, &t, &n)==3 ){
        v--; t--; n--;
        return true;
    }
    return false;
}

inline bool vt_(string& s,int& v, int& t){

    if( sscanf(s.c_str(), "%d/%d", &v, &t)==2 ){
        v--; t--;
        return true;
    }
    return false;
}

inline bool v__(string& s,int& v){
    if( sscanf(s.c_str(), "%d", &v)==1 ){
        v--;
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////
// Implemetation of ILoadable interface
//////////////////////////////////////////////////////////////////////////////////////
bool 
CObjLoader::ParseFile(bool silent)
{
    if( CheckCurrentStatus(silent)==false )
        return false;

    //Open file for reading datat
    ifstream fin(m_strFileName, ios::in);
    ReadOBJ(fin);
    fin.close();
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////
//
//
//  Protected Member Functions
//
//
//////////////////////////////////////////////////////////////////////////////////////
bool CObjLoader::CheckCurrentStatus(bool silent)
{
    if( m_strFileName==NULL )
        return false;

    //Check if file exist
    ifstream fin(m_strFileName, ios::in);
    if( !fin.good() )
    {   //not good. file not found
        if(!silent) cerr<<"File "<< m_strFileName <<" not found"<<endl;
        return false;
    }
    fin.close();

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////
//
//  Private Methods
//
////////////////////////////////////////////////////////////////////////////////////////////



/* glmFirstPass: first pass at a Wavefront OBJ file that gets all the
 * statistics of the model (such as #vertices, #normals, etc)
 *
 * model - properly initialized GLMmodel structure
 * file  - (fopen'd) file descriptor 
 */
bool CObjLoader::FirstPass(istream& in) 
{
    unsigned int  numvertices=0;  // number of vertices in model
    unsigned int  numnormals=0;   // number of normals in model 
    unsigned int  numtexcoords=0; // number of texcoords in model
    unsigned int  numtriP=0;      // number of triP in model
    unsigned int  numtriN=0;      // number of triP in model
    unsigned int  numtriT=0;      // number of triP in model

    string buf;
    char c1,c2;       //first/second character
    
    // make a default group
    //CObjGroup& group = AddGroup("default");
    
    numvertices = numnormals = numtexcoords = numtriP = 0;
    while( !in.eof() ) {
        in>>c1;
        switch(c1) {
        case '#':               //* comment 
            //* eat up rest of line
            getline(in,buf);
            break;
        ///////////////////////////////////////////////////////////////////////
        case 'v':               /* v, vn, vt */
            in.get(c2); 
            switch(c2) {
            case ' ':          /* vertex */
                // eat up rest of line 
                getline(in,buf);
                numvertices++;
                break;
            case 'n':           /* normal */
                // eat up rest of line 
                getline(in,buf);
                numnormals++;
                break;
            case 't':           /* texcoord */
                // eat up rest of line 
                getline(in,buf);
                numtexcoords++;
                break;
            default:
                cerr<<"! Error: FirstPass: Unknown token "<< buf <<".\n";
                return false;
            }
            break;
        ///////////////////////////////////////////////////////////////////////
        case 'm':
            in>>buf>>buf;
            getline(in,buf);
            //model->mtllibname = strdup(buf);
            //glmReadMTL(model, buf);
            break;
        case 'u':
            // eat up rest of line 
            getline(in,buf);
            break;
        case 'g':               /* group */
            getline(in,buf);
            break;
        case 'f':               /* face */
            {
                string junk; int v,t,n;
                in>>junk>>junk>>junk;
                if( v_n(junk,v,n) ) numtriN++;
                else if( vtn(junk,v,t,n) ){ numtriN++; numtriT++; }
                else if( vt_(junk,v,t) ) numtriT++;
                numtriP++;
            }
            getline(in,buf);
            break;  
        default:
            // eat up rest of line 
            getline(in,buf);
            break;
        }
    }
    
    /* set the stats in the model structure */
    points.reserve(numvertices);
    triP.reserve(numtriP);
    triN.reserve(numtriN);
    triT.reserve(numtriT);
    textures.reserve(numtexcoords);
    normals.reserve(numnormals);
    //group.tris_end=numtriP;

    return true;
}

/* glmSecondPass: second pass at a Wavefront OBJ file that gets all
 * the data.
 *
 * model - properly initialized GLMmodel structure
 * file  - (fopen'd) file descriptor 
 */
bool CObjLoader::SecondPass(istream& in) 
{
    string  buf;
    char c1,c2;
    Point3d pt;
    Vector3d n;
    Vector2d t;
    //CObjGroup& group=FindGroup("default");           // current group pointer
    
    /* on the second pass through the file, read all the data into the
    allocated arrays */
    while(!in.eof()) {
        in>>c1;
        switch(c1) {
        case '#':               // comment 
            // eat up rest of line 
            getline(in,buf);
            break;
        case 'v':               /* v, vn, vt */
            in.get(c2);
            switch(c2) {
            case ' ':          /* vertex */
                in>>pt;
                points.push_back(pt);
                break;
            case 'n':           /* normal */
                in>>n;
                normals.push_back(n);
                break;
            case 't':           /* texcoord */
                in>>t;
                textures.push_back(t);
                break;
            }
            break;
        case 'u':
            in>>buf>>buf;
            //group.material = material = FindMaterial(buf);
            getline(in,buf);
            break;
        case 'g':               /* group */
            getline(in,buf);
            break;
        case 'f': //triangles
            {
                in>>buf;
                Tri tv,tn,tt;
                // can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d
                if (v_n(buf,tv[0],tn[0])) { // v//n
                    in>>buf; v_n(buf,tv[1],tn[1]);
                    in>>buf; v_n(buf,tv[2],tn[2]);
                    triP.push_back(tv);
                    triN.push_back(tn);
                } 
                else if (vtn(buf,tv[0],tt[0],tn[0])) { // v/t/n
                    in>>buf; vtn(buf,tv[1],tt[1],tn[1]);
                    in>>buf; vtn(buf,tv[2],tt[2],tn[2]);
                    triP.push_back(tv);
                    triT.push_back(tt);
                    triN.push_back(tn);
                } 
                else if (vt_(buf,tv[0],tt[0])) { // v/t
                    in>>buf; vt_(buf,tv[1],tt[1]);
                    in>>buf; vt_(buf,tv[2],tt[2]);
                    triP.push_back(tv);
                    triT.push_back(tt);
                } else { // v
                    v__(buf,tv[0]);
                    in>>buf; v__(buf,tv[1]);
                    in>>buf; v__(buf,tv[2]);
                    triP.push_back(tv);
                }
            }
            break;
        default:
            // eat up rest of line 
            getline(in,buf);
            break;
        }
    }

    return true;
}


/* glmReadOBJ: Reads a model description from a Wavefront .OBJ file.
 * Returns a pointer to the created object which should be free'd with
 * glmDelete().
 *
 * filename - name of the file containing the Wavefront .OBJ format data.  
 */
bool CObjLoader::ReadOBJ(istream& in)
{   
    if( FirstPass(in)==false ) return false;
    in.clear();
    in.seekg(0,ios::beg); //set to begining.
    if( SecondPass(in)==false ) return false;
    if( normals.empty() ) 
        ComputeFaceNormal(*this);
    return true;
}

/* glmFindGroup: Find a group in the model */
/*
CObjGroup& CObjLoader::FindGroup(const string& name)
{
    typedef vector<CObjGroup>::iterator GIT;
    for(GIT i=groups.begin();i!=groups.end();i++)
        if( i->name==name ) return *i;
    return *groups.end();
}
*/

/* AddGroup: Add a group to the model */
/*
CObjGroup& CObjLoader::AddGroup(const string& name)
{   
    CObjGroup& group = FindGroup(name);
    if (&group==groups.end()) {
        CObjGroup g;
        g.name = name;
        g.material = 0;
        groups.push_back(g);
        return groups.back();
    }
    else
        return group;
}
*/
