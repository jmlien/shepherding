#include "MovieBYULoader.h"
#include "ModelFactory.h"
#include "ObjLoader.h"

IModel * CreateModelLoader(const string& file, bool silent)
{
    ILoadable * model=NULL;
    //get extension
    unsigned int pos=file.rfind('.');
    if( pos==string::npos ){
        if(!silent) cerr<<"! Error : Can't Recognize file :"<<file<<endl;
        return NULL;
    }
    string ext=file.substr(pos+1);
    if( ext=="g" )
        model=new CMovieBYULoader();
    else if( ext=="obj")
        model=new CObjLoader();
    else
        if(!silent) cerr<<"! Error : Can't Recognize extension *."<<ext<<endl;
    if( model==NULL ) return NULL;
    model->SetDataFileName(file.c_str());
    if( model->ParseFile(silent)==false ){
        delete model;
        return NULL;
    }
    
    return model;
}
