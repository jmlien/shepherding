#ifndef _SH_DRAWCLASS_H_
#define _SH_DRAWCLASS_H_

#include "Point.h"
#include <list>
#include <string>
using namespace std;
using namespace mathtool;

//a generic draw object
class sh_Draw
{ 
public:

    sh_Draw(){ m_signature="unknown"; }
    
    //the main draw function undefined
    virtual void draw()
    {
        drawDrawObject();
        drawTextInfo();
    }
    
    //check if the given draw object is compatiable with this
    //only compatiable draw object can be added to the draw list
    bool isCompatible(sh_Draw* v){ return this->m_signature==v->m_signature; }

    //add a draw object
    virtual void addDrawObj( sh_Draw * v )
    { 
        if(isCompatible(v)) m_drawobjs.push_back(v); 
    }
    
    //add a text draw object. This object will be rendered in
    //orthogonal view.
    virtual void addDrawInfo( const std::string& tag, const std::string& value)
    { m_drawinfos.push_back(pair<string,string>(tag+": ",value)); }
    
    //remove draw objects if added
    virtual void removeDrawObj( sh_Draw * v )
    { 
        if(isCompatible(v)) m_drawobjs.remove(v); 
    }
    
    virtual void removeDrawInfo( const std::string& tag )
    {
        typedef list< pair<string,string> >::iterator PIT;
        string my_tag=tag+": ";
        for( PIT i=m_drawinfos.begin();i!=m_drawinfos.end();i++ )
            if( i->first.compare(my_tag)==0 ){ 
                m_drawinfos.erase(i); break; 
            }
    }

	virtual void updateDrawInfo(const std::string& tag, const std::string& value)
	{
		typedef list< pair<string, string> >::iterator PIT;
		string my_tag = tag + ": ";
		for (PIT i = m_drawinfos.begin(); i != m_drawinfos.end(); i++)
		if (i->first.compare(my_tag) == 0){
			i->second = value;
			return;
		}
		//not found
		addDrawInfo(tag,value);
	}


protected:

    //draw the list m_drawobjs
    virtual void drawDrawObject()
    {
        //draw the user inseted objects
        typedef list<sh_Draw*>::iterator DIT;
        for(DIT i=m_drawobjs.begin();i!=m_drawobjs.end();i++)
            (*i)->draw();
    }
    
    //draw list m_drawinfos
    virtual void drawTextInfo()
    {
        //Draw customized text
        typedef list< pair<string,string> >::iterator TIT;
        for(TIT i=m_drawinfos.begin();i!=m_drawinfos.end();i++)
            drawText(i->first.c_str(),i->second.c_str());
    }
    
    //draw the given text with format "tag" "value"
    virtual void drawText(const char * tag, const char * value)
    {
        cout<<tag<<" "<<value<<endl;
    }
    

    list<sh_Draw*> m_drawobjs;
    list< pair<string,string> > m_drawinfos;
    string m_signature;  //this signature is used to check the compatiablity of draw classes 
};


#endif //_SH_DRAWCLASS_H_


