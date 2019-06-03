///////////////////////////////////////////////////////////////////////////////////////////
// ILoadable is an interface file for all model loader

#ifndef _ILOADABLE_H_
#define _ILOADABLE_H_

#include "IModel.h"
#include <string.h>

class ILoadable : public IModel
{
public:

    ILoadable(){ m_strFileName=NULL; }
    virtual void SetDataFileName( const char * szFileName )
    {
        if( szFileName==NULL )
            return;

        if( m_strFileName!=NULL )
            delete m_strFileName;

        m_strFileName=new char[strlen(szFileName)+1];
        strcpy(m_strFileName,szFileName);
    }

    virtual bool ParseFile(bool silent=false)=0;

protected:
    char * m_strFileName;
};

#endif //_ILOADABLE_H_
