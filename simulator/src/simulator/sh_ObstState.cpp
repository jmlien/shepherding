#include "sh_ObstState.h"

int CObsState::TOTAL_STATE_SIZE=0;
int CObs::TOTAL_OBS_SIZE=0;

CObsState::CObsState(CObs * type)
{
    m_Type=type;
    memset(m_R,0,sizeof(float)*9); 
    m_R[0][0]=m_R[1][1]=m_R[2][2]=1;
    m_radian=0;
    m_Texture=-1;
    m_ID=TOTAL_STATE_SIZE;
    TOTAL_STATE_SIZE++;
}

///////////////////////////////////////////////////////////////////////////
//Constructor/Destructor
CObs::CObs(const std::string& filename, int size)
{ 
    IModel * model=CreateModelLoader(filename);
	initialize(model,size);
}

CObs::CObs(IModel * model, int size)
{ 
	initialize(model,size);
}

CObs::~CObs()
{ 
    m_State.clear();
    m_ID=TOTAL_OBS_SIZE;
    TOTAL_OBS_SIZE++;
}

void CObs::initialize(IModel * model, int size)
{
    m_State.reserve(size);
    for( int i=0;i<size;i++ ){
        CObsState* s=new CObsState(this);
        if( s==NULL ) throw;
        m_State.push_back(s);
    }

    m_Model=model;
    if( m_Model==NULL ) throw;
    if( m_Obs.buildCDModel(*m_Model)==false ) throw;
}

///////////////////////////////////////////////////////////////////////////
CObsState& CObs::addState()
{
	CObsState* s=new CObsState(this);
    if( s==NULL ) throw;
    m_State.push_back(s);
	return *s;
}
