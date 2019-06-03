

#include "state.h"
#include <time.h>
#include <iostream>

//-----------------------------------------------------------------------------
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

unsigned long getTime() //in millisecond
{
#ifdef WIN32
    return timeGetTime();
#else //assuming unix-type systems
    //timezone tz;
    timeval  tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec*1000+(tv.tv_usec)/1000;
#endif
}
//-----------------------------------------------------------------------------

void MPstate::pushState(const string& name)
{
    MyState tmp;
    tmp.m_Name=name;
    m_states.push_back(tmp);
    m_states.back().m_Clock=getTime();
}

void MPstate::popState()
{
    if(m_states.empty()) return;
    
    MyState& tmp=m_states.back();
    tmp.m_Clock=getTime()-tmp.m_Clock;

    outputState();

    if(m_states.size()>1) //more than 1 element
    {
        MyState& tmp2=*(--(--m_states.end()));
        tmp2.m_CD_count+=tmp.m_CD_count;
    }

    m_states.pop_back();
}

void MPstate::updateState(const string& name)
{
    if(m_states.empty()) return;
    
    MyState& tmp=m_states.back();
    tmp.m_Clock=getTime()-tmp.m_Clock;

    outputState();
    
    if(m_states.size()>1) //more than 1 element
    {
        MyState& tmp2=*(--(--m_states.end()));
        tmp2.m_CD_count+=tmp.m_CD_count;
    }

    tmp.m_Name=name;
    tmp.m_CD_count=0;
    tmp.m_Clock=getTime();
}

void MPstate::outputState()
{
    if(m_states.empty()) return;
    MyState& tmp=m_states.back();
    
    cout<<"- "<<tmp.m_Name<<": "<<tmp.m_CD_count<<" CD calls, "
        <<tmp.m_Clock<<" milliseconds"<<endl;
}


