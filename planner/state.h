#ifndef _JML_MP_STATE_
#define _JML_MP_STATE_

#include <string>
#include <list>
using namespace std;

class MPstate
{
public:

    void pushState(const string& name);
    void popState();
    void updateState(const string& name);
    
    //increase cd count
    void incCDcount()
    { 
        if(m_states.empty()) return;
        m_states.back().m_CD_count++;
    }

private:

    void outputState();
    
    struct MyState{
        MyState(){ m_CD_count=0; m_Clock=0; }
        string m_Name;
        unsigned long m_CD_count;
        unsigned long m_Clock;
    };
    list<MyState> m_states;
};

#endif //_JML_MP_STATE_
