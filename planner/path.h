#ifndef _JML_PATH_H_
#define _JML_PATH_H_

#include <list>
#include <vector>
using namespace std;

#include "cfg.h"

class Path
{
public:

    //void optimize();

    //add new cfgs to path
    void append(const list<Cfg>& cfgs){ 
        m_path.insert(m_path.end(),cfgs.begin(),cfgs.end());
    }

    void append(const Cfg& cfg){ m_path.push_back(cfg); }

    //clear all cfgs in the path
    void clear(){ m_path.clear(); }

    //access functions
    vector<Cfg>& getPath(){ return m_path; }

    const vector<Cfg>& getPath() const { return m_path; }

private:

    vector<Cfg> m_path;

};

#endif //_JML_PATH_H_
