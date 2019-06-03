#ifndef _JML_QUERY_H_
#define _JML_QUERY_H_

#include <list>
#include <string>
using namespace std;


#include "mp.h"

//
// Query
//

class Query
{
public:
    
    Query();

    //initialize itself
    bool initialize(list<string>& toks);

    bool findPath(Path& path);

    //data access

    int getNumberofQuery() const { return m_querys.size(); }

    const Cfg& getQuery(int i) const { return m_querys[i]; }
    void addQuery(Cfg& cfg){ m_querys.push_back(cfg); }

private:

    //find a path between c1 and c2
    bool findPath(Path& path, const Cfg& c1, const Cfg& c2);

    //find a path between c1 and c2 via a CC
    bool findPath(Path& path, const Cfg& c1, const Cfg& c2, vector<VID>& CC);

    //connect c to a CC, return -1 if not connected
    VID findPath(list<Cfg>& path,const Cfg& c, vector<VID>& CC);

    vector<Cfg> m_querys;
    unsigned int m_k; 
};

#endif //_JML_QUERY_H_


