#ifndef _SH_LOCALINFO_H_
#define _SH_LOCALINFO_H_

class CEnvironment;
class CFlockState;

#include <list>
using namespace std;

void getLocalInfo
(list<CFlockState*>& flock,CEnvironment& env);

#endif//_SH_LOCALINFO_H_


