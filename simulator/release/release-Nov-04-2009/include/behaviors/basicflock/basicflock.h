#ifndef BASE_H_
#define BASE_H_

#include "ShepherdingInstance.h"

class BasicFlock : public ShepherdingInstance
{
protected:
	bool initialize(list< list<string> >& tokens, flock_raw_data& data);
};


#endif //BASE_H_
