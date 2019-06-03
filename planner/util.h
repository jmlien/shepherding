#ifndef _JML_UTIL_H_
#define _JML_UTIL_H_

// simply a set of functions that I don't knot where to put but are used
// in many places

#include <list>
#include <string>
using namespace std;

//convert a string to a list of tokens
inline list<string> tokenize(char * tmp, const char * ignore)
{
	list<string> tokens;
    char * tok=strtok (tmp,ignore);
    while(tok!=NULL)
    {
		tokens.push_back(tok);
        tok=strtok(NULL,ignore);
    }
    return tokens;
}

template<class T> bool isClass(T * ptr, const string& name){
	return string(typeid(*ptr).name()).find(name)!=string::npos;
}

#endif//_JML_UTIL_H_
