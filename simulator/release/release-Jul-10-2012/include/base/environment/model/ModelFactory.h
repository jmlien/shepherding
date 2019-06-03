#ifndef _MODEL_FACTORY_H_
#define _MODEL_FACTORY_H_

#include <string>
using namespace std;

class IModel;
IModel * CreateModelLoader(const std::string& file, bool silent=false);

#endif //_MODEL_FACTORY_H_
