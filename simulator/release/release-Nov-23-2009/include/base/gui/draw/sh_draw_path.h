#ifndef _SH_DRAW_PATH_H_
#define _SH_DRAW_PATH_H_

#include <list>
using namespace std;

#include <Point.h>
using namespace mathtool;

void drawPath(const list<Point2d>& ptlist, float r=1, bool text=false);

#endif //_SH_DRAW_PATH_H_
