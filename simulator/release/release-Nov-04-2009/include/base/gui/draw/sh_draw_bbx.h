#ifndef _SH_DRAW_BBX_H_
#define _SH_DRAW_BBX_H_

#include "sh_BoundingBox.h"

void doClip(const CBoundingBox& bbx);
void unClip();
void drawBBX(const CBoundingBox& bbx);
void drawGround(const CBoundingBox& bbx);

#endif //_SH_DRAW_BBX_H_


