
#ifndef SIMPLE_HERDING_DRAW_EXT_H
#define SIMPLE_HERDING_DRAW_EXT_H

#include "simple_herding_draw.h"

class DrawDir : public sh_Draw
{
   public:
   Vector2d dir;
   Point2d pos;

   void draw()
   {
#if GL_ON
      glDisable(GL_LIGHTING);
      glLineWidth(2);
      glColor3f(0.1f, 0.1f, 0.5f);

      glBegin(GL_LINES);
         glVertex2f(pos.x,         pos.y);
         glVertex2f(pos.x + dir.x, pos.y + dir.y);
      glEnd();
#endif
   }
};


#endif
