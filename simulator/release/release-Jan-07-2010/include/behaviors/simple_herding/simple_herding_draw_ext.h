
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
         glVertex2f(pos[0],          pos[1]);
         glVertex2f(pos[0] + dir[0], pos[1] + dir[1]);
      glEnd();
#endif
   }
};


#endif
