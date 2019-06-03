

#ifndef LOG_PARSER_H
#define LOG_PARSER_H


#include <vector>
#include <string>
#include <cmath>


struct FILE_HEADER
{
   char timestamp[50];
   char user_id[50];
   char environment[50];
   int num_shepherds;
   int num_sheep;
};

struct POINT2I
{
   int x, y;
   
   float Distance(const POINT2I& p) const
   {
      const float x_dif = x - p.x;
      const float y_dif = y - p.y;
      return sqrt(x_dif*x_dif + y_dif*y_dif);
   }
};

struct POINT2F
{
   float x, y;
   
   float Distance(const POINT2F& p) const
   {
      const float x_dif = x - p.x;
      const float y_dif = y - p.y;
      return sqrt(x_dif*x_dif + y_dif*y_dif);
   }
};

struct LOG_FRAME
{
   float time;
   int steps;
   POINT2I mouse;
   std::vector<POINT2F> shepherds;
   std::vector<POINT2F> sheep;
};


class LOG_DATA
{
   public:
   std::string filename;
   std::string timestamp;
   std::string user_id;
   std::string environment;
   int num_shepherds;
   int num_sheep;
   std::vector<LOG_FRAME> frames;
   float total_time;
   int total_steps;
   bool success;
   
   protected:
   LOG_DATA(const std::string&, const std::string&, const std::string&, const std::string&, const int, 
      const int, const std::vector<LOG_FRAME>&, const float, const int, const bool);
   
   public:
   static LOG_DATA Parse(const std::string& filename);
};


#endif // LOG_PARSER_H


