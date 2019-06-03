

#include "log_parser.h"
#include <fstream>
#include <algorithm>


LOG_DATA::LOG_DATA(const std::string& filenamer, const std::string& timestampr, const std::string& user_idr, const std::string& environmentr,
   const int num_shepherdsr, const int num_sheepr, const std::vector<LOG_FRAME>& framesr, const float total_timer, const int total_stepsr, const bool successr) :
   //
   filename(filenamer),
   timestamp(timestampr),
   user_id(user_idr),
   environment(environmentr),
   num_shepherds(num_shepherdsr),
   num_sheep(num_sheepr),
   frames(framesr),
   total_time(total_timer),
   total_steps(total_stepsr),
   success(successr)
{
   // do nothing
}

LOG_DATA LOG_DATA::Parse(const std::string& filename)
{
   std::ifstream fin(filename.c_str(), std::ios::binary);
   
   FILE_HEADER header;
   int struct_size;
   fin.read((char*)&struct_size, sizeof(int));
   fin.read((char*)&header, struct_size);
   
   std::vector<LOG_FRAME> frames;
   bool success = false;
   
   while(fin.good())
   {
      LOG_FRAME frame;
      fin.read((char*)&frame.time, sizeof(float));
      if(frame.time < 0.0f)
      {
         success = true;
         break;
      }
      fin.read((char*)&frame.steps, sizeof(int));
      fin.read((char*)&frame.mouse.x, sizeof(int));
      fin.read((char*)&frame.mouse.y, sizeof(int));
      for(int i = 0; i < header.num_shepherds; i++)
      {
         POINT2F point;
         fin.read((char*)&point.x, sizeof(float));
         fin.read((char*)&point.y, sizeof(float));
         frame.shepherds.push_back(point);
      }
      for(int i = 0; i < header.num_sheep; i++)
      {
         POINT2F point;
         fin.read((char*)&point.x, sizeof(float));
         fin.read((char*)&point.y, sizeof(float));
         frame.sheep.push_back(point);
      }
      frames.push_back(frame);
   }
   fin.close();
   
   return LOG_DATA(filename, header.timestamp, header.user_id, header.environment, header.num_shepherds, header.num_sheep, 
   frames, frames[frames.size() - 1].time, frames[std::max(0UL, frames.size() - 2)].steps, success);
}


      
      
