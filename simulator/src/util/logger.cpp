#ifndef WIN32
#include <sys/time.h>
#else
#include <Windows.h>
#include <Mmsystem.h>
#endif

#include "shepherding_base.h"
#include <stdlib.h>
#include <stdio.h>


// finds available filename, and uses that for the log
void SimLogger::InitLog(const string& environment_name)
{
   string user_id=UserId();
   string environment = environment_name;
   string sepChar="/";

#ifdef WIN32
    sepChar="\\";
#endif

   int slash_char = environment.find(sepChar);
   while(slash_char != -1)
   {
      environment.replace(slash_char, 1, "-");
      slash_char = environment.find(sepChar);
   }
   
   ifstream file_probe;
   string filename;
   for(int i=0;;i++)
   {
      stringstream ss;
      ss << "logs" << sepChar << user_id << "--" << environment << "--" << i << ".txt";
      filename=ss.str();
      file_probe.open(filename.c_str(), ios::binary);
      if( file_probe.fail() ) break;
      file_probe.close();
   }
   file_probe.close();
   cout<<"- Save log in file ("<<filename<<")"<<endl;
   
   //open file
   logfile.open(filename.c_str(), ios::binary);
   
   
   const time_t timestamp = time(0);
   strcpy(file_header.timestamp, ctime(&timestamp));                 // string-formatted time stamp
   strcpy(file_header.user_id, user_id.c_str());                     // user id
   strcpy(file_header.environment, environment_name.c_str());        // environment name
  
   CEnvironment* env = m_sim->getEnvironment();
   assert(env); //make sure that env is not null
   
   list<CFlockState*>& va = env->getFlockStates(); 
   assert(!va.empty()); //this should not be empty
  
   
   for(FSLIST::iterator i=va.begin();i!=va.end();i++)
   {
      CFlockState * a=*i;
      string type=string(typeid(*a->getType()).name());
      
      if( type.find("HerdingFlock")!=string::npos )
          shepherds.push_back(a);
      else
          sheep.push_back(a);
    }
    file_header.num_shepherds = shepherds.size();                     // number of shepherds
    file_header.num_sheep = sheep.size();                             // number of sheep
    
    int struct_size = sizeof(file_header);                            // write the (assumed 32bit int) size of the header
    logfile.write((char*)&struct_size, sizeof(int));                  //   to the beginning of the file, in case of padding
                                                                      //   issues between platforms
    logfile.write((char*)&file_header, sizeof(file_header));          // write the header to the logfile
    
    time_0 = Time();                                                  // set the initial time
}



// update the log with mouse position -- should be called from glut's passive motion func
void SimLogger::UpdateLog(const int x, const int y)
{	
   if(logfile.is_open() && m_sim->getCurrentTimeStep()%10 == 0)
   {
      // log the current time (float, in seconds)
      float t = Time() - time_0;
      logfile.write((char*)&t, sizeof(float));
      
      int timestep = m_sim->getCurrentTimeStep();
      logfile.write((char*)&timestep, sizeof(int));
      
      // logs the current mouse position (pair of ints)
      logfile.write((char*)&x, sizeof(int));
      logfile.write((char*)&y, sizeof(int));

       // write shepherd positions to file (pairs of floats)
      for(FSLIST::iterator i=shepherds.begin(); i != shepherds.end(); i++)
      {
         logfile.write((char*)&((*i)->getPos()[0]), sizeof(float));
         logfile.write((char*)&((*i)->getPos()[1]), sizeof(float));
      }
      
      // write sheep positions to file (pairs of floats)
      for(FSLIST::iterator i=sheep.begin(); i != sheep.end(); i++)
      {
         logfile.write((char*)&((*i)->getPos()[0]), sizeof(float));
         logfile.write((char*)&((*i)->getPos()[1]), sizeof(float));
      }
   }//end if
}


SimLogger& SimLogger::operator<<(int i)
{
    logfile.write((char*)&i, sizeof(int));
    return *this;
}


SimLogger& SimLogger::operator<<(float f)
{
    logfile.write((char*)&f, sizeof(float));
    return *this;
}


SimLogger& SimLogger::operator<<(double d)
{
    logfile.write((char*)&d, sizeof(double));
    return *this;
}

SimLogger& SimLogger::operator<<(const string& s)
{
	logfile.write((char*)s.c_str(), sizeof(char)*s.size());
    return *this;
}


// only closes the log file at the moment, but can add some kinda of de-init data later, if necessary
void SimLogger::CloseLog()
{
   logfile.close();
}


// get a user id
string SimLogger::UserId()
{
    RNG * rng=m_sim->getRNG();
    int id=rng->uniform(1,999);
    char sid[8];
    sprintf(sid,"%03d",id);
    return string(sid);
}

// returns current time in seconds
float SimLogger::Time()
{
    #ifdef WIN32
    return timeGetTime()*0.001f;
    #else
    static const unsigned int time_offset = time(0);
    timeval time_buffer;
    gettimeofday(&time_buffer, 0);
    return ((time_buffer.tv_sec - time_offset) + time_buffer.tv_usec*0.000001f);
    #endif
}

