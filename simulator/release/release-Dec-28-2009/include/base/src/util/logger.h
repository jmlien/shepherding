/*
 * logger.h
 *
 *
 * defined class SimLogger.
 *
 * SimLogger takes a simulator (shSimulate) and logs the following things.
 *      - user id (?)
 *      - number of flock
 *      - flock's positions
 *      - simulation time
 * 
 * TODO: This should not depend on "shepherding/sheep"
 *
 * Last Major Modification: J-M Lien 12/28/2009
 * 
 */


#ifndef _SH_SIM_LOGGER_H
#define _SH_SIM_LOGGER_H

#include <fstream>

class shSimulate;  //defined in sh_sim.h
class CFlockState; //defined in sh_FlockState.h

class SimLogger
{   
public:

    SimLogger(shSimulate * sim){ m_sim=sim; }
    
    void InitLog(const string& file);
    void UpdateLog(const int x=0, const int y=0);
    void CloseLog();

    SimLogger& operator<<(int i);
    SimLogger& operator<<(float f);
    SimLogger& operator<<(double d);
    SimLogger& operator<<(const string& s);

private:

    shSimulate * m_sim;
    ofstream logfile;
    list<CFlockState*> shepherds;
    list<CFlockState*> sheep;
    float time_0;
    
    //get a random user id
    string UserId();
    
    // returns current time in seconds
    float Time();
    
    struct FILE_HEADER
    {
        char timestamp[50];
        char user_id[50];
        char environment[50];
        int num_shepherds;
        int num_sheep;
    } file_header;

};

#endif // _SH_SIM_LOGGER_H


