/*
 * shepherding.cpp
 *
 *
 * Implementation of shepherding.h
 * 
 * Last Major Modification : J-M Lien 12/28/2009
 */
 
#include "shepherding.h"
#include "scared_flock.h"
#include "manual_herding_rules.h"
#include "ZeroForceRule.h"
#include "simple_herding/simple_herding.h"

#if USE_WIIMOTE
#include "WIIMOTE.hpp"
#endif 

#ifndef NO_BOOST
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "server_client_interface/DeviceStateClient.h"
#include "server_client_interface/DeviceState.h"
#endif

//
//global access functions
//
Shepherding * g_instance=NULL;

Shepherding * getSI()        // get the shepherding instance
{
    return g_instance;
}

//set shepherding instance
void setSI(Shepherding * si)  // set the shepherding instance
{
    g_instance=si;
}

// get simulator
shSimulate * getSimulator()
{  
    assert(g_instance);
    return g_instance->getSimulator();
}

// get environment
CEnvironment * getEnvironment()
{      
    shSimulate * sim=getSimulator();
    assert(sim);
    return sim->getEnvironment();
}

// get RNG
RNG * getRNG()
{
    shSimulate * sim=getSimulator();
    assert(sim);
    return sim->getRNG();
}


//
//Shepherding starts here
//

Shepherding::
Shepherding()
{
    m_doUpdateFromServer = false;
    m_doUpdateFromMouse =  true;
    m_doUpdateFromWiimote= false;
    m_doDrawShepherd =     true;
    m_allowAutonomy =      true;
    m_mixedAutonomy =      false;
    m_allowLog =           false;
    m_useDirectPosition =  false;
    m_showVisualHints =    false;
    m_allowMA =            true;
    m_allowRegroup =       true;
    m_checkReachGoal =     true;
    m_shepherd =           NULL; //a point to a type of shepherd
    m_sim =                NULL;
    m_doDraw =             true;
    m_succeeded =          false;
    m_RNG_seed =           time(NULL);
    m_sim_budget=          15000;
    gui_active_shepherd=   -1;
}


void Shepherding::parseCmdline(int argc, char * argv[])
{
    for (int i = 1; i < argc; i++)
    {
        string tmp=argv[i];
        if (tmp=="-g") {
            m_doDraw=false;
        }
        else if(tmp=="-client") {
            m_doUpdateFromServer = true;
            m_doUpdateFromMouse = false;
        }
        else if (tmp=="-mouse") {
            m_doUpdateFromServer = false;
            m_doUpdateFromMouse = true;
        }
        else if (tmp=="-wiimote") {
            m_doUpdateFromWiimote = true;
            m_doUpdateFromMouse = false;
        }
        else if (tmp=="-drawshepherd") {
            m_doDrawShepherd = true;
        }
        else if (tmp=="-no-drawshepherd") {
            m_doDrawShepherd = false;
        }
        else if (tmp == "-disable-autonomy") {
            m_allowAutonomy = false;
        }
        else if (tmp=="-enable-autonomy") {
            m_allowAutonomy = true;
        }
        else if (tmp=="-mixed-autonomy") {
            m_allowAutonomy = false;
            m_mixedAutonomy = true;
        }
        else if (tmp=="-use-direct-position") {
            m_useDirectPosition = true;
        }
        else if (tmp=="-use-force-position") {
            m_useDirectPosition = false;
        }
        else if (tmp=="-show-visual-hints") {
            m_showVisualHints = true;
        }
        else if (tmp=="-seed") {
            if(i+1<argc) m_RNG_seed=atol(argv[++i]);
        }
        else if (tmp=="-budget") {
            if(i+1<argc) m_sim_budget=atol(argv[++i]);
        }
        else if (tmp=="-enable-log"){
            m_allowLog=true;
        }
        else if (tmp=="-h") {
            showHelp(cout, argv[0]);
            exit(0);
        }
		
#ifdef _WIN32
		//create xbox controller
		else if (tmp == "-xbox")
		{

			//xbox controller
			xbox_controller = NULL;
			xbox_controller = new CXBOXController(1);
			if (xbox_controller->IsConnected() == false) //failed to connect
			{
				delete xbox_controller;
				xbox_controller = NULL;
			}
			else
			{
				std::cout << "- Connected to a XBox controller: battery level=";

				switch (xbox_controller->GetBatteryState().BatteryLevel)
				{
				case BATTERY_LEVEL_EMPTY: std::cout << "empty"; break;
				case BATTERY_LEVEL_LOW: std::cout << "low"; break;
				case BATTERY_LEVEL_MEDIUM: std::cout << "medium"; break;
				case BATTERY_LEVEL_FULL:  std::cout << "full"; break;
				}
				cout << endl;
			}
		}
#endif
        else{
            if(fileIsReadable(argv[i])) m_filename=argv[i];
        }
    }

}

void Shepherding::run()
{   
    if(m_sim==NULL || m_shepherd==NULL)
    {
        cerr << "! Error: Simulator or Shepherd is undefined" << endl;
        exit(1);
    }

    // Setup simulation globals.
    // todo : this should be from config file...
    m_sim->setTimeStep( 0.05 );
    m_sim->setRestitution( 0.1 );

    if (!m_doDrawShepherd)
        makeFlockInvisible(m_shepherd);
        
    // Start simulation.
    m_sim->simulate();
}

void Shepherding::stop()
{
    // stop simulation.
    m_sim->stopsimulate();
}

//
void
Shepherding::
makeFlockMemberInvisible(CFlock * flock, int flockNum)
{
    if (!flock) return;

    CFlockState & state = flock->getState(flockNum);
    state.disableDraw();
}


void
Shepherding::
makeFlockInvisible(CFlock * flock)
{
    if (!flock)
        return;

    for (int i = 0; i < flock->getStateSize(); i++) {
        CFlockState & state = flock->getState(i);
        state.disableDraw();
    }
}


void Shepherding::
showHelp(ostream & os, const char * prog)
{
    os << "  OTHER OPTIONS: \n" 
       << "    -g                    disable rendering\n"
       << "    -seed N               set random number generator with N\n" 
       << "    -budget N             set simulation budget to N sim steps\n"
       << "    -mouse                read shepherd's position from position of mouse relative to window [default]\n"
#if USE_WIIMOTE
       << "    -wiimote              read shepherd's position from wiimote(s)\n" 
#endif
#if _WIN32
	   << "    -xbox                 read shepherd's position from xbox controller(s)\n"
#endif
       << "    -client               read shepherd's position from DeviceState server\n" 
       << "    -drawshepherd         draw shepherd robot [default]\n"
       << "    -no-drawshepherd      disable drawing of shepherd (but still simulate it)\n" 
       << "    -num-shepherds  N     create N shepherd robots if target simulation supports it [default is 1]\n" 
       << "    -enable-autonomy      allow the shepherds to control themselves instead of relying on human control [default]\n" 
       << "    -disable-autonomy     don't allow the shepherds to control themselves\n"
       << "    -enable-log           allow the logger to record shepherd/flock positions in a file\n"
       << "    -mixed-autonomy       user can switch autonomous behavior on and off\n" 
       << "    -use-direct-position  set the position of the shepherd directly\n" 
       << "    -use-force-position   set the position of the shepherd indirectly via forces applied to them [default]\n" 
       << "    -show-visual-hints    display visual hints\n" 
       << flush;
}


// Returns true if a file is readable, false otherwise.
// Absolute and relative paths are both acceptable.
bool Shepherding::fileIsReadable(string filename)
{
    ifstream fin(filename.c_str());
    bool good=fin.good();
    fin.close();
    return good;
}


