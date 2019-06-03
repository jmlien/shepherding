/*
 * shepherding.h
 *
 *
 * This file contains a class called "Shepherding"
 *
 * class Shepherding: is a master class that controls the shepherding simulation 
 * There are serval types of controls. Mostly, users can control the simulation
 * from the command line. Users can also control the simulation from
 * different user interfaces, including mouse and Wiimote.
 *
 * It should be easy to modify this class for other types of simulation.
 *
 * Last Major Modification : J-M Lien 12/28/2009
 * 
 */


#ifndef SHEPHERDING_INSTANCE_H
#define SHEPHERDING_INSTANCE_H

#include "shepherding_base.h"
#include "ws_parser.h"
#include "herding_flock.h"

#ifdef _WIN32
#include "CXBOXController.h"
#endif

class Shepherding
{
    public:

        Shepherding();
        
        //setup from command line
        virtual void parseCmdline(int argc, char * argv[]);
        
        //run simulation
        virtual void run();
        
        //stop simulation
        virtual void stop(); 
       
        //access functions
        shSimulate * getSimulator() { return m_sim; }
        void setSimulator(shSimulate * sim) { m_sim=sim; }
        
        CHerdingFlock * getShepherd(){ return m_shepherd; }
        void setShepherd(CHerdingFlock * s){ m_shepherd=s; }
        
        //thread realated functions
        void beginPollingThread();
        void updatePositionFromServer(string serialState);
        void updatePositionFromMouse();
        void updateMultipleFromMouse();

#if USE_WIIMOTE
        void updatePositionFromWiimote();
#endif

#ifdef _WIN32
		//xbox controller
		CXBOXController* getXBoxController(){ return xbox_controller; }
#endif
 
        //some help functions here
        bool allowAutonomy() const { return m_allowAutonomy; }
        
        bool showVisualHint() const {return m_showVisualHints; }
        void disableVisualHint() { m_showVisualHints=false;}
        void enableVisualHint() {m_showVisualHints=true;}
        
        bool allowMedialAxis() const { return m_allowMA; }
        void disableMedialAxis() { m_allowMA=false;}
        void enableMedialAxis() {m_allowMA=true;}
        
        bool allowRegroup() const { return m_allowRegroup; }
        void disableRegroup() { m_allowRegroup=false; }
        void enableRegroup() { m_allowRegroup=true; }

        bool checkReachGoal() const { return m_checkReachGoal; }
        void disableReachGoal() { m_checkReachGoal=false; }
        void enableReachGoal() { m_checkReachGoal=true; }
        
        bool checkLog() const { return m_allowLog; }
        void disableLog() { m_allowLog=false; }
        void enableLog() { m_allowLog=true; }
        
        bool allowDraw() const { return m_doDraw; }
        void disableDraw() { m_doDraw=false; }
        void enableDraw() { m_doDraw=true; }        
        
        void markSucceeded() {  m_succeeded=true; }
        bool checkSucceeded() const{ return m_succeeded; }
        
        long getSeed() const { return m_RNG_seed; }
        
        long getSimBudget() const { return m_sim_budget; }
        
        string getConfigFileName() const { return m_filename; }
        
        //active shepherd via gui
        void setActiveShepherdID(int id){gui_active_shepherd=id;}
        void setActiveShepherdPos(Point2d pos){ gui_active_shepherd_position=pos; }
        
        void showHelp(ostream & os, const char * prog);

    protected:

        //position functions
        bool setFlockMemberPosition( CFlock * flock, int memberNum, float targetX, float targetZ );
        void makeFlockMemberInvisible(CFlock * flock, int flockNum);
        void makeFlockInvisible(CFlock * flock);

        Point2d getNextFreePosition( CEnvironment & environment, CRobot2D & robot,
                                     Point2d oldPosition, Point2d goalPosition );
        bool isInBounds( CEnvironment * environment, Point2d & position );
        Point2d getAbsoluteTargetPosition(float relativeTargetX, float relativeTargetZ, bool * wasError);
        
        bool setShepherdTargetForce(CFlock * flock, int memberNum, float targetX, float targetZ );
        
        bool fileIsReadable(string filename);

        // MEMBER OBJECTS
        bool m_doUpdateFromServer;
        bool m_doUpdateFromMouse;
        bool m_doUpdateFromWiimote;
        bool m_doDrawShepherd;
        bool m_allowAutonomy;
        bool m_mixedAutonomy;
        bool m_allowLog;
        bool m_useDirectPosition;
        bool m_showVisualHints;
        bool m_allowMA;              //default is true
        bool m_allowRegroup; 
        bool m_checkReachGoal;
        bool m_doDraw;               //turn on/off drawing 
        long m_RNG_seed;             //RNG seed
        long m_sim_budget;           //simulation budget
        bool m_succeeded;            //flag marked true if shepherding succeeded
        string m_filename;           //shepherding filename
        
        shSimulate    * m_sim;              //the simulator
        CHerdingFlock * m_shepherd;         //pointer to the shepherds
        
        //active shepherd (that is selected by user via GUI)
        int gui_active_shepherd;
        Point2d gui_active_shepherd_position; 

#ifdef _WIN32
		//xbox controller
		CXBOXController* xbox_controller;
#endif
};

//
// global access functions
// 
// note: only shepherding is a singleton
//       environment and rng are part of the singleton.
//       these functions are provided here for easy access 
//       should find a way to remove them in the future.
//
Shepherding * getSI();         // get the shepherding instance
void setSI(Shepherding * si);  // set the shepherding instance
shSimulate * getSimulator();           // get simulator
CEnvironment * getEnvironment();       // get environment
RNG * getRNG();                        // get random number generator

#endif
