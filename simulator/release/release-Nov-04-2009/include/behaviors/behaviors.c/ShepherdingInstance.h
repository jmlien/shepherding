#ifndef SHEPHERDING_INSTANCE_H
#define SHEPHERDING_INSTANCE_H

#include "sh_gui_main.h"
#include "ShepherdingParser.h"
#include "sh_Robot2D.h"

void beginPollingThread();
void updatePositionFromServer(std::string serialState);
void updatePositionFromMouse(std::string serialState);

class ShepherdingInstance
{
    public:
        ShepherdingInstance();
		//don't know how to initialize this
		virtual bool initialize(list< list<string> >& tokens, flock_raw_data& data)=0; 
        virtual void run(int argc, char * argv[]);
    
        void beginPollingThread();
        void updatePositionFromServer(std::string serialState);
        void updatePositionFromMouse();
        void updateMultipleFromMouse();
#if USE_WIIMOTE
        void updatePositionFromWiimote();
#endif
        static void showHelp(ostream & os, const char * prog);
 
 
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


    protected:
        

		virtual void parseCmdline(int argc, char * argv[]);

	    //position functions
        bool setFlockMemberPosition( CFlock * flock, int memberNum, float targetX, float targetZ );
        void makeFlockMemberInvisible(CFlock * flock, int flockNum);
        void makeFlockInvisible(CFlock * flock);

        Point2d getNextFreePosition( CEnvironment & environment, CRobot2D & robot,
                                     Point2d oldPosition, Point2d goalPosition );
        //Point2d getLastPositionBeforeObstacle( CEnvironment & environment, CRobot2D & robot,
        //                                       Point2d oldPosition, Point2d goalPosition );
        bool isInBounds( CEnvironment * environment, Point2d & position );
        Point2d getAbsoluteTargetPosition(float relativeTargetX, float relativeTargetZ, bool * wasError);
        
        bool setShepherdTargetForce(CFlock * flock, int memberNum, float targetX, float targetZ );

        // MEMBER OBJECTS
        bool m_doUpdateFromServer;
        bool m_doUpdateFromMouse;
        bool m_doUpdateFromWiimote;
        bool m_doDrawShepherd;
        bool m_allowAutonomy;
        bool m_mixedAutonomy;
        bool m_useDirectPosition;
		bool m_showVisualHints;
		bool m_allowMA;              //default is true
		bool m_allowRegroup; 
		bool m_checkReachGoal;
        
		CFlock * m_shepherd;         //pointer to the shepherds
};

ShepherdingInstance * getSI(); //get the shepherding instance


#endif
