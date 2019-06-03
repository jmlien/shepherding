#include "main.h"

//-------------------------------------------------------------------
bool parseArg(int argc, char ** argv)
{
    for(int i=1;i<argc;i++){
        if(argv[i][0]=='-'){
            switch(argv[i][1]){
                case 'g': b_disableGL=true; break;
            }//end switch
        }
        else{
            strFilename=argv[i];
        }
    }//end for

    if(strFilename.empty()) return false;
    return true;
}

//-------------------------------------------------------------------
int main(int argc, char ** argv)
{
    //parse the argument
    if(!parseArg(argc,argv)){
        cerr<<"Usage: "<<argv[0]<<" [-g] *.mp"<<endl;
        return 1;
    }

    cout<<"- Initialize"<<endl;

    bool suc=getMP()->initialize(strFilename);
    if(!suc){  //failed
        cerr<<"! Epic Fail !"<<endl;
        return 1;
    }

    suc=getMP()->findPath();

    if(suc)
    {
        //cout<<"\n- Path found"<<endl;
    }
    else{
        //cout<<"\n- Path NOT found"<<endl;
        return 0;
    }

    if(!b_disableGL)
    {
#if GL_ON
        gSolver.setEnvironment(getEnvironment());

        ///////////////////////////////////////////////////////////////////////
        //Init glut
        int argc=1; char * argv[1];
        char shep[10] = "shepherd";
        argv[0] = shep;
        glutInit( &argc, argv );
        glutInitDisplayMode( GLUT_RGB|GLUT_DOUBLE|GLUT_DEPTH );
        glutInitWindowPosition(0,0);
        glutInitWindowSize( windowW, windowH );
        glutCreateWindow( "shepherding" );

        ///////////////////////////////////////////////////////////////////////
        //Init SH displaying env.
        Init();
        gli::gliInit();
        glutReshapeFunc(Reshape);
        glutKeyboardFunc(replay_keyboard);
        glutPassiveMotionFunc(PassiveMotion);
        gli::gliMotionFunc(Motion);
        gli::gliMouseFunc(Mouse);
        gli::gliDisplayFunc(Display);
        isGlutInitialized = true; // solves issues with glutPostRedisplay() being
                                  // called before glutInit, which causes a
                                  // runtime error
        show3D();          // toggle to 2d
        showHeadingDir();  //toggle heading
        showTxt();
        //showBBX();       // toggle the bounding box + black background
		addDrawObj(&drawTarget);
		addDrawObj(&drawTreeMap);
		if( getMP()->getGraph()!=NULL )
			addDrawObj(&drawGraphMap);

        toFlockState(getMP()->getQuery()->getQuery(0));

        ///////////////////////////////////////////////////////////////////////
        gli::gliMainLoop();
#endif
    }//end b_disableGL

    return 0; /// everything is fine
}

void replay()
{
#if GL_ON
    if(!Displayed) return; //not displayed yet
	if(getMP()->getPath()->getPath().empty()) return; //empty path...

    static unsigned int cur_path_cfg_id=0;
    static int time_step_left=0;

    if(cur_path_cfg_id==0){
        //reset the system to inital state
        toFlockState(getMP()->getPath()->getPath()[0]);
    }

    if(time_step_left==0){
        cur_path_cfg_id++;
        const vector<Cfg>& path=getMP()->getPath()->getPath();
        if(path.size()==cur_path_cfg_id){//should stop
            cur_path_cfg_id=0;
            CurTimeStep=0;
            startReplay(); //stop replay
            return;
        }

        //get necessary data
        FSLIST& shepherds=getMP()->getShepherds();
        const Cfg& cfg=path[cur_path_cfg_id];
		//set time step
        time_step_left=cfg.m_sim_time_steps;
		//if(cfg.m_sim_time_steps==0) return; //? why would it be 0?

        TreePMP * tree=getMP()->getTree();
		//if(tree==NULL) return;

        //prepare for simulation
        /*if( isClass(tree,"nb_rt_RRT") || isClass(tree,"nb_rm_RRT") ){
            //const Cfg& cfg2=path[cur_path_cfg_id-1];
            //cout<<"sup pos="<<cfg2.m_shepherd_pos.back()<<endl;
            //set all target positions
            typedef vector<Point2d>::const_iterator IT;
            FSLIST::iterator sh_it=shepherds.begin();
            for(IT i=cfg.m_shepherd_tar.begin();i!=cfg.m_shepherd_tar.end();i++,sh_it++){
                CHerdingFlockState * s=(CHerdingFlockState*)(*sh_it);
                s->target=*i; //set s' target to i
            }
			//draw targets
			drawTarget.tagets=cfg.m_shepherd_tar;
        }
        else*/{  //there are behavior based

			//
			//const Cfg& cfg2=path[cur_path_cfg_id-1];
			//Cfg now;
			//fromFlockState(now);
			//cout<<"["<<CurTimeStep<<"] sup="<<cfg2.m_flock_cen
			//    <<" now="<<now.m_flock_cen<<" next time="<<CurTimeStep+time_step_left<<endl;

            const Point2d& goal=cfg.m_flock_tar;
            FSLIST::iterator sh_it=shepherds.begin();
            for(;sh_it!=shepherds.end();sh_it++){
                CHerdingFlockState * s=(CHerdingFlockState*)(*sh_it);
                s->goal=cfg.m_flock_tar; //set s' goal to i
            }
        }
    }

    gSolver.updateState();
    Displayed=false;
    CurTimeStep++;
    time_step_left--;
#endif
}

//void TimerCallback(int value)
//{
//    if( value==0 || !Simulating ) return;
//    in simuation state
//    glutPostRedisplay();
//    glutTimerFunc(30, TimerCallback, value);
//}

void startReplay()
{
#if GL_ON
    int index=Simulation_Event;
    if(!Simulating){ //if not start simu
        //glutChangeToMenuEntry(index, "Pause Simulation", index);
        //Get time
        glutTimerFunc(0, TimerCallback, 1);
        glutIdleFunc(replay);
    }
    else{
        //glutChangeToMenuEntry(index, "Resume Simulation", index);
        glutTimerFunc(0, TimerCallback, 0);
        glutIdleFunc(NULL);
    }

    Simulating=!Simulating;
#endif
}

void replay_keyboard( unsigned char key, int x, int y ){
#if GL_ON
	int size;
    switch( key ){
        case 27 : exit(0);
        case ' ': startReplay();      break;
        case '1': show3D();           break;
        case '2': showViewingRange(); break;
        case '3': showHeadingDir();   break;
        case '4': showFlockID();      break;
        case '5': showBBX();          break;
        case 't': showTxt(); 	      break;
		case 'g': drawGraphMap.m_disable=!drawGraphMap.m_disable; break;
        case '[': if (drawTreeMap.drawStopID > 0) drawTreeMap.drawStopID--;     break;
        case ']':
        	size = getMP()->getRM()->getVector().size();
        	if (drawTreeMap.drawStopID < size)
        		drawTreeMap.drawStopID++;
        	break;
        case '\\':
        	size = getMP()->getRM()->getVector().size();
        	if ((drawTreeMap.drawStopID >= size) || (drawTreeMap.drawStopID < 0))
        		drawTreeMap.drawStopID = 0;
        	else
        		drawTreeMap.drawStopID = size;
        	break;
        default:
        	//printf("key: %c : %02X\n", key, key);
        	break;
    }

    if (isGlutInitialized)
        glutPostRedisplay();
#endif
}


