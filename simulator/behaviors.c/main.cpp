/*
 * main.cpp
 *
 * 
 *
 * Entry point of the shepherding behavior simulation 
 * 
 * 
 */


#include "shepherding_base.h"
#include "shepherding_behaviors.h"

void showHelp(ostream & os, const char * prog)
{
    os << "usage: " << prog << " *.sh" << endl;
}


int main(int argc, char * argv[])
{
    //create a shepherding instance (SI) 
    setSI(new Shepherding());
    assert(getSI());
    //from now on, SI can be obtained from getSI()
    
    if(argc < 2){
        showHelp(cerr,argv[0]);
        getSI()->showHelp(cerr, argv[0]);
        return 1;
    } //something went wrong
    
    //setup from command line
    getSI()->parseCmdline(argc, argv);

    //get workspace file
    const string& configFile = getSI()->getConfigFileName();
    if (configFile.empty()) 
    {
        showHelp(cerr,argv[0]);
        getSI()->showHelp(cerr, argv[0]);
        return 1;
    }

    //create a simulator
    shSimulate * sim=(getSI()->allowDraw())?new shepherding_gui():new shSimulate();
    assert(sim);
    getSI()->setSimulator(sim);
    
    //create env
    getSimulator()->setEnvironment(new CEnvironment()); 
    assert(getSimulator()->getEnvironment());
    
    //create solver
    getSimulator()->setSolver(new CParticleSolver(getEnvironment()));
    assert(getSimulator()->getSolver());


    //check if logger should be created 
    if(getSI()->checkLog()){
        SimLogger * logger = new SimLogger(getSimulator());
        assert(logger);
        getSimulator()->setLogger(logger);
    }
    
    //set up random number generation
    cout<<"- RNG seed: "<<getSI()->getSeed()<<endl;
    RNG * rng = new P_RNG(getSI()->getSeed());
    getSimulator()->setRNG(rng);
    
    //parse the workspace file
    WSParser parser;
    if(!parser.parse(configFile)) return 1;

    //initialize logger
    if(getSimulator()->getLogger()!=NULL) 
        getSimulator()->getLogger()->InitLog(configFile);
        
    //initialize the simulator with everything that we have loaded to the simulator
    getSimulator()->initialize();

    //everything seems ok, start to run
    getSI()->run();
    
    //close logger if needed
    if(getSimulator()->getLogger()!=NULL)
    {
        if( getSI()->checkSucceeded() ) 
            (*getSimulator()->getLogger())<<-1.0f; //so we know we've succeeded
        getSimulator()->getLogger()->CloseLog();
    }
    
    return 0;
}


