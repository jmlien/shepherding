
#include "shepherding.h"
#include "simple_herding.h"
#include "scared_flock.h"
#include "simple_herding_rules.h"
#include "simple_herding_draw.h"

CSimpleHerdingFlock::CSimpleHerdingFlock
(CEnvironment * env, const string& name):CHerdingFlock(env,name)
{    
    //do nothing
}


bool CSimpleHerdingFlock::initialize(list< list<string> >& tokens, flock_raw_data& data)
{
    //additional data
    float target_attract=10;
    int roadmap_n = 80, roadmap_k = 5;
    bool goal_defined=false;
    CSimpleHerdingGoal goal; 
    
    //additional parsing
    for(list< list<string> >::iterator i=tokens.begin();i!=tokens.end();i++){
        //
        string label=i->front();
        i->pop_front();
        //
        if(label=="target_attract"){
            if(i->empty()){ cerr<<"! ERROR: shepherd.target_attract has no value"<<endl; return false; }
            target_attract=atof(i->front().c_str());
        }
        else if(label=="roadmap_n"){
            if(i->empty()){ cerr<<"! ERROR: shepherd.roadmap_n has no value"<<endl; return false; }
            roadmap_n=atoi(i->front().c_str());
        }
        else if(label=="roadmap_k"){
            if(i->empty()){ cerr<<"! ERROR: shepherd.roadmap_k has no value"<<endl; return false; }
            roadmap_k=atoi(i->front().c_str());
        }
        else if(label=="goal")
        {
            if(i->size()<2){ cerr<<"! ERROR: shepherd.goal has no value"<<endl; return false; }
            float x=atof(i->front().c_str());
            string& y_str=*(++(i->begin()));
            float y=atof(y_str.c_str());
            goal.setPosition(Point2d(x,y));
            
            if(i->size()==2){ //if no polygon given use a circle
                goal.setRadius(5); //TODO: need a way to fix this
                goal.setCenter(Point2d(x,y));
            }
            else{
                //this will build the center and radius from
                //the given polygon
                goal.build(i->back());
            }
            goal_defined=true;
        }
    }//end for

    //check if goal is defined
    if(!goal_defined){
        cerr<<"! Shepherd's goal is not defined"<<endl; 
        return false;
    }
    
    //this setups the basic part of the shepherd type
    data.setupBasicFlock(this);

    //create force rule
    CSimpleHerdingForceRule * frule = new CSimpleHerdingForceRule();
    data.setupBasicForceRule(frule);
    frule->TargetAttract=target_attract;
    setForceRule(frule);
    
    //create behavior rule
    setBehaviorRule(new CSimpleHerdingBehaviorRule());

    //create roadmap
    CRoadMap * map=new CRoadMap(this,roadmap_n);
    assert(map);
    
    string rmap_filename=getRoadmapCatchName(getEnvironment(),roadmap_n,roadmap_k);
    if(map->read(rmap_filename)==false)
    {
        PRMS prm(getEnvironment(),getRNG());
        prm.samplePRMNodes(*map,roadmap_n/3);
        prm.sampleMAPRMNodes(*map,roadmap_n/3);
        prm.sampleOBPRMNodes(*map,roadmap_n/3);
        prm.connectNodes(*map,roadmap_k);
        prm.simplifyMap(*map);
        map->save(rmap_filename); //save to file
    }
    
	//it is possible that this is an empty herding 
	if (data.size == NULL) return true;

    //create flock state
    for( int i=0;i<data.size;i++){
        CHerdingFlockState * s=new CHerdingFlockState(this,map);
        s->goal=goal;
        addState(s);
    }

    // Deploy in environment
    deployFlock( getRNG(), data.position, data.scatteredness );
    getEnvironment()->addFlock(this);    

    //create draw objects
    //drawing goal
    sh_Draw * renderer=getSimulator()->getRenderer();
    if(renderer!=NULL){
        glDrawGoal * dgoal=new glDrawGoal();
        assert(dgoal);
        if(renderer->isCompatible(dgoal)){
            dgoal->goal=goal;
            renderer->addDrawObj(dgoal);
        
            if(getSI()->showVisualHint())
			{
                //draw roadmap
                glDrawRoadMap * dmap=new glDrawRoadMap(*map);
                assert(dmap);
                dmap->setColor(Point3d(1,1,0));
                renderer->addDrawObj(dmap);
                
                //this is for drawing groups 
                dgroup=new glDrawGroups(this);
                assert(dgroup);
                renderer->addDrawObj(dgroup);
            
                //this is for drawing flock path
                dpath=new glDrawMovingPath(this);
                assert(dpath);
                renderer->addDrawObj(dpath);
            
                //this is for drawing milestones
                dmile=new glDrawMilestones(this);
                assert(dmile);
                renderer->addDrawObj(dmile);
            }//if visual hints
        }// if compatible
    }//if renderer!=NULL
    
    return true;
}


string CSimpleHerdingFlock::getRoadmapCatchName(CEnvironment * env, int n, int k)
{
    list<CObs*>& obsts=env->getObstacles();
    int facet_size=0;
    int vertex_size=0;
    for(list<CObs*>::iterator i=obsts.begin();i!=obsts.end();i++)
    {
        CObs* obst=*i;
        vertex_size+=obst->getGeometry().getGeo().size();
        facet_size+=obst->getGeometry().getTri().size();
    }

    Point3d first_vertex=obsts.back()->getGeometry().getGeo().front();
    Point3d last_vertex=obsts.front()->getGeometry().getGeo().back();

    char name[256];
    sprintf(name,".rmap-%d%d%.2f%.2f%d%d",facet_size,vertex_size,first_vertex[0],last_vertex[0],n,k);
    return name;
}
