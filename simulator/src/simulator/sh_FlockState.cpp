#include "sh_FlockState.h"
#include "sh_Environment.h"
#include "sh_CollisionDetection.h"
#include "model/ModelFactory.h"

#include <Gauss.h>
#include <cassert>

int CFlockState::TOTAL_STATE_SIZE=0;
int CFlock::TOTAL_FLOCK_SIZE=0;

CFlockState::CFlockState(CFlock * type)
{
    memset(m_R,0,sizeof(float)*9); //rotation matrix
    m_Y=0; //rotation
    m_Type=type;
    m_ID=TOTAL_STATE_SIZE;
    TOTAL_STATE_SIZE++;
    m_SeeObstacle=false;
    m_BR=NULL;
    m_FR=NULL;
    m_Color.set(1,1,1);
    m_drawEnabled = true;
}

CFlockState::~CFlockState(){}

CBehaviorRule * CFlockState::getBehaviorRule()
{
    return (m_BR==NULL)?m_Type->getBehaviorRule():m_BR;
}

CForceRule * CFlockState::getForceRule()
{
    return (m_FR==NULL)?m_Type->getForceRule():m_FR;
}

CFlock::CFlock(CEnvironment * env, const string& name, int size)
{
    //read model
    IModel * model=CreateModelLoader(name);
    initialize(env,model,size);
}

CFlock::CFlock(CEnvironment * env, IModel * model, int size)
{
	initialize(env,model,size);
}

void CFlock::initialize(CEnvironment * env, IModel * model, int size)
{
    m_pEnv=env;
    m_ForceRule=NULL;
    m_BehaviorRule=NULL;
    m_Mass=1;
    m_ViewAngle=(float)(2*PI);
    m_ViewRadius=10;
    m_CheckCDforView=false;
    m_ID=TOTAL_FLOCK_SIZE;
    TOTAL_FLOCK_SIZE++;
    m_State.reserve(size);
    m_Model=NULL;
    m_Color.set(1,1,1);

    //read model
    m_Model=model;
    if( m_Model==NULL ){ cerr<<"! ERROR: Create Model Failed"<<endl; throw; }
    if( m_Robot.buildCDModel(*m_Model)==false )
    { cerr<<"! ERROR: Create CD Model Failed"<<endl; throw; }
}

CFlock::~CFlock()
{
    m_State.clear();
}

CFlockState& CFlock::addState()
{
    CFlockState * s=new CFlockState(this);
    addState(s);
    return *s;
}

void CFlock::addState(CFlockState * s){
    if(s==NULL) return;
    s->updateRot_using_V(); //user may setup pos and vec but not rot
    s->setColor(m_Color[0],m_Color[1],m_Color[2]);
    m_State.push_back(s);
}

//depoly gaussian distribution
void CFlock::deployFlock
(RNG * rng,const Point2d& center,float dev,float vm,Vector2d dir)
{
    typedef vector<CFlockState*>::iterator FIT;
    if( m_State.empty() ) cerr<<"! Warning : No state added"<<endl;
    m_pEnv->getBBX().createCSPace(getGeometry().getRadius());
    shCD CD(m_pEnv); //collision detector

    bool dir_given=(dir.normsqr()==0)?false:true; //the dir is given
    for( FIT iI=m_State.begin();iI!=m_State.end();iI++ ){
        CFlockState& s=**iI;
        Point2d pos;
        do{
            pos.set(rng->gauss(dev)+center[0],rng->gauss(dev)+center[1]);
        }
        while( CD.isCollision(m_Robot,pos) );

        s.setPos(pos);

        //compute rotation & vel dir
        if( dir_given ) {
            s.setVelocity(dir.normalize()); //prevent that vm is very small
            s.updateRot_using_V();
            s.setVelocity(dir*vm);
        }
        else{
            float y=fmod((float)rng->gauss(dev),PI*2);
            s.setVelocity(Vector2d(sin(y),cos(y))*vm);
            s.updateRot_using_V();
        }
    }//end for
}

void CFlock::createTrueProject()
{
    if( m_Robot.buildTrueCDModel(*m_Model)==false )
    { cerr<<"! ERROR: Create True Projection Failed"<<endl; throw; }
}



