
//--------------
//
// Cfg
//
//--------------

#include "mp.h"

Cfg::Cfg()
{
    const int& dof=getMP()->getDOF();
    m_value=new float[dof];
	for(int i=0;i<dof;i++) m_value[i]=0; //set all to 0s
	m_shepherd_pos.reserve(getMP()->getShepherds().size());
	m_shepherd_tar.reserve(m_shepherd_pos.size());
	m_shepherd_ss.reserve(m_shepherd_pos.size());

	m_flock_rad=0;
	m_sim_time_steps=0;
	m_in_map=false;
	m_cls_vid=-1;
}

Cfg::Cfg(const Cfg& c)
{
    const int& dof=getMP()->getDOF();
    m_value=new float[dof];
    memcpy(m_value,c.m_value,sizeof(float)*dof);
	m_shepherd_pos=c.m_shepherd_pos;
	m_shepherd_tar=c.m_shepherd_tar;
	m_shepherd_ss=c.m_shepherd_ss;

	m_flock_rad=c.m_flock_rad;
	m_flock_cen=c.m_flock_cen;
	m_flock_tar=c.m_flock_tar;
	m_flock_dir=c.m_flock_dir;
	m_sim_time_steps=c.m_sim_time_steps;
	m_in_map=c.m_in_map;
	m_cls_vid=c.m_cls_vid;
}

const Cfg& Cfg::operator=(const Cfg& other)
{
    const int& dof=getMP()->getDOF();
    memcpy(m_value,other.m_value,sizeof(float)*dof);
	m_shepherd_pos=other.m_shepherd_pos;
	m_shepherd_tar=other.m_shepherd_tar;
	m_shepherd_ss=other.m_shepherd_ss;

	m_flock_rad=other.m_flock_rad;
	m_flock_cen=other.m_flock_cen;
	m_flock_tar=other.m_flock_tar;
	m_flock_dir=other.m_flock_dir;
	m_sim_time_steps=other.m_sim_time_steps;
	m_in_map=other.m_in_map;
	m_cls_vid=other.m_cls_vid;
	//m_rand_seed=other.m_rand_seed;

    return *this;
}

bool Cfg::operator==(const Cfg& other) const
{
    const int& dof=getMP()->getDOF();
    for(int i=0;i<dof;i++)
        if( m_value[i]!=other.m_value[i] ) return false;
    return true;
}

ostream& operator<<(ostream& out, const Cfg& c)
{
    const int& dof=getMP()->getDOF();
    for(int i=0;i<dof;i++)
        out<<c[i]<<" ";
    return out;
}

//--------------
//
// Cfg Functions
//
//--------------

void fromFlockState(Cfg& c)
{
	//create Cfg from flock state
	FSLIST& states=getEnvironment()->getFlockStates();
	int index=0;
	for(FSLIST::iterator i=states.begin();i!=states.end();i++){
		CFlockState*s=*i;
		const Point2d& pos=s->getPos();
		const Vector2d& vel=s->getVelocity();
		c[index++]=pos[0];
		c[index++]=pos[1];
		c[index++]=vel[0];
		c[index++]=vel[1];
	}//end for


	//other stuff
	//get shepherd positions
	c.m_shepherd_pos.clear();
	c.m_shepherd_ss.clear();
	FSLIST& sh=getMP()->getShepherds();
	for(FSLIST::iterator i=sh.begin();i!=sh.end();i++){
		CHerdingFlockState *shepherd=(CHerdingFlockState*)*i;
		c.m_shepherd_pos.push_back(shepherd->getPos());
		c.m_shepherd_ss.push_back(shepherd->swing_step);
	}


	//get flock radius/center
	pair<float, Point2d> rc=findEC(getMP()->getFlock());
	c.m_flock_rad=rc.first;
	c.m_flock_cen=rc.second;

	//get closest node
	CMapFlockState * shepherd=(CMapFlockState *)getMP()->getShepherds().front();

	c.m_cls_vid=shepherd->getMap()->closestNode(c.m_flock_cen);

}

void toFlockState(const Cfg& c)
{
	//create Cfg from flock state
	FSLIST& states=getEnvironment()->getFlockStates();
	int index=0;
	Point2d pos;
	Vector2d vel;
	for(FSLIST::iterator i=states.begin();i!=states.end();i++){
		CFlockState*s=*i;
		pos[0]=c[index++];
		pos[1]=c[index++];
		vel[0]=c[index++];
		vel[1]=c[index++];
		s->setPos(pos);
		s->setVelocity(vel);
		s->updateRot_using_V();
	}//end for

	FSLIST& sh=getMP()->getShepherds();
	int id=0;
	for(FSLIST::iterator i=sh.begin();i!=sh.end();i++){
		CHerdingFlockState *shepherd=(CHerdingFlockState*)*i;
		shepherd->swing_step=c.m_shepherd_ss[id++];
	}
}


Cfg operator+(const Cfg& c1, const Cfg& c2)
{
    Cfg cfg;
    const int& dof=getMP()->getDOF();
    //-------------------------------------------------
    for(int i=0;i<dof;i++) cfg[i]=c1[i]+c2[i];
    return cfg;
}

Cfg operator-(const Cfg& c1, const Cfg& c2)
{

    Cfg cfg;
    const int& dof=getMP()->getDOF();
    for(int i=0;i<dof;i++) cfg[i]=c1[i]-c2[i];
    return cfg;
}

Cfg operator*(const Cfg& c1, float s)
{
    Cfg cfg;
    const int& dof=getMP()->getDOF();
    for(int i=0;i<dof;i++) cfg[i]=c1[i]*s;
    return cfg;
}

Cfg operator/(const Cfg& c1, float s)
{
    Cfg cfg;
    const int& dof=getMP()->getDOF();
    for(int i=0;i<dof;i++) cfg[i]=c1[i]/s;
    return cfg;
}

float normsqr(Cfg& c)
{
    const int& dof=getMP()->getDOF();
    //---------------------------------------------------
	float v=0;
	for(int i=0;i<dof;i++) v+=(c[i]*c[i]);
	return v;
}

float dot(Cfg& c1, Cfg& c2)
{
    const int& dof=getMP()->getDOF();
    //---------------------------------------------------
	float v=0;
	for(int i=0;i<dof;i++) v+=(c1[i]*c2[i]);
	return v;
}

