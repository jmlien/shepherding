#include "mp.h"

//for qhull
extern "C"{
char qh_version[] = "xxx";
}

//singleton
MP g_mp;
MP * getMP() { return &g_mp; }

//-----------------------------------------------------------------------------

MP::~MP() {
	if(m_tree)
		delete m_tree;
	for(list<LP*>::iterator i = m_lps.begin();
		i != m_lps.end(); i++) {
		if(*i)
			delete (*i);
	}
	if(m_dm)
		delete m_dm;
	if(m_rng)
		delete m_rng;
}

bool MP::initialize(const string& file)
{
    m_in_mpfile=file;
    ifstream fin(file.c_str());
    if(!fin.good()){
        cerr<<"! Error: Cannot open file:"<<file<<endl;
        return false;
    }//end good
    bool r=initialize(fin);
    fin.close();
    return r;
}

//read from file and build MP
bool MP::initialize(istream& in)
{
    const int size=1024;
    char * tmp=new char[size];
	m_tree=NULL;
	m_graph=NULL;

    long seed = time(NULL);
    string gen;
    string t_wkspace_filename;

    while(!in.eof()){
        in.getline(tmp,size);
        list<string> tok=tokenize(tmp," \t[]()<>,=");
        if(tok.empty()) continue;

        string label=tok.front();
        tok.pop_front();
        if(label[0]=='#') continue; //comment

        //create stuff
        if(label=="tree" || label=="Tree" || label=="TREE")
        {
        	m_tree=TreePMP::create(tok);
        	if(m_tree==NULL) return false;
        }
		else if(label=="graph" || label=="Graph" || label=="GRAPH")
		{
			m_graph=GraphPMP::create(tok);
            if(m_graph==NULL) return false;
		}
        else if(label=="lp" || label=="LP")
        {
            LP * lp=LP::create(tok);
            if( lp==NULL ) return false;
            m_lps.push_back(lp);
        }
        else if(label=="dm" || label=="DM" )
        {
            m_dm=DM::create(tok);
            if( m_dm==NULL ) return false;
        }
        else if(label=="wkspace" || label=="WKSpace" || label=="WKSPACE")
        {
        	t_wkspace_filename = tok.front();
        	tok.pop_front();
        }
        else if (label=="seed" || label=="Seed" || label=="SEED")
        {
        	seed = atoi(tok.front().c_str());
        }
        else if (label=="rng" || label=="RNG")
        {
        	gen = tok.front();
        }
        else
            cerr<<"! Warning: Ignore unknown values: "<<tmp<<endl;
    }

    // do the RNG seeding first.
    if(gen == "STDLIB" || gen == "stdlib") {
    	cout << "- RNG: STDLIB with seed " << seed << endl;
    	m_rng = new STDLIB_RNG(seed);
    } else {
    	cout << "- RNG: GSL with seed " << seed << endl;
    	m_rng = new P_RNG(seed);
    }

    //initialize shepherding here ----------------------------
	setEnvironment(new CEnvironment()); //this does nothing in fact...
	getEnvironment()->setRNG(m_rng);
	// Setup simulation globals. (shouldn't this from file..)
	setTimeStep( 0.05 );
	setRestitution( 0.95 );
	//parse
	bool suc=parseWS(t_wkspace_filename);
	assert(suc); //make sure not NULL
	//dof
	m_dof=getEnvironment()->getFlockStates().size()*4; //p_x, p_y, v_x, v_y
	//----------------------------------------------------------

    if(getEnvironment()==NULL)
    {
        cerr<<"! Error: No Workspace defined"<<endl;
        return false;
    }

    if( m_tree==NULL && m_graph==NULL)
    {
        cerr<<"! Error: No Tree or Graph Planner defined"<<endl;
        return false;
    }

    if( m_lps.empty() )
    {
        cerr<<"! Error: No Local Planner defined"<<endl;
        return false;
    }
    if( m_dm==NULL )
    {
        cerr<<"! Error: No Distance Metric defined"<<endl;
        return false;
    }
    delete [] tmp;

    //classify shepherds/flock
    FSLIST& states=getEnvironment()->getFlockStates();
    for(FSLIST::iterator i=states.begin();i!=states.end();i++){
        CFlockState * a=*i;
		if(isClass(a,"CHerdingFlock"))
            m_shepherds.push_back(a);
        else
            m_flock.push_back(a);
    }//end

    if( m_shepherds.empty() )
    {
        cerr<<"! Error: No shepherds defined"<<endl;
        return false;
    }

    if( m_flock.empty() )
    {
        cerr<<"! Error: No flock defined"<<endl;
        return false;
    }

	//disable visual hint
	getSI()->disableVisualHint();

    //get goal from shepherd
    m_goal=((CHerdingFlockState*)m_shepherds.front())->goal;

	//move the shepherds around the flock
	snapShepherdsAroundFlock();

    //setup query
    Cfg cfg;
    fromFlockState(cfg);
    m_query.addQuery(cfg);
	m_map.addnode(cfg);
	m_start=cfg.m_flock_cen;

	//setup initial goal in the tree
	if(m_tree!=NULL){
		float view_range=m_flock.front()->getType()->getViewRadius();
		m_tree->setGoal(m_goal,view_range);
	}

    return true;
}


bool MP::findPath()
{
    //turn off some simulation features
	getSI()->disableMedialAxis();
	getSI()->disableReachGoal();
	//
    bool found=false;
    m_state.pushState("Find path");

    m_path.clear();

	if(m_tree!=NULL)
	    found=m_tree->findPath(m_path);
	else
		found=m_graph->findPath(m_path);

	if (found)
		printf("Path found in %d steps.\n", getLPs().front()->getTotalSimulateStepCount());
	else
		printf("Path NOT found after %d steps.\n", getLPs().front()->getTotalSimulateStepCount());

	//if(found)
	queryMap();

    //pop Find path
    m_state.popState();

	//
	getSI()->enableVisualHint();
	//
    return true;
    // return found;
}

bool MP::queryMap()
{
    m_path.clear();
    return m_query.findPath(m_path);
}


void MP::snapShepherdsAroundFlock()
{
	pair<float, Point2d> rc=findEC(m_flock);
    float view_range=m_flock.front()->getType()->getViewRadius();

	//get dir
	Vector2d dir;
    CSimpleHerdingBehaviorRule tmp;
    vector<Point2d> milestones=tmp.findMilestone
                                  (*(CMapFlockState*)m_shepherds.front(),rc.second,m_goal,
								  view_range);

	if(!milestones.empty()){
		dir=(milestones.front()-rc.second).normalize();
	}
	else{
		dir=Vector2d(m_rng->uniform(),m_rng->uniform()).normalize();
	}

	//setup shepherds' targets
    int size=m_shepherds.size();

    Vector2d f_v = -dir*rc.first;

	if(size==1){
		m_shepherds.front()->setPos(rc.second+f_v);
	}
	else{
		float angle=0.1*PI;
		float theta = ((PI-2*angle)/(size-1));
		int j=0;
		for(FSLIST::iterator i=m_shepherds.begin();i!=m_shepherds.end();i++,j++){
			float phi = theta*j + angle;
			float cos_phi=cos(phi);
			float sin_phi=sin(phi);
			float x2 = cos_phi*f_v[0] - sin_phi*f_v[1] + rc.second[0];
			float y2 = sin_phi*f_v[0] + cos_phi*f_v[1] + rc.second[1];
			(*i)->setPos(Point2d(x2,y2));
		}//end for
	}
}

void MP::printStatistics()
{   
	//initialize
	static int count=0;
	static float minDistSqr = numeric_limits<float>::max();
    Cfg& q = getRM()->getVector().back();

	CMapFlockState * shepherd=(CMapFlockState *)(m_shepherds.front());
	list<Point2d> path;
	PRMS * prm=((CHerdingFlock*)shepherd->getType())->prm;
	prm->findPath(*shepherd->getMap(),q.m_flock_cen,m_goal,path); 
	float dist=::pathLength(path);

    //check if we are making progress
	if (dist < minDistSqr)
	{
		minDistSqr = dist;
		printf("\nExpansion: %d  DistanceToGoal: %f\n", count, minDistSqr);
	}

	count++;
}

