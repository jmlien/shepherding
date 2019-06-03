#include "shepherding_base.h"
#include "shepherding_behaviors.h"

//-----------------------------------------------------------------------------

void flock_raw_data::setupBasicFlock(CFlock * boid)
{
    assert(boid);
    boid->setColor(color[0], color[1], color[2]);
    boid->setViewRadius( view_range );
    boid->setViewAngle( view_angle );
}

void flock_raw_data::setupBasicForceRule(CBasicForceRule * frule)
{
    assert(frule);
    frule->setSeparation( separation );
    frule->setCohesion( cohesion );
    frule->setAlignment( alignment );
    frule->setObstRepulsion( obst_repulsion );
    frule->setDampingFactor( damping );
    frule->setMaxForce( max_force );
}

//-----------------------------------------------------------------------------

bool WSParser::parse(string filename)
{
    ifstream fin(filename.c_str());
    if(!fin.good()){
        cerr<<"! Error: Cannot open file:"<<filename<<endl;
        return false;
    }//end good

    bool r=parse(fin);
    fin.close();
    return r;
}

//parsing the ws file
bool  WSParser::parse(istream& in)
{
    const int size=1024;
    char * tmp=new char[size];

    while(!in.eof())
	{
        in.getline(tmp,size);
        list<string> tok=tokenize(tmp," \t[]()<>,={");
        if(tok.empty()) continue;

        string label=tolower(tok.front());
        tok.pop_front();
        if(label[0]=='#') continue; //comment

        //create cfg generator
        if(label=="obstacle")
        {
            list< list<string> > tokens;
            obst_raw_data data;
            getAllLabels(in,tokens);
            readObstacles(tokens,data);
            createObstacles(data);
        }
        else if(label=="flock")
        {
            list< list<string> > tokens;
            flock_raw_data data;
            getAllLabels(in,tokens);
            readFlock(tokens,data);
            createFlock(tokens,data);
        }
        else if(label=="shepherd"){
            list< list<string> > tokens;
            flock_raw_data data;
            getAllLabels(in,tokens);
            readFlock(tokens,data);
            createShepherds(tokens,data);
        }
        else if(label=="bbox")
        {
            //expect 6 values
            if(tok.size()!=6){ cerr<<"! Error: expect 6 values for bounding box, found "<<tok.size()<<endl; }
            float bbox[6]; int index=0;
            for(list<string>::iterator i=tok.begin();i!=tok.end();i++)
                bbox[index++]=atof(i->c_str());
            getEnvironment()->setBBX(bbox);
        }
        else{
            cerr<<"! WARNING: Unknown label: "<<label<<endl;
        }
    }//end while

    return true;
}

bool WSParser::readObstacles(list< list<string> >& tokens,obst_raw_data & data )
{
    for(list< list<string> >::iterator i=tokens.begin();i!=tokens.end();i++){
        //
		string label = tolower(i->front());
        i->pop_front();
        //
        if(label=="size"){
            if(i->size()!=1){ cerr<<"! ERROR: obstacle.size has no value"<<endl; return false; }
            data.size=atoi(i->front().c_str());
        }
        else if(label=="geo"){
            if(i->empty()){ cerr<<"! ERROR: obstacle.geo has no value"<<endl; return false; }
            data.geo_name=i->front();
        }
        else if(label=="color"){
            if(i->size()!=3){ cerr<<"! ERROR: obstacle.color is incorrect"<<endl; return false; }
            Point3d color; int index=0;
            for(list<string>::iterator j=i->begin();j!=i->end();j++)
                color[index++]=atof(j->c_str());
            data.colors.push_back(color);
        }
        else if(label=="position"){
            if(i->size()!=2){ cerr<<"! ERROR: obstacle.position is incorrect"<<endl; return false; }
            Point2d pos; int index=0;
            for(list<string>::iterator j=i->begin();j!=i->end();j++)
                pos[index++]=atof(j->c_str());
            data.positions.push_back(pos);
        }
        else if(label=="angle"){
            if(i->size()!=1){ cerr<<"! ERROR: obstacle.angle is incorrect"<<endl; return false; }
            float angle=atof(i->front().c_str());
            data.angles.push_back(angle);
        }
        else{
            cerr<<"! WARNING: obstacle: unknow label: "<<label<<endl;
        }
    }//end for

    return true;
}

bool WSParser::createObstacles(obst_raw_data & data )
{
    if(data.geo_name.empty()){ cerr<<"! ERROR: obstacle.geo is not specified"<<endl; return false; }
    CObs * obst = new CObs(data.geo_name, data.size);

    for(unsigned int i=0;i<(unsigned int)data.size;i++){ //setup obstacles
        CObsState & obState = obst->getState(i);
        if(i<data.positions.size())
            obState.setPos(data.positions[i]);
        if(i<data.colors.size())
            obState.setColor(data.colors[i][0],data.colors[i][1],data.colors[i][2]);
        if(i<data.angles.size())
            obState.setRot(data.angles[i]);
    }

    getEnvironment()->addObstacle(obst);
	return true;
}

//read basic flock stuff
bool WSParser::readFlock(list< list<string> >& tokens, flock_raw_data& data)
{
    list< list<string> > unknown_tokens;

    for(list< list<string> >::iterator i=tokens.begin();i!=tokens.end();i++){
        //
        string label=tolower(i->front());
        i->pop_front();
        //
        if(label=="type"){
            if(i->size()!=1){ cerr<<"! ERROR: flock.type has no value"<<endl; return false; }
			data.type = tolower(i->front());
        }
        else if(label=="size"){
            if(i->size()!=1){ cerr<<"! ERROR: flock.size has no value"<<endl; return false; }
            data.size=atoi(i->front().c_str());
        }
        else if(label=="geo"){
            if(i->empty()){ cerr<<"! ERROR: flock.geo has no value"<<endl; return false; }
            data.geo=i->front();
        }
        else if(label=="color"){
            if(i->size()!=3){ cerr<<"! ERROR: flock.color is incorrect"<<endl; return false; }
            int index=0;
            for(list<string>::iterator j=i->begin();j!=i->end();j++)
                data.color[index++]=atof(j->c_str());
        }
        else if(label=="dist_center"){
            if(i->size()!=2){ cerr<<"! ERROR: flock.position is incorrect"<<endl; return false; }
            int index=0;
            for(list<string>::iterator j=i->begin();j!=i->end();j++)
                data.position[index++]=atof(j->c_str());
        }
        else if(label=="mass"){
            if(i->size()!=1){ cerr<<"! ERROR: flock.mass is incorrect"<<endl; return false; }
            data.mass=atof(i->front().c_str());
        }
        else if(label=="view_radius"){
            if(i->size()!=1){ cerr<<"! ERROR: flock.view_radius has no value"<<endl; return false; }
            data.view_range=atof(i->front().c_str());
        }
        else if(label=="view_angle"){
            if(i->empty()){ cerr<<"! ERROR: flock.view_angle has no value"<<endl; return false; }
            data.view_angle=atof(i->front().c_str());
        }
        else if(label=="dist_dev"){
            if(i->empty()){ cerr<<"! ERROR: flock.dist_dev has no value"<<endl; return false; }
            data.scatteredness=atof(i->front().c_str());
        }
        else if(label=="separation"){
            if(i->empty()){ cerr<<"! ERROR: flock.separation has no value"<<endl; return false; }
            data.separation=atof(i->front().c_str());
        }
        else if(label=="cohesion"){
            if(i->size()!=1){ cerr<<"! ERROR: flock.cohesion has no value"<<endl; return false; }
            data.cohesion=atof(i->front().c_str());
        }
        else if(label=="alignment"){
            if(i->empty()){ cerr<<"! ERROR: flock.alignment has no value"<<endl; return false; }
            data.alignment=atof(i->front().c_str());
        }
        else if(label=="obst_repulsion"){
            if(i->empty()){ cerr<<"! ERROR: flock.obst_repulsion has no value"<<endl; return false; }
            data.obst_repulsion=atof(i->front().c_str());
        }
        else if(label=="damping"){
            if(i->empty()){ cerr<<"! ERROR: flock.damping has no value"<<endl; return false; }
            data.damping=atof(i->front().c_str());
        }
        else if(label=="max_force"){
            if(i->empty()){ cerr<<"! ERROR: flock.max_force has no value"<<endl; return false; }
            data.max_force=atof(i->front().c_str());
        }
        else{
            i->push_front(label);
            unknown_tokens.push_back(*i);
        }//unknown token
    }//end for
    tokens.swap(unknown_tokens); //return unknown tokens
    return true;
}

//create flocks
bool WSParser::createFlock(list< list<string> >& tokens, flock_raw_data& data)
{
    CFlock * boid = new CFlock( getEnvironment(), data.geo, data.size );
    data.setupBasicFlock(boid);
    
	//create states
    for(int i=0; i<data.size; i++)
        boid->addState(new CFlockState(boid));

    boid->deployFlock(getRNG(), data.position, data.scatteredness );
    
	//force rule
    CBasicForceRule * frule=NULL;
    if(data.type=="basic"){
        frule = new CBasicForceRule();
    }
    else if(data.type=="scared"){
        boid->setBehaviorRule(new ScaredBehaviorRule());
        frule=new ScaredForceRule();
        //get addition parameters
        for(list< list<string> >::iterator i=tokens.begin();i!=tokens.end();i++){
            //
			string label = tolower(i->front());
            i->pop_front();
            //
            if(label=="afraid"){
                if(i->empty()){ cerr<<"! ERROR: flock.afraid has no value"<<endl; return false; }
                ((ScaredForceRule*)frule)->setAfraid(atof(i->front().c_str()));
            }
            //FOR EXPERIMENT ONLY. PLEASE REMOVE AFTER 2010/1/31
            else if(label=="random_force"){
                ((ScaredForceRule*)frule)->random_force=atof(i->front().c_str());
            }
            
        }//end for
    }
    else if(data.type == "movable_obstacle") {
        frule = new MovableObstacleForceRule();
        boid->getGeometry().buildCompleteCDModel(*boid->getRawModel());
    }
    else{
        cerr<<"! ERROR: Unknow flock type: "<<data.type<<endl;
        return false;
    }
    data.setupBasicForceRule(frule);
    boid->setForceRule(frule);
    getEnvironment()->addFlock(boid);

	return true;
}

//create shepherds
bool WSParser::createShepherds(list< list<string> >& tokens, flock_raw_data& data)
{
    CHerdingFlock *  i=NULL;
    CEnvironment * pEnv=getEnvironment();
    if(data.type=="simple"){
        i=new CSimpleHerdingFlock(pEnv,data.geo);
    }
    else if(data.type=="better"){
        i=new CBetterHerdingFlock(pEnv,data.geo);
    }
    else if(data.type=="multiple"){
        i=new CMultiHerdingFlock(pEnv,data.geo);
    }
    else if(data.type=="deform"){
        i=new CDeformHerdingFlock(pEnv,data.geo);
    }
    else if(data.type=="manual")
    {
		i = new CManualHerdingFlock(pEnv, data.geo);
    }
    else{//unknown type
        cerr<<"! Error: Unknown shepherd type="<<data.type<<endl;
        exit(1);
    }

    assert(i);

    bool r=i->initialize(tokens,data);
    
    getSI()->setShepherd(i);
    
    return r;
}


void WSParser::getAllLabels(istream& in, list< list<string> >& tokens)
{
    const int size=1024;
    char * tmp=new char[size];

    while(!in.eof()){
        in.getline(tmp,size);
        //check for termination
        if( string(tmp).find("}")!=string::npos ) //found "}"
            break;

        list<string> tok=tokenize(tmp," \t[]()<>,={");
        if(tok.empty()) continue;
        string label=tok.front();
        if(label[0]=='#') continue; //comment

        tokens.push_back(tok);
    }//end while

    delete [] tmp;
}


//convert a string to a list of tokens
list<string> WSParser::tokenize(char * tmp, const char * ignore)
{
    list<string> tokens;
    char * tok=strtok (tmp,ignore);
    while(tok!=NULL)
    {
        tokens.push_back(tok);
        tok=strtok(NULL,ignore);
    }
    return tokens;
}

