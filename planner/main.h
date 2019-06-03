#include "mp.h"

string strFilename;
bool b_disableGL=false;

ShepherdingInstance * instance=NULL;
extern bool Displayed;
extern bool Simulating;//a flag that tells if it is currently simulating
extern bool isGlutInitialized;
extern CParticleSolver gSolver;
extern int windowW,windowH;

void replay_keyboard( unsigned char key, int x, int y );
void startReplay();
void show3D();
void TimerCallback(int value);


//------------------------------------------------------------------------

class DrawShepherdTargets : public sh_Draw
{
public:

	vector<Point2d> tagets;

	void draw()
	{
#if GL_ON
        #ifdef NO_DRAW_GUIDES
        return;
        #endif
		if(tagets.empty()) return;

		glDisable(GL_LIGHTING);
		glColor3d(0,0.5,0);

		float radius=getMP()->getShepherds().front()->getType()->getGeometry().getRadius();
		int count=0;
		for(vector<Point2d>::iterator i=tagets.begin();i!=tagets.end();i++){
			const Point2d& pt=*i;

			glPushMatrix();
			glLineWidth(1);
			glTranslated(pt[0],0,pt[1]);
			glColor3f(0,1,0);
			drawCircle(radius, PI2, true); //filled circle
			glColor3f(0,0.5f,0);
			glTranslated(0,0.5,0);
			drawCircle(radius, PI2, false); //filled circle
			glPopMatrix();

		}//end for i
#endif
	}

};

DrawShepherdTargets drawTarget;

//------------------------------------------------------------------------

//
// draw tree-based roadmap
//

#include "draw/sh_draw_map.h"
class DrawTreeMap : public sh_Draw
{
public:

	int drawStopID;	// draw all the nodes up to this ID
	bool drawNodeIDs;
	int m_GID;

	DrawTreeMap(){
		drawStopID = -1;
		drawNodeIDs = true;
		m_GID=-1;
		//drawStopID = getMP()->getRM()->getVector().size();
	}

	void draw()
	{
		//if(m_GID==-1){
		//	m_GID= glGenLists (1);
		//    glNewList(m_GID,GL_COMPILE);
			buildGraph();
		//	glEndList();
		//}
		//glCallList(m_GID);
	}

//	void drawNodeIDs()
//	{
//	    glDisable(GL_LIGHTING);
//	    glColor3d(0.2,0.2,0.1);
//	    typedef list<CFlock*>::iterator FIT;
//	    char value[32];
//	    //for each type of obst
//	    for( FIT i=flock.begin();i!=flock.end();i++ ){
//	        CFlock* flock_type=*i;
//	        int size=flock_type->getStateSize();
//	        float height=1.2*flock_type->getGeometry().getHeight();
//	        //for each obst
//	        for( int i=0;i<size;i++ ){
//	            CFlockState & state = flock_type->getState(i);
//	            const Point2d& pos=state.getPos();
//	            sprintf(value,"%d",state.getID());
//	            drawstr(pos[0],height,pos[1],value);
//	        }
//	    }//end OIT
//	}

	void buildGraph(){
#if GL_ON
		glDisable(GL_LIGHTING);
   		// init
    	Roadmap * rm = getMP()->getRM();
    	MPGraph& graph = rm->getGraph();

		//get all nodes in the roadmap
    	vector<VID> all_nodes;
    	graph.GetVerticesVID(all_nodes);

		//node radius
    	float radius=getMP()->getFlock().front()->getType()->getGeometry().getRadius();
	    ///////////////////////////////////////////////////////////////////////////
    	//draw nodes
	    char value[32];
		int nodesize=all_nodes.size();

		glColor3f(0.5f,0.05f,0.5f);
		glBegin(GL_POINTS);
		for(int i=0;i<nodesize;i++){
			if ((i >= drawStopID) && (drawStopID >= 0))
				continue;
			Cfg& c=rm->getData(graph.GetData(all_nodes[i]));
	        Point2d& pos=c.m_flock_cen; //flock center
			glVertex3f(pos[0],-0.51,pos[1]);

    	    //glPushMatrix();
        	//glTranslatef(pos[0],-0.51,pos[1]);
			//drawCircle(0.25,PI2,true);
    	    //glPopMatrix();
/*
			//draw target
			Point2d& pos2=c.m_flock_tar;
			glColor3f(0.9f,0.1f,0.5f);
			glBegin(GL_POINTS);
			glVertex3f(pos2[0],1,pos2[1]);
			glEnd();


			//add a line in between
			glColor3f(0.2f,0.9f,0.0f);
			glBegin(GL_LINES);
			glVertex3f(pos2[0],1,pos2[1]);
			glVertex3f(pos[0],1,pos[1]);
			glEnd();
*/
	    }
		glEnd();

		// draw node IDs
		if (drawNodeIDs)
			for(int i=0; i < nodesize; i++){
				if ((i >= drawStopID) && (drawStopID >= 0))
					break;
				Cfg& c=rm->getData(graph.GetData(all_nodes[i]));
				Point2d& pos=c.m_flock_cen; //flock center

				sprintf(value,"%d", i);
				drawstr(pos[0], 1.0, pos[1], value);
			}

	    ///////////////////////////////////////////////////////////////////////////
    	//draw edges
    	vector< pair<VID,VID> > edges;
	    typedef vector< pair<VID,VID> >::iterator EIT;
	    graph.GetEdges(edges);
    	glBegin(GL_LINES);
		glColor3f(0,0,0);
		//printf("\n\nEdges:\n");
	    for(EIT i=edges.begin();i!=edges.end();i++){
	    	if ((i->second >= drawStopID) && (drawStopID >= 0))
	    		continue;
	    	//printf("\t%d %d\n", i->first, i->second);
			Cfg& c1=rm->getData(graph.GetData(all_nodes[i->first]));
			Cfg& c2=rm->getData(graph.GetData(all_nodes[i->second]));
	        glVertex3f(c1.m_flock_cen[0],-0.55f,c1.m_flock_cen[1]);
			glVertex3f(c2.m_flock_cen[0],-0.55f,c2.m_flock_cen[1]);
	    }//end for
    	glEnd();

		glEnable(GL_LIGHTING);
#endif 
	}
};


DrawTreeMap drawTreeMap;

//------------------------------------------------------------------------

//
// draw graph-based roadmap
//
class DrawGraphMap : public sh_Draw
{
public:
	
	bool m_disable;
	int m_GID;

	DrawGraphMap(){
		m_disable=true;
		m_GID=-1;
	}

	void draw()
	{
#if GL_ON
		if(m_disable) return;
		if(m_GID==-1){
			m_GID=glGenLists(1);
		    glNewList(m_GID,GL_COMPILE);
			buildGraph();
			glEndList();
		}
		glCallList(m_GID);
#endif
	}

#if GL_ON
	void buildGraph(){

		glDisable(GL_LIGHTING);
   		// init
		MetaRoadmap& rm=((Graph_meta_fuzzy*)getMP()->getGraph())->getRoadmap();
    	MetaGraph& graph = rm.getGraph();

		//get all nodes in the roadmap
    	vector<VID> all_nodes;
    	graph.GetVerticesVID(all_nodes);

		//node radius
    	//float radius=getMP()->getFlock().front()->getType()->getGeometry().getRadius();
	    ///////////////////////////////////////////////////////////////////////////
    	//draw nodes
		int nodesize=all_nodes.size();
		for(int i=0;i<nodesize;i++){
			//get cfg
			MetaCfg& c=rm.getData(all_nodes[i]);
	        Point2d& pos=c.m_flock_cen; //flock center
			glPushMatrix();
			glTranslatef(pos[0],-0.9,pos[1]);
			glColor3f(0.6f,0.55f,0.3f);
			drawCircle(c.m_flock_rad, PI2, true);
			glColor3f(0,0,0);
			glTranslatef(0,0.1,0);
			glLineWidth(2);
			drawCircle(0.5f, PI2, false); //filled circle
			glTranslatef(0,0.1,0); //draw line
			glLineWidth(3);
			glBegin(GL_LINES);
			glColor3f(1,0,0);
			glVertex3f(0,0,0);
			glVertex3f(c.m_flock_dir[0],0.5,c.m_flock_dir[1]);
			glEnd();
			glPopMatrix();
	    }

	    ///////////////////////////////////////////////////////////////////////////
    	//draw edges
		glLineWidth(1);
    	vector< pair<VID,VID> > edges;
	    typedef vector< pair<VID,VID> >::iterator EIT;
	    graph.GetEdges(edges);
    	glColor3f(0,0,0);
	    {for(EIT i=edges.begin();i!=edges.end();i++){
			MetaCfg& c1=rm.getData(i->first);
			MetaCfg& c2=rm.getData(i->second);

			Point2d s=c1.m_flock_cen;
			Point2d g=c2.m_flock_cen;
			Vector2d v=(g-s).normalize();
			s=s+v*0.5;
			g=g-v*0.5;
			glBegin(GL_LINES);
	        glVertex3f(s[0],-0.55f,s[1]);
			glVertex3f(g[0],-0.55f,g[1]);
			glEnd();
			Vector2d v1(-v[1],v[0]);
			Vector2d v2(v[1],-v[0]);
			Point2d g1=g+(v1-v-v-v).normalize()*0.95;
			Point2d g2=g+(v2-v-v-v).normalize()*0.95;
			glBegin(GL_TRIANGLES);
			glVertex3f(g[0],-0.54f,g[1]);
			glVertex3f(g2[0],-0.54f,g2[1]);
			glVertex3f(g1[0],-0.54f,g1[1]);
			glEnd();
	    }}//end for

		glEnable(GL_LIGHTING);
		glLineWidth(1);
	}
#endif
};


DrawGraphMap drawGraphMap;

