#ifndef _DEFORM_HERDING_GRID_H_
#define _DEFORM_HERDING_GRID_H_

#include "shepherding_base.h"

class dhGrid : public sh_Draw
{
public:

	dhGrid(){}
	
	//grid cell type
	struct cell{
		cell(){m_free=true; m_shepherd=m_sheep=m_target=false;m_dist2goal=INT_MAX;m_flag=0;}
		int m_dist2goal;
		bool m_free;
		bool m_target;     //is a target cell
		bool m_sheep;      //is sheep in this cell
		bool m_shepherd;   //is a shepherd cell
		
		//for path search
		unsigned int  m_flag;
		pair<int,int> m_parent;
	};

	bool initialize()
	{
		//get box
		const CBoundingBox& bbox=getEnvironment()->getBBX();
		
		//get view
		CFlockState * sheep=NULL;
		FSLIST& flock=getEnvironment()->getFlockStates();
		for(FSLIST::iterator i=flock.begin();i!=flock.end();i++){
			CFlockState * a=*i;
			string type=string(typeid(*a->getType()).name());
			if( type.find("CHerdingFlock")==string::npos ){ //not a shepherd
				sheep=a;
				break;
			}//end if
		}//end for
		
		if(sheep==NULL) return false; //failed...

		int w=bbox.getBBXValue()[1]-bbox.getBBXValue()[0];
		int h=bbox.getBBXValue()[5]-bbox.getBBXValue()[4];
		m_res=sheep->getType()->getGeometry().getRadius()*2;
		m_origin.set(bbox.getBBXValue()[0],bbox.getBBXValue()[4]);
		m_width=(int)ceil(w/m_res);
		m_height=(int)ceil(h/m_res);

		//allocate
		m_grid=new cell[m_width*m_height];
		assert(m_grid);

		//evaluate each cell...
		CRobot2D& robot=sheep->getType()->getGeometry();
		for(int i=0;i<m_width;i++){
			for(int j=0;j<m_height;j++){
				Point2d pos(m_origin[0]+m_res*(i+0.5),m_origin[1]+m_res*(j+0.5));

				if(isCollision(*getEnvironment(),robot,pos))
					m_grid[i*m_height+j].m_free=false;
			}
		}//end for
	}
	
	pair<int,int> getID(const Point2d& pos) const
	{
		Vector2d diff=(pos-m_origin)/m_res;
		int i=(int)floor(diff[0]);
		int j=(int)floor(diff[1]);
		return pair<int,int>(i,j);
	}
	
	Point2d getPos(const pair<int,int>& id) const
	{
		return m_origin+Vector2d(m_res*(id.first+0.5f),m_res*(id.second+0.5f));
	}
	
	//get cell by position
	cell& get(const pair<int,int>& id)
	{
		return m_grid[id.first*m_height+id.second];;
	}
	
	//get cell by position
	cell& get(const Point2d& pos)
	{
		Vector2d diff=(pos-m_origin)/m_res;
		int i=(int)floor(diff[0]);
		int j=(int)floor(diff[1]);
		return m_grid[i*m_height+j];
	}
	
	int width() const { return m_width; }
	int height() const { return m_height; }
	
	void clearDist2Goal()
	{
		int size=m_width*m_height;	
		for(int i=0;i<size;i++) m_grid[i].m_dist2goal=INT_MAX;
	}
	

	void computeDist2Goal(const Point2d& goal)
	{
		pair<int,int> id=getID(goal);
		cell& c=get(id);
		c.m_dist2goal=0;
		list< pair<int,int> > open;
		open.push_back(id);
		
		while(!open.empty()){
			pair<int,int> id=open.front();
			open.pop_front();
			int dist=get(id).m_dist2goal;
			//propogate
			for(int i=-1;i<=1;i++){
			    int x=id.first+i;
			    if(x<0||x>=m_width) continue;
				for(int j=-1;j<=1;j++){
					int y=id.second+j;
				    if(y<0||y>=m_height) continue;					
				    pair<int,int> nid(x,y);
				    cell& c=get(nid);
				    if(!c.m_free) continue;
				    if(c.m_dist2goal!=INT_MAX) continue; //visited
				    c.m_dist2goal=dist+1;
				    open.push_back(nid);
				}//end j
			}//end i
		}//end while
	}
	
	unsigned int getNextCellFlag(){
		static unsigned int FLAG=0;
		FLAG++;
		
		if(FLAG==0){ //this happens rarely
			// clear up all flags and reset them to 0
			int size=m_width*m_height;
			for(int i=0;i<size;i++) m_grid[i].m_flag=0;
			FLAG++; //i.e., FLAG=1;
		}
		return FLAG;
	}
	
	//this is best first search
	bool findPath
	(const Point2d& start, const Point2d& goal, list< pair<int,int> >& path, bool for_shepherd=true)
	{
		typedef pair<int,int> ID;
		ID sid=getID(start);
		ID gid=getID(goal);
		const unsigned int FLAG=getNextCellFlag(); //new unused flag
		
		cell& c=get(sid);
		c.m_flag=FLAG;
		c.m_parent=sid; //itself
		vector< pair<float,ID> > open;
		open.push_back(pair<float,ID>(-(start-goal).normsqr(),sid));
		push_heap(open.begin(), open.end()) ;
		
		bool path_found=false;
		
		while(!open.empty()){
			pair<float,ID> did=open.front();
			pop_heap(open.begin(), open.end()) ;
			open.pop_back();
			ID id=did.second;

			//propogate
			for(int i=-1;i<=1;i++){
			    int x=id.first+i;
			    if(x<0||x>=m_width) continue;
				for(int j=-1;j<=1;j++){
					int y=id.second+j;
				    if(y<0||y>=m_height) continue;					
				    pair<int,int> nid(x,y);
				    cell& c=get(nid);
				    if(!c.m_free) continue;
				    //
				    if(for_shepherd){
					    if(c.m_target) continue; //dont want to go through target
					    if(c.m_sheep) continue; //dont want to go through sheep
					}
					//
				    if(c.m_flag==FLAG) continue; //visited
				    c.m_flag=FLAG;
				    c.m_parent=id;
				    if(nid==gid) {
				    	path_found=true;
				    	break;
				    }
				    float d=-(getPos(nid)-goal).normsqr();
				    open.push_back(pair<float,ID>(d,nid));
				}//end j
			}//end i
			
			//
			if(path_found) break;
		}//end while
		
		//trace back
		if(path_found){
			ID tmp=gid;
			while(true){
				cell& c=get(tmp);
				path.push_front(tmp);
				if(tmp==c.m_parent) break; //back to start
				tmp=c.m_parent;
			}
		}
		
		//done
		return path_found;
	}
	
	//draw
	void draw()
	{
#if GL_ON
		static int GID=-1;
		if(GID<0){
			float H=0.5f;
			GID=glGenLists(1);
			glNewList(GID,GL_COMPILE);
			//build the index
			glDisable(GL_LIGHTING);
			for(int i=0;i<m_width;i++){
				for(int j=0;j<m_height;j++){
					int index=i*m_height+j;
					if(m_grid[index].m_free) continue;
					glColor4f(0.3f,0.3f,0.3f,0.5f);
					drawCell(i,j,H);
				}//end for
			}//end for
			glEndList();
		}
		glCallList(GID);
#endif 
	}
	
	void drawCell(int i, int j, int H)
	{
#if GL_ON
		int index=i*m_height+j;
		
		//fill cell	
		glBegin(GL_QUADS);
		glVertex3f(m_origin[0]+i*m_res,     H, m_origin[1]+j*m_res);
		glVertex3f(m_origin[0]+(i+1)*m_res, H, m_origin[1]+j*m_res);
		glVertex3f(m_origin[0]+(i+1)*m_res, H, m_origin[1]+(j+1)*m_res);
		glVertex3f(m_origin[0]+i*m_res,     H, m_origin[1]+(j+1)*m_res);
		glEnd();
		
		//draw boundary
		glBegin(GL_LINE_LOOP);
		glColor3f(0.5f,0.5f,0.5f);
		glVertex3f(m_origin[0]+i*m_res,     H+0.01f, m_origin[1]+j*m_res);
		glVertex3f(m_origin[0]+(i+1)*m_res, H+0.01f, m_origin[1]+j*m_res);
		glVertex3f(m_origin[0]+(i+1)*m_res, H+0.01f, m_origin[1]+(j+1)*m_res);
		glVertex3f(m_origin[0]+i*m_res,     H+0.01f, m_origin[1]+(j+1)*m_res);
		glEnd();
#endif
	}
	
private:

	cell * m_grid;
	int m_width, m_height;
	Point2d m_origin;
	float m_res; //resolution

};

//draw a subset of grids
class drawCells : public sh_Draw
{
protected:
	
	typedef pair<int,int> CellID;
	typedef list<CellID>  CellIDList;
	CellIDList& m_cells;  //shepherd cells
	Point3d m_color;	
	dhGrid * m_grid;
public:
	drawCells(dhGrid * grid, CellIDList& cells, const Point3d& color): m_cells(cells){
		m_grid=grid;
		m_color=color;
	}

	//draw
	void draw()
	{
#if GL_ON
		glDisable(GL_LIGHTING);
		float H=0.51f;
		for(CellIDList::iterator i=m_cells.begin();i!=m_cells.end();i++){
			glColor4f(m_color[0],m_color[1],m_color[2],0.5f);
			m_grid->drawCell(i->first,i->second,H);
		}//end for
#endif 
	}
};

#endif 


