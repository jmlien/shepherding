#ifdef _WIN32
#pragma warning(disable : 4786)
#endif

#include "sh_Roadmap.h"
#include "float.h"
#include "shepherding_base.h"


/////////////////////////////////////////////////////////////////////
//
// CNodeData
//
/////////////////////////////////////////////////////////////////////

CNodeData::CNodeData(const Point2d& pos) { 
    m_ID=-1;
    m_Pos=pos;
    m_BRule=NULL;
}

CNodeData::~CNodeData()
{

}

void CNodeData::addSuccessor(CNodeData * s, float w)
{
    EdgeData e;
    e.m_Successor=s;
    e.m_Weight=w;
    m_EdgeData.push_back(e);
}

void CNodeData::changeWeight( CNodeData * s, float w )
{
    typedef list<EdgeData>::iterator EIT;
    //find successor with given id and change its weight
    if( w>0 ){      
        EdgeData e;
        e.m_Successor=s;
        EIT i=find(m_EdgeData.begin(),m_EdgeData.end(),e);
        if( i==m_EdgeData.end() ) return; //not found
        i->m_Weight+=w;
    }
    else{ //w<0
        w=-w;
        for( EIT i=m_EdgeData.begin();i!=m_EdgeData.end();i++ ){
            if( i->m_Successor!=s ){
                i->m_Weight+=w;
            }
        }
    }//end if w<0
}

CNodeData * CNodeData::getRandSuccessor(RNG * rng, CNodeData * s, int determin)
{
    //only one successor, need to go back
    if( m_EdgeData.size()==1 ) return m_EdgeData.front().m_Successor;
    typedef list<EdgeData>::iterator EIT;
    typedef list<EdgeData>::reverse_iterator REIT;

    if( determin<=10 ){

        float id_w=0;
        if(s!=NULL){ //find the weight of s....
            //all edge weights are less than weight of edge with given id
            {for( EIT i=m_EdgeData.begin();i!=m_EdgeData.end();i++ ){
                if( i->m_Successor==s ){id_w=i->m_Weight; break;}
            }}//end find id's weight
        }

        //find if id_w is max & compute total weight
        bool all_less=true; 
        float total_w=0;
        {for( EIT i=m_EdgeData.begin();i!=m_EdgeData.end();i++ ){
            if( i->m_Successor==s ) continue;
            if( i->m_Weight>=id_w ){ all_less=false; }
            total_w+=(float)pow(i->m_Weight,determin);
        }}
        if( all_less ) return s; //return id, when id's weight is largest

        //find successor with given id and change its weight
        float die=rng->uniform();
		{for( EIT i=m_EdgeData.begin();i!=m_EdgeData.end();i++ ){
            if( i->m_Successor==s ) continue; //ignore this one
            float w=(float)pow(i->m_Weight,determin)/total_w;
            die=die-w;
            if( die<0 ) return i->m_Successor;
        }}

        //make sure id is not returned
        {for(REIT i=m_EdgeData.rbegin();i!=m_EdgeData.rend();i++){
            if( i->m_Successor!=s ) return i->m_Successor;
        }}
        return NULL;
    }
    else{ //just get the maximum one
        CNodeData * maxN=NULL;
        float maxW=0;
        for( EIT i=m_EdgeData.begin();i!=m_EdgeData.end();i++ ){
            if( i->m_Successor==s ) continue; //ignore this one
            if( i->m_Weight>maxW ){ 
                maxW=i->m_Weight; 
                maxN=i->m_Successor;
            }
        }//end for
        return maxN;
    }//end if
}

/////////////////////////////////////////////////////////////////////
//
// Roadmap2D
//
/////////////////////////////////////////////////////////////////////

CRoadMap::CRoadMap(CFlock * f,int size)
{
    m_CNodeData.reserve(size);
    m_F=f;
}

CRoadMap::~CRoadMap()
{
    typedef vector<CNodeData*>::iterator NIT;
    for(NIT i=m_CNodeData.begin();i!=m_CNodeData.end();i++ )
        delete *i;
    m_F=NULL;
}

int CRoadMap::addNode(CNode& node)
{
    CNodeData * data=new CNodeData(node.getPos());
	return addNodeData(node,data);
}

int CRoadMap::addNodeData(CNode& node, CNodeData * data)
{
    if( data==NULL ) return -1;
    data->m_ID=(int)m_CNodeData.size();
    m_CNodeData.push_back(data);
    node.setID(data->getID());
    int id=m_Graph.AddVertex(node);
    if( id!=node.getID() ){
        cerr<<"! Error : CRoadMap::addNode Error"<<endl;
        return -1;
    }
    return data->getID();
}

void CRoadMap::delNode(int id)
{
    if( (int)m_CNodeData.size()<=id ) return;
    m_Graph.DeleteVertex(id);
    delete m_CNodeData[id];
    m_CNodeData[id]=NULL;
}


void CRoadMap::addEdge( CNodeData* n1, CNodeData* n2, float weight )
{
    n1->addSuccessor(n2,1);
    n2->addSuccessor(n1,1);
    int v1=n1->getID();
    int v2=n2->getID();
    m_Graph.AddEdge(v1,v2,weight);
    m_Graph.AddEdge(v2,v1,weight);
}

void CRoadMap::addEdge( int v1, int v2, float weight )
{
    int size=m_CNodeData.size();
    if( v1<0 || v1>=size || v2<0 || v2>=size ){
        cerr<<"! Error : addEdge : index out of range"<<endl;
        return;
    }
    CNodeData* n1=m_CNodeData[v1];
    CNodeData* n2=m_CNodeData[v2];
    addEdge(n1,n2,weight);
}


void CRoadMap::addEdge( int v1, int v2 )
{
    int size=m_CNodeData.size();
    if( v1<0 || v1>=size || v2<0 || v2>=size ){
        cerr<<"! Error : addEdge : index out of range"<<endl;
        return;
    }
    CNodeData* n1=m_CNodeData[v1];
    CNodeData* n2=m_CNodeData[v2];
    float dist=(float)(n1->getPos()-n2->getPos()).norm();
    addEdge(n1,n2,dist);
}

VID CRoadMap::closestNode(const Point2d& pt)
{
	int size=m_CNodeData.size();
	float min=FLT_MAX;
	VID id=size;

	for(int i=0;i<size;i++){
		if(m_CNodeData[i]==NULL) continue;
		float d=(m_CNodeData[i]->getPos()-pt).normsqr();
		if(d<min){
			min=d;
			id=i;
		}
	}

	//done
	return id;
}

bool CRoadMap::read(string& filename) //read from
{
    ifstream fin(filename.c_str());
    if(fin.good()==false) return false;

    //read node info

    int nsize=0;
    //fin.read((char*)&nsize,sizeof(int));
	fin >> nsize;
    for(int i=0;i<nsize;i++)
    {
        float v[3];
        //fin.read((char*)v,sizeof(float)*3);
		fin >> v[0] >> v[1] >> v[2];

        CNode node;
        Point2d pos(v[0],v[1]);
        node.setPos(pos);
        node.setClearance(v[2]);
        int id=addNode(node);

        //if(v[0]==FLT_MAX && v[1]==FLT_MAX &&  v[2]==FLT_MAX ) //this was deleted in original node...
        //delNode(id);
    }

    //read edge info
    int esize=0;
    //fin.read((char*)&esize,sizeof(int));
	fin >> esize;

	//cout << "read edge size=" << esize << endl;

    for(int i=0;i<esize;i++)
    {
        int id[2];
        float w;
        //fin.read((char*)id,sizeof(int)*2);
        //fin.read((char*)&w,sizeof(float));
		fin >> id[0] >> id[1] >> w;
        addEdge(id[0],id[1],w);
    }//end for

    fin.close();

	//cout << "done reading from file" << endl;

    return true;
}

bool CRoadMap::save(string& filename) //save to
{
	ofstream fout(filename.c_str());
	if(fout.good()==false) return false;
	
	int nsize = m_CNodeData.size(); //m_Graph.GetVerticesData(nodes);

	//remap node id
	vector<int> new_node_id(nsize,-1);
	int new_id = 0;
	for (int i = 0; i < nsize; i++)
	{
		CNodeData* data = m_CNodeData[i];
		if (data != NULL) new_node_id[i] = new_id++;
	}

	//write node info
	//vector<CNode> nodes;
    
    //fout.write((char*)&nsize,sizeof(int));
	fout << new_id << "\n";
    for(int i=0;i<nsize;i++){
        CNodeData* data=m_CNodeData[i];
        if(data!=NULL)
        {
            assert(data->m_ID==i);
            CNode node=m_Graph.GetData(data->m_ID);
            float v[3]={node.getPos()[0], node.getPos()[1], node.getClearance()};
            //fout.write((char*)v,sizeof(float)*3);
			fout << v[0] << " " << v[1] << " " << v[1] << "\n";
        }
    }

    //write edge info
    vector< pair< pair<VID,VID>, CEdge> > weights;
    typedef vector< pair< pair<VID,VID>, CEdge> >::iterator WIT;
    m_Graph.GetEdges(weights);
    int esize=weights.size();

	//cout << "write edge size=" << esize << " m_Graph edge count=" << m_Graph.GetEdgeCount()<< endl;
    //fout.write((char*)&esize,sizeof(int));
	fout << esize << "\n";
    for( WIT iw=weights.begin();iw!=weights.end();iw++ ){
		int id[2] = { new_node_id[iw->first.first], new_node_id[iw->first.second] };
		assert(id[0] != -1 && id[1]!=-1);
        float w=iw->second.Weight();
        //fout.write((char*)id,sizeof(int)*2);
        //fout.write((char*)&w,sizeof(float));
		fout << id[0] << " " << id[1] << " " << w << "\n";
    }//end for

    fout.close();

	return true;
}

