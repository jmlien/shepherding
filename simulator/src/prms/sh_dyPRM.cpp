#ifdef _WIN32
#pragma warning(disable : 4786)
#endif

#include "sh_dyPRM.h"
#include "sh_Environment.h"
#include "sh_FlockState.h"
#include "sh_CollisionDetection.h"
#include <assert.h>

#include "graph/GraphAlgo.h"

//////////////////////////////////////////////////////////////////////
#define SCALE_R 1.5f
#define K_C 10

//////////////////////////////////////////////////////////////////////
// Constructor & Destructor

CDynPRM::CDynPRM(CEnvironment * env, RNG * rng): m_CD(env), m_PRMs(env, rng)
{
    m_pEnv=env;
    m_pRNG=rng;
    if( m_pEnv==NULL || m_pRNG==NULL ) throw "! Error : CDynPRM init error";
}

CDynPRM::~CDynPRM()
{
    m_pEnv=NULL;
    m_pRNG=NULL; 
}


bool CDynPRM::queryPath(const Point2d& s,const Point2d& g,
CRoadMap& d_map,CRoadMap& g_map, CFlockState* shepherd,
list<CFlockState*>& flock, list<Point2d>& path /*return*/)
{
    updateMap(d_map,shepherd,flock);
    expandMap(d_map,shepherd,flock);
    return findPath(s,g,d_map,g_map,shepherd,flock,path);
}

//expand the map
void CDynPRM::expandMap
(CRoadMap& dy_map,CFlockState* shepherd,list<CFlockState*>& flock)
{
    int nodesize=5;

    //compute flock center and radius
    typedef list<CFlockState*>::iterator FIT;
    for( FIT i=flock.begin();i!=flock.end();i++){
        const Point2d& pos=(*i)->getPos();
        float VR=(*i)->getType()->getViewRadius()*SCALE_R;
        int size=countNodes(dy_map,pos,VR);
        if( size>100 ) continue;
        //sample nodes inside the circle
        sampleInCircle(shepherd,flock,dy_map,pos,VR,nodesize);
    }

    //connect nodes
    connectNodes(dy_map);
    m_NewNodes.clear();
}

void CDynPRM::updateMap
(CRoadMap& dy_map, CFlockState* shepherd, list<CFlockState*>& flock)
{
    //CFlock * m_F=dy_map.getFlock();
    //CRobot2D& geo=m_F->getGeometry();
    float viewR=flock.back()->getType()->getViewRadius()*SCALE_R;
    WDG& g=dy_map.getGraph();
    ///////////////////////////////////////////////////////////////////////////
    // check map nodes
    vector<VID> vids;
    typedef vector<VID>::iterator VIT;
    g.GetVerticesVID(vids);

    for( VIT i=vids.begin();i!=vids.end();i++ ){
        CNodeData * node=dy_map.getNode(*i);
        float W=computeWeight(flock,node->getPos(),viewR);
        m_Weight[*i]=W;
        if( W==0 ) dy_map.delNode(*i);
    }
    ///////////////////////////////////////////////////////////////////////////
    // check map edges
    vector< pair< pair<VID,VID>, CEdge> > weights;
    typedef vector< pair< pair<VID,VID>, CEdge> >::iterator WIT;
    g.GetEdges(weights);

    for( WIT iw=weights.begin();iw!=weights.end();iw++ ){
        int id1=iw->first.first;
        //const Point2d& p1=dy_map.getNode(id1)->getPos();
        int id2=iw->first.second;
        //const Point2d& p2=dy_map.getNode(id2)->getPos();
        float w=(m_Weight[id1]+m_Weight[id2])/2;
        g.ChangeEdgeWeight(id1,id2,w);
    }//end for
}


///////////////////////////////////////////////////////////////////////////////
//
// Private Stuff
//
///////////////////////////////////////////////////////////////////////////////
bool CDynPRM::findPath(const Point2d& s,const Point2d& g,
CRoadMap& d_map,CRoadMap& g_map, CFlockState* shepherd,
list<CFlockState*>& flock, list<Point2d>& path /*return*/)
{
    //connect g to the d_map
    list<Point2d> path2Goal, path2Map;
    int gid,sid;
    if( (gid=connectToMap(g,d_map,g_map,path2Goal))==-1 )
        return false;
    path2Goal.reverse();
    if( (sid=connectToMap(s,d_map,g_map,path2Map,true,shepherd->getFacingDir()))==-1 )
        return false;
    if( findPathV1V2(d_map,sid,gid,path)==false )
        return false;
    path.insert(path.begin(),path2Map.begin(),path2Map.end());
    path.insert(path.end(),path2Goal.begin(),path2Goal.end());
    return true;
}

int CDynPRM::countNodes(CRoadMap& dy_map, const Point2d& O, float R)
{
    typedef vector<CNodeData*>::iterator NIT;
    vector<CNodeData*>& nodes=dy_map.getNodes();
    float RR=R*R;
    //for each node find k cloest
    int count=0;
    for( NIT i=nodes.begin();i!=nodes.end();i++){
        if( (*i)==NULL ) continue;
        if( ((*i)->getPos()-O).normsqr()<RR )
            count++;
    }
    return count;
}

void CDynPRM::sampleInCircle
(CFlockState* shepherd, list<CFlockState*>& flock,
 CRoadMap& dy_map, const Point2d& O, float R, int size)
{
    CRobot2D & shepherdGeo=shepherd->getType()->getGeometry();
    m_Weight.reserve(dy_map.getNodeSize()+size);
    float viewR=flock.back()->getType()->getViewRadius()*SCALE_R;

    for( int i=0;i<size;i++ ){
        //sample a point
        Point2d cfg=sampleInCircle(O,R);
        if( m_CD.isCollision(shepherdGeo,cfg) ){ i--; continue; }
        float W=computeWeight(flock,cfg,viewR);
        if( W==0 ) continue;
        CNode node; node.setPos(cfg);
        dy_map.addNode(node);
        m_Weight.push_back(W);
        m_NewNodes.push_back(dy_map.getNode(m_Weight.size()-1));
    }
}

Point2d CDynPRM::sampleInCircle(const Point2d& O, float R)
{
    float theta=m_pRNG->uniform()*2*PI;
    float rand=fabs(m_pRNG->gauss(0.2f));
    if( rand>1 ) rand=1;
    float r=(1-rand)*R;
    return O+Vector2d(r*cos(theta),r*sin(theta));
}

float CDynPRM::computeWeight
(list<CFlockState*>& flock, const Point2d& cfg, float R)
{
    float RR=R*R;
    //find the closest
    typedef list<CFlockState*>::iterator IIT;
    float min_dist=1e10;
    CFlockState* closeF=NULL;
    {
        for(IIT i=flock.begin();i!=flock.end();i++){
            const Point2d& pos=(*i)->getPos();
            float dist=(pos-cfg).normsqr();
            if( dist<min_dist ){
                min_dist=dist;
                closeF=*i;
            }
        }//end for
    }

    //check if cfg can connect to closeF
    if( min_dist>RR ) return 0; //too far
    //can't see??
    if( m_CD.isCollision(closeF->getType()->getGeometry(),cfg,closeF->getPos()) )
        return 0;
    float W=0;
    {
        for(IIT i=flock.begin();i!=flock.end();i++){
            const Point2d& pos=(*i)->getPos();
            float dist=(float)(pos-cfg).normsqr();
            if( dist<=RR ){
                /*
                if(isCollision
                   (*m_pEnv,(*i)->getType()->getGeometry(),cfg,pos))
                    continue;
                */
                W+=(RR-dist);
            }
        }//end for
    }
    return W;
}

void CDynPRM::connectNodes(CRoadMap& rmap)
{
    CFlock * m_F=rmap.getFlock();
    typedef list<CNodeData*>::iterator NIT;
    vector<CNodeData*>& nodes=rmap.getNodes();
    CRobot2D& geo=m_F->getGeometry();
    WDG& g=rmap.getGraph();

    //for each node find k cloest
    for( NIT i=m_NewNodes.begin();i!=m_NewNodes.end();i++){
        if( *i==NULL ) continue; //deleted
        int vid1=(*i)->getID();
        CNodeData * close[K_C];
        int k=FindKClosest(nodes[vid1],close,nodes);
        for( int j=0;j<k;j++ ){
            int vid2=close[j]->getID();
            if( g.IsEdge(vid1,vid2) ) continue;
            const Point2d& pos1=nodes[vid1]->getPos();
            const Point2d& pos2=nodes[vid2]->getPos();
            if( !m_CD.isCollision(geo,pos1,pos2) ){
                float w=(m_Weight[vid1]+m_Weight[vid2])/2;
                rmap.addEdge(nodes[vid1],nodes[vid2],w);
            }
        }//end j
    }//end i
}


int CDynPRM::connectToMap
(const Point2d& cfg, CRoadMap& d_map, CRoadMap& g_map, list<Point2d>& path,
bool checkDir, Vector2d dir)
{
    CFlock * m_F=d_map.getFlock();
    CRobot2D& geo=m_F->getGeometry();
    //get K closest
    CNodeData * close[K_C];
    vector<CNodeData*>& d_nodes=d_map.getNodes();
    if( d_nodes.empty() ) return -1; //nothing in the map
    CNodeData tmp(cfg);
    int k=FindKClosest(&tmp,close,d_nodes,true); //sort using weight too
    //connect from cfg to these K nodes
    {for( int j=0;j<k;j++ ){
            int vid=close[j]->getID();
            const Point2d& pos=d_nodes[vid]->getPos();
            if(checkDir) if( (pos-cfg)*dir<0 ) continue;
            if( !m_CD.isCollision(geo,cfg,pos) ){
                path.push_back(cfg);
                path.push_back(pos);
                return vid;
            }
    }}
    //hmm...can't connect directly, use g_map
    {for( int j=0;j<k;j++ ){
            int vid=close[j]->getID();
            const Point2d& pos=d_nodes[vid]->getPos();
            if( findPath_In_G_Map(cfg,pos,g_map,path) )
                return vid;
    }}
    return -1; //can't find path
}

bool CDynPRM::findPath_In_G_Map
(const Point2d& s,const Point2d& g,CRoadMap& g_map, list<Point2d>& path)
{
    unsigned int size=path.size();
    m_PRMs.findPath(g_map,s,g,path);
    if( size==path.size() ) return false; //nothing found
    return true;
}


//find K closest nodes to the given node
//return number of nodes found
int CDynPRM::FindKClosest
(CNodeData * node, CNodeData * close[K_C], vector<CNodeData*>& nodes,bool sortW)
{
    typedef list< pair<float,int> >::iterator SIT;

    ///////////////////////////////////////////////////////////////////
    //sort using dist
    list< pair<float,int> > sorted;
    //int full=0;
    int nSize=nodes.size();
    for( int iN=0;iN<nSize;iN++ ){
        if( node==nodes[iN] ) continue;
        if( nodes[iN]==NULL ) continue;
        float dist=(node->getPos()-nodes[iN]->getPos()).normsqr();
        sorted.push_back(pair<float,int>(dist,iN));
    }
    sorted.sort();
    ///////////////////////////////////////////////////////////////////
    //sort again using weight
    list< pair<float,int> > sorted2;
    if(sortW){
        int j=0;
        for(SIT i=sorted.begin(); i!=sorted.end()&&j<K_C;i++,j++)
            sorted2.push_back(pair<float,int>(m_Weight[i->second],i->second));
        sorted2.sort();
    }
    else
        sorted2.swap(sorted); //...no sort but swap
    ///////////////////////////////////////////////////////////////////
    int j=0;
    for(SIT i=sorted2.begin();i!=sorted2.end()&&j<K_C;i++,j++){
        //float w=i->first;
        close[j]=nodes[i->second];
    }

    return j;
}

bool CDynPRM::findPathV1V2
(CRoadMap& rmap, int v1, int v2, list<Point2d>& path)
{
    if( v1==v2 ) return true;
    list< CNode > nodes;
    typedef list<CNode>::iterator NIT;
    int size=BFS(rmap.getGraph(),v1,v2,nodes);
    if( size<=0 ) return false;

    for( NIT i=nodes.begin();i!=nodes.end();i++ ){
        path.push_back(i->getPos());
    }//end for

    return true;
}

//////////////////////////////////////////////////////////////////////
struct __BFSNode{
    __BFSNode(){id=-1; weight=1e10; parent=NULL; visited=false;}
    int id;
    __BFSNode * parent;
    float weight;
    int visited;
};

inline
void insert(list<__BFSNode *>& q, __BFSNode* n)
{
    typedef list<__BFSNode *>::iterator LIT;

    for( LIT i=q.begin();i!=q.end();i++ )
        if( (*i)->weight>=n->weight ){
            q.insert(i,n);
            return;
        }
    q.push_back(n);
}

int CDynPRM::BFS(WDG& g,int v1, int v2,list<CNode>& nodes)
{
    typedef vector<VID>::iterator VIT;

    //get all nodes and init it as inifite big
    int maxID=g.getVertIDs();
    __BFSNode * mynodes=new __BFSNode[maxID];
    {
        for( int i=0;i<maxID;i++ )
            mynodes[i].id=i;
    }

    list<__BFSNode *> q;
    q.push_back(&mynodes[v1]);
    mynodes[v1].weight=0;
    mynodes[v1].visited=true;

    //start to search
    bool YA=false;
    while( q.empty()==false ){
        //int qsize=q.size();
        __BFSNode * n1=q.front();
        if( n1->id==v2 ){
            YA=true;
            break;
        }
        q.pop_front();
        vector<VID> nei; //neightbors
        g.GetSuccessors(n1->id,nei);

        ///////////////////////////////////////////////////////////////////////
        for( VIT i=nei.begin();i!=nei.end();i++ ){
            __BFSNode * n2=&mynodes[*i];
            if( n2->visited )
                continue;
            n2->weight=m_Weight[*i];
            if( n2->weight<n1->weight ) n2->weight=n1->weight;
            n2->weight+=25; //penalty...bad...
            n2->visited=true;
            n2->parent=n1;
            insert(q,n2);
        }//edn for
    }//edn while

    int size=0;
    __BFSNode *curNode=&mynodes[v2];
    while( curNode!=NULL ){
        nodes.push_front(g.GetData(curNode->id));
        size++;
        curNode=curNode->parent;
    }

    delete [] mynodes;
    return size;
}

