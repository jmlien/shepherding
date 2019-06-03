#####################################################################
#
# Jyh-Ming Lien jmlien@cs.gmu.edu
# last update: Aug 26 2009
#
#####################################################################

This file gives an brief desciption of classes and 
function provided by this library.

#####################################################################

!Note, function may be overloaded or default values are given.
When default values are given, they are enclosed in [].

#-------------------------------------------------------------------
# System Functions
#-------------------------------------------------------------------

void start();                    # start simulation
void stop();                     # stop sumulation
setEnvironment(Environment&);    # set environment
Environment& getEnvironment();   # get environment
setTimeStep(int s);              # set simulation time step
int getCurrentTimeStep();        # get current simulation time
setConvertFile(string file);     # set the location of "convert" program
setRestitution(double r);        # bouncing restitution

#-------------------------------------------------------------------
# Environmental Classes
#-------------------------------------------------------------------

#-------------------------------------------------------------------
CLASS NAME:
Environment

MEMBER FUNCTION:
setBBX( double min_x,double max_x,      # xz plane is the ground
        double min_y,double max_y,
	double min_z,double max_z );
addFlock(Flock&);
addObstacle(Obs&);

#-------------------------------------------------------------------
CLASS NAME:
Flock
(string& geo_file, [int size=50]) # geo_file is the file name of *.g or *.obj
                                  # if geo_file is in form of name####.g
				  # The libiray will look for file name0000.g
				  # to name9999.g for canned animation.
                                  # size is the reserved space for states

MEMBER FUNCTION:
void addState(FlockState&);
FlockState& getState(int);                # get i-th state
vector<FlockState&>& getStates();         # get all states
int getStateSize();                       # number of states
deployFlock(Environment&,                 # deploy the flock
            const Point2d& pos,           # <--center
	    double dev,                   # <--deviation
            [double vm=1,                 # <--velocity magnitude
	    Vector2d dir=Vector2d(0,0)]); # facing dir

setColor(double r,double g,double b); # set color, R,G,B
setTexture(string filename);          # set texture
int getID();                          # get the unique id of the flock

setForceRule(ForceRule&);
ForceRule& getForceRule();
CBehaviorRule& getBehaviorRule();
setBehaviorRule(CBehaviorRule&);
	
DATA MEMBER:
double Mass;
double ViewRadius;
double ViewAngle;

#-------------------------------------------------------------------
CLASS NAME:
Obs
(string& filename, int size);   # the name of *.g gile, size is number 
                                # of obstacles

MEMBER FUNCTION:
ObsState& getState(int i);  # get i-th obstacle state
int getStateSize();         # get number of the obstacles
bool buildGLModel();        # build GL model. Normally this is called 
                            # when the obstacle is added to the env. 
			    # Sometimes, we don't want to add obst to 
			    # env..this must be called in order 
			    # to visualize it

#-------------------------------------------------------------------
# Simulation Classes
#-------------------------------------------------------------------

#-------------------------------------------------------------------
CLASS NAME:  
FlockState(Flock&);  # what kind of flock it is

MEMBER FUNCTION:  
setPos(const Point2d&);
const Point2d& getPos();
setVelocity(const Vector2d&);
const Vector2d& getVelocity();
Flock& getType();                  # what kind of flock this is
int getID();                       # unique id in all states
CBehaviorRule& getBehaviorRule();  # get rule associated with state
setBehaviorRule(CBehaviorRule&);   # associated the rule with state
bool seeObstalce();                # yes, if obs is in the front
const Point2d& getVisibleObstPt    # the closet point on visible obst
const Vector2d& getVisibleObstN(); # the normal of the closet point
list<FlockState&> getVisibleAgent  # all visible agent states
setVisibleAgent(list<FlockState&>);# set visible flock members
Vector2d getFacringDir();          # get the heading direction

#-------------------------------------------------------------------
# Global Function

# check if s1 can see s2 or not
# if env is given, visibility checking will include the environment
# otherwise, environment is not under concern.

bool isVisible(FlockState& s1,FlockState& s2,[Environment& env])

# check if s1 can see pt or not
# if env is given, visibility checking will include the environment
# otherwise, environment is not under concern.

bool isVisible(FlockState& s1, const Point2d& pt,[Environment& env])

# group given list according their visibility
# if env is given, visibility checking will include the environment
# otherwise, environment is not under concern.

list< list<FlockState&> > getGroups(list<FlockState&>,[Environment& env]);

#-------------------------------------------------------------------
CLASS NAME:
MapFlockState : FlockState  # derive from FlockState
(Flock&,RoadMap&);          # flock and roadmap

MEMBER FUNCTION:
list<Point2d&>& getGoals();
pushGoal(Point2d&);         #push into stack
void popGoal();             #pop the top of the stack
Point2d& peepGoal();        #view the top of the stack
bool isGoalEmpty();         #yes if nothing in the stack
list<NodeData&> getMems();  #a list of visited nodes
pushMem(NodeData&);
void popMem();
NodeData& peepMem();
bool isMemEmpty();
RoadMap& getMap();          #retrive the map
		       
#-------------------------------------------------------------------
CLASS NAME:
ObstState();  # can't be created in python
              # get it from Obs' getState

MEMBER FUNCTION:
setPos(Point2d& pos);
setRot(double degree);                  # 0~360
setColor(double r, double g, double b); # R,G,B
setTexture(string name);

#-------------------------------------------------------------------
CLASS NAME:
ForceRule   # abstract class
();

MEMBER FUNCTION:
Vector2d getForce(FlockState&);  #not implemented

#-------------------------------------------------------------------
CLASS NAME:
BasicForceRule : ForceRule      # derive from ForceRule
();                        

MEMBER FUNCTION:
Vector2d getForce(FlockState&); # implement basic flocking forces

DATA MEMBER :
double Cohesion;                # flocking coefficients
double Separation;
double Alignment;
double ObstRepulsion;
double MaxForce;
double Damping;

#-------------------------------------------------------------------
CLASS NAME:
BehaviorRule # abstract
();

MEMBER FUNCTION:
void applyRule(FlockState&);

#-------------------------------------------------------------------
# PRM Classes
#-------------------------------------------------------------------

#-------------------------------------------------------------------
CLASS NAME:
RoadMap             # Flock : the Flock used to generete this map
(Flock&,int n);     # n: the reserved space of storeing nodes

MEMBER FUNCTION:
NodeData& getNode(int id);           # get node with id
int getNodeSize();                   # number of nodes in the map
addEdge(NodeData& n1, NodeData& n2); # connect nodes n1 and n2
addEdge(int id1, int id2);           # connect nodes with id1 and id2
	
#-------------------------------------------------------------------
(not tested yet)
CLASS NAME:
NodeData     # map node
(const Point2d&);

MEMBER FUNCTION:
void changeWeight(CNodeData& to,double d);    # incr/decr edge weight
NodeData& getRandSuccessor(CNodeData& from);  # get random nodes
int getID();
Point2d& getPos();

#-------------------------------------------------------------------
CLASS NAME:
PRMs
(Environment&);

MEMBER FUNCTION:

sampleMAPRMNodes(RoadMap&,int n,[float s=0]);    # sample n MAPRM nodes
                                                 # s is required space bettwen
						 # sampled nodes.
						 
sampleOBPRMNodes(RoadMap&,int n,[float s=0]);    # sample n OBPRM nodes
                                                 # s is required space bettwen
						 # sampled nodes.
						 
sampleOBPRMNodes_on_Obst(CObs&,RoadMap&,int n,   # sample n OBPRM nodes
                         [float s=0]);           # around the given obstacle
			                         # s is required space bettwen
						 # sampled nodes.
						 
connectNodes(RoadMap&,int k);                    # connect nodes in map 
                                                 # using k-closest
						 # k is just recommand.

CNodeData& connect2Map(RoadMap&,Point2d& s);     # connect point s to the map

findPath(RoadMap&,Point2d& s, Point2d& g,        # find a path from s to g
         list<Point2d>& path, [bool smooth=T]);  # smooth will smooth the path

findPath(RoadMap&, int v1, int v2,               # find a path from v2 to v2
         list<Point2d>& path, [bool smooth=T]);  # smooth will smooth the path

#-------------------------------------------------------------------
CLASS NAME:
DynPRM               # dynamic PRM
(Environment&);

MEMBER FUNCTION:

# find a path from s to g, if failed, expand the map
bool queryPath(Point2d s,Point2d g,Roadmap dmap,
               Roadmap gmap, FlockState shepherd,
	       list<FlockState*> flock, list<Point2d> path);
	      
# add nodes to the map
void expandMap
(Roadmap,FlockState shepherd,list<FlockState*> flock);

# remove nodes from the map
void updateMap
(Roadmap,FlockState shepherd,list<FlockState*> flock);

#-------------------------------------------------------------------
# Utility Classes / Functions
#-------------------------------------------------------------------

#-------------------------------------------------------------------

# check if s is in collision
bool isCollision(Environment& e, const FlockState& s);

# check if the straight line between s1 and s2 is in collision
bool isCollision(Environment& e, const FlockState& s1, const FlockState& s2);

# check if the straight line between s and pt is in collision
bool isCollision(Environment& e, const FlockState& s, const Point2d& pt);

# check if the flock is in collision with the given obstacle.
bool isCollision(Obs&,FlockState&);

# push the point to the MA, clearance of the point on MA will
# be returned. pt must be collision free.
double Push2MedialAxis(Environment, FlockState s, Point2d pt);

# push the pt to free space if pt is in collision.
Push(Environment, FlockState s, Point2d pt);

# push the pt to free space in given direction if pt is in collision.
PushInDir(Environment, FlockState s, Vector2 dir, dPoint2d pt);

#-------------------------------------------------------------------
CLASS NAME:
Point2d(double,double);
Point3d(double,double,double);

MEMBER FUNCTION:
Point* +  Vector*
Point* -  Point*
Point* == Point*

DATA MEMBER:
x
y
z   # for Point3d only

#-------------------------------------------------------------------
CLASS NAME:
Vector2d(double,double);
Vector3d(double,double,double);

MEMBER FUNCTION:
Vector* normalize();  # return normalized vector
double norm();        # lenght of the vector
double normsqr();     # square of the lenght of the vector
Vector* -  Vector* 
Vector* +  Vector* 
Vector* *  Vector*     # dot product
Vector* *  double      # scale
double  *  Vector* 
Vector* /  double
Vector* %  Vector*     # cross product
Vector* == Vector* 

DATA MEMBER:
x
y
z   # for Point3d only

#-------------------------------------------------------------------
# STL classes
#-------------------------------------------------------------------

CLASS NAME:
NodeDataList      # i.e. list<CNodeData&>
size().

CLASS NAME:
FlockStateList    # i.e. list<CFlockState&>
size(), push_back(), empty(), clear(), front().

CLASS NAME:
FSListList        # i.e. list< list<CFlockState&> >
size(), push_back(), empty(), clear(), front().

CLASS NAME:
FlockStateVector  # i.e. vector<CFlockState&>
size().

CLASS NAME:
PtList            # i.e. list<Point2d>
size(), reverse(), pop_back(), pop_front()
empty(), clear().

#-------------------------------------------------------------------
# Drawing Classes/Functions
#-------------------------------------------------------------------

#-------------------------------------------------------------------
CLASS NAME:
glDraw();    # abstract class, user should derived this in python
             # and add it using addDrawObj.

MEMBER FUNCTION:
draw();     # not implemented

#-------------------------------------------------------------------
# drawing functions

# general opengl drawing
addDrawObj(glDraw&);
removeDrawObj(glDraw&);

# drawing text
addDrawInfo(string& tag, string& info); # draw text info on screen
removeDrawObj(string& tag);

drawMap(CRoadMap&, [bool b_ShowID=false]); # draw the roadmap [with id]

drawPath(list<Point2d>&,                   # draw path
         [double r=1,bool text=false]);    # r is radius of the point

drawCircle(doulbe r)                       #draw a cirle with radius given

#-------------------------------------------------------------------
CLASS NAME:
DrawCoverage                                   # draw covered/uncovered
(Environment, Flock,                           # env and flock
 int xsize,int zsize,                          # width and depth
 [float H=1,                                   # height
  float R=0,float G=0,float B=0,float A=0.5)]; # color RGBA
	 
MEMBER FUNCTION:
draw();           # not implemented
update(Point2d, float value, [float vr=0]); # decrease the value
                                            # of cells in the circle centered
                                            # at give the point and radius
					    # when vr==0, flock's vr is used
float coverageRate();   # % of covered area

#-------------------------------------------------------------------
# Exported OpenGL functions

glTranslated
glRotated
glPushMatrix
glPopMatrix
glVertex3d
glVertex2d
glBegin
glEnd
glutSolidSphere
glutSolidCube
glNormal3d
glColor3d
glDisable
glEnable
glLineWidth
glPointSize
glGenLists
glNewList
glEndList
glCallList

GLenum.GL_POINTS
GLenum.GL_LINES
GLenum.GL_LINE_LOOP
GLenum.GL_LINE_STRIP
GLenum.GL_TRIANGLES
GLenum.GL_LIGHTING
GLenum.GL_COMPILE

