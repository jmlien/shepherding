/*
 * shepherding_base.h
 *
 *
 * This file includes almost all base classes that
 * will be used for multi-agent flocking simulation
 * 
 * Last Major Modification : J-M Lien 12/28/2009
 *
 */

#ifndef _SHEPHERDING_BASE_H_
#define _SHEPHERDING_BASE_H_

//some standard stuff
#include <iostream>
#include <sstream>
#include <list>
#include <typeinfo>
#include <iomanip>
#include <float.h>
#include <limits.h>
using namespace std;


//mathtool headers
#include "Point.h"
#include "Vector.h"
#include "Matrix.h"
using namespace mathtool;

//utilities
#include "logger.h"
#include "RNG.h"

//graph
#include "graph/Graph.h"
#include "graph/GraphAlgo.h"
using namespace graph;

//
#include "sh_Environment.h"
#include "sh_FlockState.h"
#include "sh_ForceRules.h"
#include "sh_ObstState.h"
#include "sh_CollisionDetection.h"
#include "sh_Robot2D.h"
#include "sh_ParticleSolver.h"
#include "sh_BehaviorRules.h"
#include "sh_MapBasedFlockState.h"
#include "sh_PRM.h"
#include "sh_FlockFunc.h"

//guiless simulation
#include "sh_sim.h"

//gl-based simulation
#include "gl_draw.h"
#include "sh_sim_GL.h"


#endif //_SHEPHERDING_BASE_H_
