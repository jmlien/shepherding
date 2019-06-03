
#
# define tree
# nbrtRRT, nbrmRRT, dbrmRRT, rbrmRRT
#
graph = (fuzzy n=5000, k=15, tree=(dbrmRRT k=75 bias=1))


# define local planner
# st or sb
#
LP=(sb t=30)

#
# define distance metric
# flock or flockgeo or shepherd
#
DM=flock

# define workspace (in *.ws file)
WKSpace=maze.sh

# Random seed for mersenne twister
# seed=1235268893
#seed=1235307187
