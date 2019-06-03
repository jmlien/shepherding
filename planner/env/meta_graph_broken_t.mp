
#
# define tree
# nbrtRRT, nbrmRRT, dbrmRRT, rbrmRRT
#
graph = (fuzzy n=500, k=150, l=3, tree=(dbrmRRT k=100 bias=1))


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
WKSpace=broken_t.sh

# Random seed for mersenne twister
# seed=1235434732
