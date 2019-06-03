
#
# define tree
# nbrtRRT, nbrmRRT, dbrmRRT, rbrmRRT
#
tree=(dbrmRRT k=1000 bias=0.95) 

#
# define local planner
# st or sb
#
LP=(sb t=100)

#
# define distance metric
# flock or flockgeo or shepherd
#
DM=flock

# define workspace (in *.ws file)
WKSpace=test_1.sh

# Random seed for mersenne twister
# seed=43
