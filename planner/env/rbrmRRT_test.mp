
#
# define tree
# nbrtRRT, nbrmRRT, dbrmRRT, rbrmRRT
#
tree=(rbrmRRT k=100) 

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
WKSpace=rbrmRRT_test.sh

# mersenne twister seed
# seed=43

