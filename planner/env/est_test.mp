#
# EST Configuration File
# Tree choices are:
# - BasicEST
# - NaiveEST
# - MinEST
# - HeuristicEST
#
# n is the number of expansions
#   default 200
# 
# nhood_size: the size of the neighborhood to search
#   as a distance from a configuration point
#   default 3.0
# 
# nhood_func: the function that is used to expand into
#   a neighborhood. choices are:
# - uniform 
# - gaussian
# - gauss_ring
# - circle (default)
#
tree=(MinEST n=500 nhood_size=3 nhood_func=circle simsteps=500000) 

#
# define local planner
# st or sb
# t: the steps to run the simulation
#
LP=(sb t=100)

#
# define distance metric
# flock or flockgeo or shepherd
#
DM=flock

# define workspace (in *.ws file)
WKSpace=est_test.sh

# mersenne twister seed
# seed=43

