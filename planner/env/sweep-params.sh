#!/bin/bash

##
#
# This script creates a list of experiments to run
# 
# - The first column is the name of the experiment
# - The remaining columns are parameters to use for the experiment 
#
##

# Maps:
#	test_1.sh
#	test_2.sh (omit)
#	broken_t.sh
#	spiral.sh
#	maze.sh
#	maze_6.sh

# Methods:
#	meta_graph: graph = (fuzzy n=850, k=15, l=3, simsteps=500000000, tree=(dbrmRRT k=75 bias=1))
#	sim_only: tree=(None simsteps=1000000)
#	est: tree=(MinEST n=500 nhood_size=3 nhood_func=circle simsteps=1000000)
#		NaiveEST ??
#		BasicEST ??
#		HeuristicEST ??
#	rbrmRRT: tree=(rbrmRRT k=1000 bias=0.95 simsteps=1000000) 
#	dbrmRRT: tree=(dbrmRRT k=1000 bias=0.95 simsteps=1000000)

seed=10205 # initial seed
simsteps=400000
numruns=50

maps="spiral_6	test_1_6 maze2_6	maze_6	"

# Meta Graph: fuzzy
for meta_graph in fuzzy
	do
	for map in ${maps}
		do
		for ((i=0; i<${numruns}; i++))
			do				
			echo -n "${i}.${meta_graph}.${map}"
			echo -n " mp=\"graph=(${meta_graph} n=5000 k=150 l=3 simsteps=${simsteps} tree=(dbrmRRT k=100 bias=1))\""
			echo -n " lp=sb lp-params=\"t=100\""
			echo -n " sh-file=${map}.sh"
			echo -n " dm=flock"
			echo " seed=${seed}"
			let seed++
			done
		done
	done
