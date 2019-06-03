#!/bin/bash

##
#
# This script processes the output of the shepherding experiments, grepping
# .out files and making .dat files. It also prints out the number of successful 
# runs compared to the total runs for each configuration.
#
##


maps="test_1 broken_t spiral maze maze_6"

# Function to runs stats with a given pattern
do_stats () {
   if [ -z "$1" ]                           # Is parameter #1 zero length?
   	then
		echo "-Parameter #1 is zero length.-"  # Or no parameter passed.
	  	exit 1
   	fi
		
	pattern=$1
	
	# make sure some files exist that match the pattern
	num_files=`ls job.*.${pattern}.out | wc -l`
	if [ $num_files -eq 0 ]
		then
		echo "No files matching job.*.${pattern}.out"
		return 1
		fi
		
	# print the number of successful runs out of the total
	num_found=`grep "Path found in" job.*.${pattern}.out | wc -l`
	echo "-- ${pattern}: found ${num_found} of ${num_files}"
	
	#return 0
	
	# make files showing the success rates growing over time
	echo "0 0" > ${pattern}.success
	grep "Path found in" job.*.${pattern}.out | awk '{print $4}' | sort -n | awk '{print $1, NR}' >> ${pattern}.success
	temp=`tail -1 ${pattern}.success`
	echo "${temp}" | awk '{print 400000, $2}' >> ${pattern}.success

	return 0
	
	for file in job.*.${pattern}.out
		do
		if [ ! -e ${file/%.out/.dat} ]
			then
			grep DistanceToGoal $file | awk '{ print $2 " " $4 }' > ${file/%.out/.dat}
			fi
		done
	
		
	# pad any files that ended early
	sh ~/research/scripts/checkNumLines.sh "job.*.${pattern}.dat"
	
	#return 0	
	if [ ! -e ${pattern}.var ]
		then
		echo "--- Calculating stats for: ${pattern}"
		Rscript ~/research/scripts/calcStats.R "." "job.*.${pattern}.dat" "${pattern}.var"
		fi
			
   return 0
}



# Sim Only
for map in ${maps}
	do	
	do_stats "sim_only.${map}"
	done

# MinEST, NaiveEST, BasicEST
for est in MinEST NaiveEST BasicEST
	do
	for map in ${maps}
		do
		for n in 4000
			do
			for nhood_size in 3 5 7 9
				do
				do_stats "${est}.${map}.n.${n}.nhood_size.${nhood_size}"
				done
			done
		done
	done
	

# rbrmRRT, dbrmRRT
for rrt in rbrmRRT dbrmRRT
	do
	for map in ${maps}
		do
		for k in 4000
			do
			do_stats "${rrt}.${map}.k.${k}"
			done
		done
	done
	

# Meta Graph: fuzzy
for meta_graph in fuzzy
	do
	for map in ${maps}
		do
		do_stats "${meta_graph}.${map}"
		done
	done
	


