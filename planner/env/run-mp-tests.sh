#!/bin/sh

# Write a .mp file to be used with shpmp
# Usage: run-mp-tests.sh num-runs

if [ $# -lt 1 ]
then
	echo "Usage: `basename $0` <num-runs>"
	exit 1
fi

seed=347326		# starting seed. seeds will increment from here.

for ((i=0; i<$1; i++))
	do
	echo  --- job $i
			
	./replace-tags.sh template.mp \
		tree=MinEST \
		tree-params="n=234 nhood_size=7" \
		lp=sb \
		lp-params="t=100" \
		dm=flock \
		sh-file=est_test.sh \
		seed=$seed \
		> temp.sh
		
	../shpmp -g temp.sh > "job.$i.out"
	let seed++
	done
	