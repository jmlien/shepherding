#!/bin/sh

##
#
# Periodically check to the progress of the experiments
#
##

# require at least 2 parameters
if [ $# -lt 1 ]
then
	echo "Usage: `basename $0` <string> <token> [<token> ...]"
	exit 1
fi

prev_count=-1

while [ true ]
do
	count=`ls *.out | wc -l`
	if [ $count -ne $prev_count ]; then
		echo `date` ---  $count of `ls file*.mp | wc -l`
		let prev_count=count
	fi
		
	sleep 1
	#echo ding!

done
