#!/bin/bash

##
#
# This script takes the given file and replaces tags in it. 
# The format for each tag is as follows:
#		<param>=value
#
# The script will replace each occurance of <param> with value.
#
##

# require at least 2 parameters
if [ $# -lt 2 ]
then
	echo "Usage: `basename $0` <template-file> <tag> [<tag> ...]"
	exit 1
fi

# require readability of template file
if [ ! -r $1 ]
then
	echo "Error: Could not open file $1."
	exit 1
fi

template=`cat $1`
shift

while [ "$1" != "" ]; do
	param=${1%%=*}
	value=${1#*=}
	template=${template//<$param>/$value}
	shift
done

echo "$template"

