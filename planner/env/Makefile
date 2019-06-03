##
#
# Example Makefile for running experiments in parallel
# 
# It uses the JOB_LIST script to obtain a list of experiments to run.
# It creates a parameter file for each experiment using the REPLACE script 
# to insert the parameters from the JOB_LIST into a TEMPLATE parameter file,
# and then runs PROGRAM on the parameter file to execute the experiment.
#
##

# Locations of GNU shell utilities
AWK := awk
GREP := grep

# Locations of the scripts and template
PROGRAM := ../shpmp
TEMPLATE := template.mp
REPLACE := ./replace-tags.sh
JOB_LIST := ./sweep-params.sh
JOB_CACHE := jobcache.dat

# Run the JOB_LIST command to get 1st column (names of the jobs)
JOB_NAMES := $(shell ${JOB_LIST} | $(AWK) '{print $$1}')

# Generate the list of targets using JOB_NAMES
OUT_FILES := $(addprefix job.,$(addsuffix .out,${JOB_NAMES}))
PARAM_FILES := $(addprefix file.,$(addsuffix .mp,${JOB_NAMES}))

# ---- Useful Functions --------------------------------------

# function to get everything except first item in a given list
rest=$(wordlist 2,$(words $1),$1)

# function to query JOB_LIST to get params for a given job name  
get_params=$(shell $(GREP) -m 1 $1 $(JOB_CACHE))

# ---- Rules --------------------------------------------------

all: ${OUT_FILES} ${PARAM_FILES}

job.%.out : file.%.mp
	$(PROGRAM) -g $< > $@ 2> $(basename $@).err

file.%.mp : $(JOB_CACHE)
	$(REPLACE) $(TEMPLATE) $(call get_params,$*) 2>&1 > $@

${JOB_CACHE} :
	$(shell ${JOB_LIST} > $@)

clean:
	rm -rf ${JOB_CACHE} file.*.mp job.*.out job.*.err *~
