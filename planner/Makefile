SH_DIR = ../simulator/src
BH_DIR = ../simulator/behaviors.c
include ../simulator/M_incl

####### Files
SRCS=$(wildcard *.cpp)
OBJECTS = ${SRCS:.cpp=.o}

MYLIB = -L$(SH_DIR)/../libs -lbehaviors -lshepherd

BH_SUBDIRS = basicflock multi_herding simple_herding \
             better_herding balanced_herding manual_herding deform_herding \
	   	     balanced_herding/BEHAVIOR .

BH_INCL += $(addprefix -I$(BH_DIR)/,$(BH_SUBDIRS))



CXXFLAGS += $(OPTS) $(BH_INCL)

TARGET = shpmp

#--------------------------------------------------------------------
# PROGRAMS
#--------------------------------------------------------------------
.PHONY: all

all :  $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) ${CXXFLAGS} -o $@ $(OBJECTS) $(MYLIB) $(LIB)

clean:
	-rm -f $(OBJECTS) $(TARGET) *~ core *.core  Dependencies *.o

Dependencies:
	touch Dependencies

-include Dependencies

