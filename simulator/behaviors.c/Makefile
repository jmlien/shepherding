SH_DIR = ../src
include ../M_incl

#--------------------------------------------------------------------

TARGET = shepherding
BH_LIB = ../libs/libbehaviors.a
MYLIBS = -L. -lbehaviors -L$(SH_DIR)/../libs -lshepherd

SUBDIRS = . basicflock multi_herding simple_herding \
          better_herding manual_herding deform_herding movable_obstacle

INCLUDE += $(addprefix -I,$(SUBDIRS))

SRCS=$(wildcard $(addsuffix /*.cpp,$(SUBDIRS)))
OBJS=${SRCS:.cpp=.o}

#--------------------------------------------------------------------
# add device interface if boost is defined
#--------------------------------------------------------------------

ifndef NO_BOOST
  OBJS  += server_client_interface/DeviceState.o  \
           server_client_interface/DeviceStateClient.o
  MYLIBS += $(BOOST_LIBS)
  CXXFLAGS += -Iserver_client_interface $(BOOST_INC) -g3
endif

CXXFLAGS += -DDRAW_EXTRA=0 
#--------------------------------------------------------------------
# PROGRAMS
#--------------------------------------------------------------------
.PHONY: all

all :  $(TARGET)

$(BH_LIB):  $(OBJS) 
	ar ruvs $@ $(OBJS)

$(TARGET): main.o $(BH_LIB) $(SH_DIR)/../libs/libshepherd.a
	$(CXX) ${CXXFLAGS} -o $@ main.o $(MYLIBS) $(LIB)



#--------------------------------------------------------------------

Dependencies :
	touch Dependencies

include Dependencies


clean:
	rm -f *.o *.d \
		$(OBJS) $(TARGET) $(BH_LIB) Dependencies \
		server_client_interface/*.d server_client_interface/*.o


