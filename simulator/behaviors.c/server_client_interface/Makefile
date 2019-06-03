CXX=g++
CXXFLAGS=-ggdb -Wall

SCRIPTS_DIR      := ../../../scripts
GETBOOST   := $(SCRIPTS_DIR)/getboost
BOOST_LIBS=`$(GETBOOST) --libs  boost_system   boost_thread`
BOOST_INC  := `$(GETBOOST) --cflags`

CXXFLAGS += $(BOOST_INC) -g3
LFLAGS=-L/usr/local/lib -L/usr/lib $(BOOST_LIBS)
RM=rm -f

OBJS=DeviceStateClient.o DeviceStateServer.o DeviceState.o client.o

PROGS= client            \
	   server

all : $(PROGS)




client : client.o DeviceStateClient.o DeviceState.o
	$(CXX) $(LFLAGS) $+ -o $@

#DeviceStateServer: DeviceStateServer.o DeviceState.o
#	$(CXX) $(LFLAGS) $+ -o $@
	
server : server.o DeviceStateServer.o DeviceState.o
	$(CXX) $(LFLAGS) $+ -o $@
	
#DeviceState: DeviceState.o
#	$(CXX) $(LFLAGS) $+ -o $@


%.o : %.cpp common.h
	$(CXX) $(CXXFLAGS) $< -c -o $@


.PHONY : clean
clean:
	$(RM) $(PROGS) $(OBJS) *.d
