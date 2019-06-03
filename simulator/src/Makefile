SH_DIR = .
include ../M_incl

TARGET=../libs/libshepherd.a
#--------------------------------------------------------------------
SRCS=$(wildcard $(addsuffix /*.cpp,$(SH_SUB_DIRS)))
OBJS=${SRCS:.cpp=.o}

$(TARGET): $(OBJS)
	mkdir -p ../libs
	ar ruv $@ $(OBJS)
		
#--------------------------------------------------------------------
clean:
	rm -f *.o Dependencies $(OBJS) libs/* *~ $(TARGET)

Dependencies:
	touch Dependencies

include Dependencies

