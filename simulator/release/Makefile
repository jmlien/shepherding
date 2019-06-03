OSNAME := $(shell uname -s)
MACHINE := $(shell uname -m)

ifeq ($(OSNAME),Linux)
  ifeq ($(MACHINE),x86_64)
    PLATFORM := linux64
  else
    PLATFORM := linux
  endif
else 
  ifeq ($(OSNAME),Darwin)
    PLATFORM := osx
  else
    PLATFORM := other
  endif
endif

TODAY := $(shell date '+%b-%d-%Y')
TARGET=release-$(TODAY)
LIB=$(TARGET)/lib
INCL=$(TARGET)/include

#--------------------------------------------------------------------
$(TARGET): clean
	mkdir -p $(LIB)
	mkdir -p $(INCL)
	cp -r ../src $(INCL)
	cp -r ../behaviors.c  $(INCL)
	rm -rf `find $@ -name "*.cpp"` 
	rm -rf `find $@ -name ".svn"`
	rm -rf `find $@ -name "Makefile"`
	rm -rf `find $@ -name "vc_files"`
	cp M_incl $@
	mv $(INCL)/src $(INCL)/base
	mv $(INCL)/behaviors.c $(INCL)/behaviors
	cd ..; make;
	cp ../libs/libbehaviors.a $(LIB)/libbehaviors.$(PLATFORM).a
	cp ../libs/libshepherd.a $(LIB)/libshepherd.$(PLATFORM).a

	
#--------------------------------------------------------------------
clean:
	cd ..; make clean;

reallyclean: clean
	rm -rf $(TARGET)


