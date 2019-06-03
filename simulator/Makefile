
all: MAKE_DIRS


.PHONY : MAKE_DIRS 
MAKE_DIRS: _SRC BEHAVIORS

_SRC: 
	cd src; $(MAKE)

BEHAVIORS: _SRC
	cd behaviors.c; $(MAKE)



.PHONY : clean
clean : CLEAN_SRC  CLEAN_BEHAVIORS

CLEAN_SRC:
	cd src; $(MAKE) clean

CLEAN_BEHAVIORS:
	cd behaviors.c; $(MAKE) clean

