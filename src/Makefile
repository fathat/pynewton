OBJS=body.cpp collision.cpp contact.cpp joint.cpp heightfieldwrapper.cpp HeightFieldCollision.cpp material.cpp materialgroup.cpp ragdoll.cpp utility.cpp world.cpp wrapper.cpp pynewton_wrap.cxx
SRC=`echo $(OBJS) | sed 's/.o/.cpp/g'`
TARGET=_pynewton.so
VERSION=0.0.1
LIBS=-lc libNewton.a
INCLUDEPATHS=-I/usr/include/python2.5 -InewtonSDK
CC=g++
CFLAGS=-shared

all: pynewton
	
.cpp.o:
	$(CC) $(CFLAGS) -c $*.cpp -o $*.o

.PHONY: pynewton clean depend

pynewton: $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LIBS) $(INCLUDEPATHS)
	
clean:
	rm -rf $(TARGET) *.o
	
depend:
	@makedepend $(CFLAGS) -Y $(SRC) 2> /dev/null

