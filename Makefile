RND = /Users/benmalartre/Documents/RnD

CXXFLAGS = 	-Wall\
		-O3 \
		-D_LANGUAGE_C_PLUS_PLUS\
		-mmacosx-version-min=10.9\
		-isysroot /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.9.sdk \
		-I$(RND)/bullet3-2.83.7/src \
		-I$(RND)/Mezcal/gl3w \
		-o prog \
		-D_GNU_SOURCE \
		-L$(RND)/Mezcal/libs

LIBS =	-std=c++11 \
	-framework Cocoa \
	-framework OpenGL \
	-framework IOKit \
	-framework CoreVideo \
	-lglfw3 \
	-lLinearMath \
	-lBulletCollision \
	-lBulletDynamics \
	-lBulletInverseDynamics \
	-lBulletSoftBody

# Should be equivalent to your list of C files, if you don't build selectively
SRC=$(wildcard framework/*.cpp) tinycthread/tinycthread.c gl3w/gl3w.c main.cpp
OBJS=$(patsubst %.cpp,%.o,$(SRC))


all: $(SRC)
	g++-7 -o $@ $^ $(CXXFLAGS) $(LIBS)

#gl3w: gl3w/gl3w.c
#	g++-7 -o gl3w $(CXXFLAGS) $(LIBS)
	
clean:
	rm -f $(OBJS) *.o