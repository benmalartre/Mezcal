# Alembic makefile
BULLET_PATH = /Users/benmalartre/Documents/RnD/bullet3-2.83.7

# Includes
BULLET_INCLUDES = -I$(BULLET_PATH)/src
BULLET_LIBRARIES = -L$(BULLET_PATH)/bin

CFLAGS += -std=c++11  -DNDEBUG -c $(BULLET_INCLUDES) -static

#LIBS += /Users/benmalartre/Documents/RnD/bullet3-2.83.7/bin/#libBullet3Collision_gmake_x64_release.a
#LIBS += /Users/benmalartre/Documents/RnD/bullet3-2.83.7/#libBullet3Common_gmake_x64_release.a
#LIBS += /Users/benmalartre/Documents/RnD/bullet3-2.83.7/bin/#libBullet3Dynamics_gmake_x64_release.a
#LIBS += /Users/benmalartre/Documents/RnD/bullet3-2.83.7/bin/#libBullet3Geometry_gmake_x64_release.a

LDFLAGS += -static $(LIBS)

# Rules

all: CAPI.o BulletCAPI.a
	
CAPI.o: capi.cpp capi.h
	g++ $(CFLAGS) -c -Wall -g capi.cpp

BulletCAPI.a:  CAPI.o
	ar rcs $@ $^ 

clean:
	rm -rf *.o *.gah