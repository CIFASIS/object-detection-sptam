CC=g++

DIR_MODULES=../..
DIR_CONFIGURATION=$(DIR_MODULES)/Configuration
DIR_FRAMEGENERATOR=$(DIR_MODULES)/FrameGenerator
DIR_CORE=$(DIR_MODULES)/Core
DIR_LIBCAM=$(DIR_MODULES)/libcam
DIR_LOCALIZATION=$(DIR_MODULES)/Localization


DIR_SOURCE = $(DIR_CONFIGURATION) $(DIR_FRAMEGENERATOR) $(DIR_CORE) $(DIR_LIBCAM) $(DIR_LOCALIZATION)

INCLUDE = $(addprefix -I,$(DIR_SOURCE))

EXECUTABLE=ReadCameras
DIR_BUILD=../../../build

CFLAGS=-Wall
# OpenCV
LFLAGS= $(CFLAGS) `pkg-config --cflags --libs opencv`


OBJECTS=$(DIR_BUILD)/$(EXECUTABLE).o $(DIR_BUILD)/CameraFrameGenerator.o $(DIR_BUILD)/MotionModel.o $(DIR_BUILD)/EKFMath.o $(DIR_BUILD)/libcam.o $(DIR_BUILD)/GlobalConfig.o

all: checkdirs $(OBJECTS)
	$(CC) -o $(EXECUTABLE) $(OBJECTS) $(LFLAGS) $(INCLUDE)

$(DIR_BUILD)/$(EXECUTABLE).o:$(EXECUTABLE).cpp
	$(CC) -c -o $@ $< $(CFLAGS) $(INCLUDE)

$(DIR_BUILD)/GlobalConfig.o:$(DIR_CONFIGURATION)/GlobalConfig.cpp $(DIR_CONFIGURATION)/GlobalConfig.h
	$(CC) -c -o $@ $< -Wall $(INCLUDE)

$(DIR_BUILD)/CameraFrameGenerator.o:$(DIR_FRAMEGENERATOR)/CameraFrameGenerator.cpp $(DIR_FRAMEGENERATOR)/IFrameGenerator.h $(DIR_FRAMEGENERATOR)/CameraFrameGenerator.h $(DIR_BUILD)/libcam.o
	$(CC) -c -o $@ $< $(CFLAGS) $(INCLUDE)

$(DIR_BUILD)/MotionModel.o:$(DIR_LOCALIZATION)/MotionModel.cpp $(DIR_LOCALIZATION)/MotionModel.h $(DIR_LOCALIZATION)/State.h
	$(CC) -c -o $@ $< $(CFLAGS) $(INCLUDE)

$(DIR_BUILD)/libcam.o: $(DIR_LIBCAM)/libcam.cpp $(DIR_LIBCAM)/libcam.h
	$(CC) -c -o $@ $< $(CFLAGS) $(INCLUDE)

$(DIR_BUILD)/EKFMath.o: $(DIR_CORE)/EKFMath.cpp $(DIR_CORE)/EKFMath.h $(DIR_CORE)/Base.h
	$(CC) -c -o $@ $< $(INCLUDE)

#crea el directorio ../build si no existe
checkdirs: $(DIR_BUILD)

$(DIR_BUILD):
	@mkdir -p $@
########################################


clean:
	rm -rf $(EXECUTABLE) $(DIR_BUILD)/*
