CXX = g++
CC = $(CXX)
#CXXFLAGS = -std=c++17 -Wall
#LDLIBS = -lm

SRC=../Src
BUILD=Build
PRO=Protocol

TARGET = interface_cs
TARGETo = $(TARGET).o
TARGETcpp = $(TARGET).cpp

ROSDIST = noetic

ROSoptions = -I/opt/ros/$(ROSDIST)/include -L/opt/ros/$(ROSDIST)/lib -lroscpp -lrostime -lrosconsole -lroscpp_serialization

# check out wildcard
# SRC = src
# SOURCES := $(wildcard $(SRC)/*.cpp)


$(TARGET): $(TARGETo) IOBus.o MessageBus.o NetworkBus.o NetworkClientIO.o NetworkServerIO.o
	$(CC) -pthread $(TARGETo) IOBus.o MessageBus.o NetworkBus.o NetworkClientIO.o NetworkServerIO.o -o $(TARGET) $(ROSoptions)

$(TARGETo): $(TARGETcpp) $(SRC)/$(BUILD)/Build.h $(SRC)/RoCo.h handlers_cs.h
	$(CC) -c $(TARGETcpp) $(ROSoptions)

IOBus.o: $(SRC)/IOBus.cpp $(SRC)/IOBus.h $(SRC)/MessageBus.h
	$(CC) -c $(SRC)/IOBus.cpp
MessageBus.o: $(SRC)/MessageBus.cpp $(SRC)/MessageBus.h
	$(CC) -c $(SRC)/MessageBus.cpp
NetworkBus.o: $(SRC)/NetworkBus.cpp $(SRC)/$(PRO)/Protocol.h $(SRC)/NetworkBus.h $(SRC)/IOBus.h
	$(CC) -c $(SRC)/NetworkBus.cpp
NetworkClientIO.o: $(SRC)/NetworkClientIO.cpp $(SRC)/NetworkClientIO.h $(SRC)/IODriver.h $(SRC)/NetworkIO.h
	$(CC) -c -pthread $(SRC)/NetworkClientIO.cpp
NetworkServerIO.o: $(SRC)/NetworkServerIO.cpp $(SRC)/NetworkServerIO.h $(SRC)/IODriver.h $(SRC)/NetworkIO.h
	$(CC) -c -pthread $(SRC)/NetworkServerIO.cpp

clean:
	rm -r *.o $(TARGET)
