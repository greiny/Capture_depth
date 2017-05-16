#name of program
EXE = histogram

#path of source
SRC=$(wildcard *.cpp *.c) 

#compiler
CC = g++  

#library package
LIBS= $(--cflags -lrealsense -lopencv_core -lopencv_omgproc -lopencv_contrib)



all:
	g++ -std=c++11 cpp-tutorial-1-depth.cpp -lrealsense $(pkg-config --cflags --libs opencv) -o test
	
clean:
	rm -rf *.o