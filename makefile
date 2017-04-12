#name of program
EXE = histogram

#path of source
SRC=$(wildcard *.cpp *.c) 

#compiler
CC = g++  

#library package
LIBS= $(--cflags -lrealsense -lopencv_core -lopencv_omgproc -lopencv_contrib)



all:
	g++ -std=c++11 main.cpp -lrealsense -lopencv_core -lopencv_imgproc -lopencv_highgui -o test
	
clean:
	rm -rf *.o