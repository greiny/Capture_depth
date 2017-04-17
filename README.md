Compile command 

g++ main.cpp -o capture_depth $(pkg-config --cflags --libs opencv)

Running command

./capture_depth -d /dev/video13-i 1 -u
