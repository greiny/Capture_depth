Compile command 

g++ main.cpp -o capture_depth $(pkg-config --cflags --libs opencv)

Running command

./aero_flow_v4l2 -d /dev/video13-i 1 -u
