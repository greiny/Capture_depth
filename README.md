Compile command 

g++ main.cpp -o capture_depth -lrealsense $(pkg-config --cflags --libs opencv)

./capture_depth 