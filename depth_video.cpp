// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

///////////////////////////////////////////////////////
// librealsense tutorial #3 - Point cloud generation //
///////////////////////////////////////////////////////

// First include the librealsense C++ header file
#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cstdio>
#include <sstream>
#include <vector>

using namespace cv;
using namespace std;

int const INPUT_WIDTH 	= 320;
int const INPUT_HEIGHT 	= 240;
int const FRAMERATE 	= 60;

char* const WINDOW_DEPTH = "Depth Image";
bool loop = true;
std::ostringstream ss;
VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),50, Size(INPUT_WIDTH,INPUT_HEIGHT*2),true);

/////////////////////////////////////////////////////////////////////////////////////////

static void onMouse( int event, int x, int y, int, void* window_name )
{
       if( event == cv::EVENT_LBUTTONDOWN )
       {
             loop = false;
       }
}

void setup_windows( )
{
       cv::namedWindow( WINDOW_DEPTH, 0 );
       cv::setMouseCallback( WINDOW_DEPTH, onMouse, WINDOW_DEPTH );
}

/////////////////////////////////////////////////////////////////////////////////////////

inline void make_depth_histogram(uint8_t rgb_image[], const uint16_t depth_image[], int width, int height)
{
    static uint32_t histogram[0x10000];
    memset(histogram, 0, sizeof(histogram));


    for(int i = 0; i < width*height; ++i) ++histogram[depth_image[i]];
    for(int i = 2; i < 0x10000; ++i) histogram[i] += histogram[i-1]; // Build a cumulative histogram for the indices in [1,0xFFFF]
    for(int i = 0; i < width*height; ++i)
    {
        if(uint16_t d = depth_image[i])
        {
            int f = histogram[d] * 255 / histogram[0xFFFF]; // 0-255 based on histogram location
            rgb_image[i*3 + 0] = f;
            rgb_image[i*3 + 1] = 0;
            rgb_image[i*3 + 2] = 255 - f;
        }
        else
        {
            rgb_image[i*3 + 0] = 0;
            rgb_image[i*3 + 1] = 5;
            rgb_image[i*3 + 2] = 20;
        }
    }
}

inline void draw_depth_histogram(const uint16_t depth_image[], const uint8_t color_image[], int width, int height)
{
    static uint8_t rgb_image[INPUT_WIDTH*INPUT_HEIGHT*3];
    static uint8_t colored_image[INPUT_WIDTH*INPUT_HEIGHT*3];
    static uint8_t fusion_image[INPUT_WIDTH*(INPUT_HEIGHT*2)*3];
    
    for (int i=0; i < INPUT_WIDTH*INPUT_HEIGHT*3 ; i++)
	{
		colored_image[i] = color_image[i];
	}
    for (int i=0; i < INPUT_WIDTH*INPUT_HEIGHT*2*3 ; i++)
	{
		if (i < INPUT_WIDTH*INPUT_HEIGHT*3)
			{fusion_image[i] = rgb_image[i];}
		else
			{fusion_image[i] = colored_image[i-INPUT_WIDTH*INPUT_HEIGHT*3];}
	}
    make_depth_histogram(rgb_image, depth_image, width, height);
    Mat rgb (height, width, CV_8UC3, rgb_image);
    Mat color (height, width, CV_8UC3, colored_image);
    Mat fusion (height*2, width, CV_8UC3, fusion_image);
    imshow(WINDOW_DEPTH, fusion);
    video.write(fusion);
    cvWaitKey( 1 );
}

///////////////////////////////////////////////////////////////////////////////////////

int main() try
{
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    // Configure depth and color to run with the device's preferred settings
    rs::device * dev = ctx.get_device(0);
    dev->enable_stream(rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE);
    dev->enable_stream(rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::bgr8, FRAMERATE);
    rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    setup_windows();
    dev->start();



    while( loop )
    {
        // Wait for new frame data
        dev->wait_for_frames();
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
	const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);
        draw_depth_histogram(depth_image, color_image, depth_intrin.width, depth_intrin.height);
        //ss << rs::stream::depth << ": " << depth_intrin.width << " x " << depth_intrin.height << " " << dev.get_stream_format(rs::stream::depth) << " (" << fps << "/" << dev.get_stream_framerate(rs::stream::depth) << ")" << ", F#: " << dev.get_frame_number(rs::stream::depth);
       // putText(frame, Point(10,10), ss.str(), CV_FONT_HERSHEY_PLAIN, CV_RGB(0,0,250));
       //putText(frame, ss.str(), Point(10,10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 1);
    }

    dev->stop();
    destroyAllWindows( );

    return EXIT_SUCCESS;
}

catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}


