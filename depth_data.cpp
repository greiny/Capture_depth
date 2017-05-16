// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

///////////////////////////////////////////////////////
// librealsense tutorial #3 - Point cloud generation //
///////////////////////////////////////////////////////

// First include the librealsense C++ header file
#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <cstdio>

using namespace cv;
using namespace std;

int main() try
{
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    // Configure depth and color to run with the device's preferred settings
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev->start();

    for(int i=0; i<300; i++)
    {
        // Wait for new frame data
        dev->wait_for_frames();

        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
        const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
        float scale = dev->get_depth_scale();
	Mat depth(Size(depth_intrin.width,depth_intrin.height),CV_32FC1);
	printf(" part 1\n");

        for(int dy=0; dy<depth_intrin.height; ++dy)
        {
            for(int dx=0; dx<depth_intrin.width; ++dx)
            {
                // Retrieve the 16-bit depth value and map it into a depth in meters
                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                float depth_in_meters = depth_value * scale;
		
                // Skip over pixels with a depth value of zero, which is used to indicate no data
                if(depth_value == 0) continue;

		depth.at<float>(dx+dy*depth_intrin.width) = depth_in_meters;
            }
        }
	printf(" part 2\n");
	cv::FileStorage fp("pint.xml", cv::FileStorage::WRITE);
        fp << "depth" << depth;
	fp.release();

	//double min, max;
	//minMaxIdx(depth, &min, &max);
	//printf("min: %f, max: %f\n", (float)min, (float)max);

	cv::Mat depth8u = depth;	
	depth8u.convertTo( depth8u, CV_8UC1, 255.0/30.0);
	equalizeHist(depth8u,depth8u);
	cv::Mat rgb( depth_intrin.height, depth_intrin.width, CV_8UC3);
	applyColorMap(depth8u, rgb, cv::COLORMAP_JET);
	//cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
	printf(" part 3\n");
	imshow("WINDOW_RGB", rgb);
	imwrite("pint.png",rgb);
	cvWaitKey( 1 );
    }
    
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}


