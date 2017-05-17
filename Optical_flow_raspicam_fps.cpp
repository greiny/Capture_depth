// to compile: g++ -o main main.cpp  -lrealsense $(pkg-config --libs --cflags opencv)
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdio.h>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <stdlib.h>
#include <unistd.h>
#include <librealsense/rs.hpp>

using namespace std;
using namespace cv;


int main ( int argc,char **argv ) {
	
    //check for FPS(Frame Per Second)
    int frames = 0;
    float time = 0, fps = 0;
    auto t0 = std::chrono::high_resolution_clock::now();

    
    //set camera params
    int const INPUT_WIDTH 	= 320;
    int const INPUT_HEIGHT 	= 240;
    int const FRAMERATE 	= 60;
    cv::Mat imgA, imgB;
    Mat imgA(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC3, (uchar *)dev->get_frame_data(rs::stream::color));  //prevent to conflicts
    rs::context ctx;
    rs::device * dev = ctx.get_device(0);
    
    //Open camera
    cout<<"Opening Camera..."<<endl;
    dev->enable_stream(rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE);
    dev->start();

    while(1) {
        //check for FPS(Frame Per Second)
        auto t1 = std::chrono::high_resolution_clock::now();
        time += std::chrono::duration<float>(t1-t0).count();
        t0 = t1;
        ++frames;
        if(time > 0.5f)
        {
            fps = frames / time;
	  cout<<"FPS(Hz)"<<fps<<endl;
            frames = 0;
            time = 0;
        }

        dev->wait_for_frames();
	    cv::Mat imgB(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC3, (uchar *)dev->get_frame_data(rs::stream::color));
		cvtColor(imgB, imgB, COLOR_RGB2GRAY);
		Size img_sz = imgB.size();
		Mat imgC(img_sz,1);
		imgB.copyTo(imgC);

		int win_size = 10;
		int maxCorners = 300;
		double qualityLevel = 0.01;
		double minDistance = 5.0;
		int blockSize = 3;
		double k = 0.04;
		
        std::vector<cv::Point2f> cornersA;
        cornersA.reserve(maxCorners);
        std::vector<cv::Point2f> cornersB;
        cornersB.reserve(maxCorners);

        if (imgA.empty() == false ) {
        		goodFeaturesToTrack( imgA,cornersA,maxCorners,qualityLevel,minDistance,cv::Mat(),blockSize,false,0.04);
        		goodFeaturesToTrack( imgB,cornersB,maxCorners,qualityLevel,minDistance,cv::Mat());

           // Call Lucas Kanade algorithm
           std::vector<unsigned char> features_found;
           features_found.reserve(maxCorners);
           std::vector<float> feature_errors;
           feature_errors.reserve(maxCorners);
           calcOpticalFlowPyrLK( imgA, imgB, cornersA, cornersB, features_found, feature_errors,Size( win_size, win_size ), 5, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );
                                                                                                                                              
           // Find the homography transformation between the frames.
           Mat H= findHomography(cornersA, cornersB, CV_RANSAC,3);
           Matx33d K = Matx33d(1,0,0,
                       0,1,0,
                       0,0,1);
           vector<Mat> R, T, N;      
           decomposeHomographyMat(H,K,R,T,N);

           Mat T1 = T[0];
           //cout<<"Homography : "<< H <<endl;
           //cout<<"Rotation : "<< R[0] <<endl;
           //cout<<"Translation : "<< T[0] <<endl;
           //cout<<"Normal vector : "<< N[0] <<endl;

           //T1.at<float>(0,0) result in strange result!!!!!!
           //cout<<"x : "<< T1.at<double>(0,0) << endl;
           //cout<<"y : "<< (T1.at<double>(1,0)) << endl;
           //cout<<"z : "<< T1.at<double>(2,0) << endl;

           // Make an image of the results
           for( int i=0; i < features_found.size(); i++ ){
        	   if( features_found[i] == 0 || feature_errors[i] > 550 )
        	   	   {
        		   	   continue;
        	   	   }
               Point p0( ceil(cornersA[i].x ), ceil(cornersA[i].y ) );
               Point p1( ceil(cornersB[i].x ), ceil(cornersB[i].y ) );
               line( imgC, p0, p1, CV_RGB(255,255,255), 2 );
	   }
           

           //namedWindow( "LKpyr_OpticalFlow",  WINDOW_AUTOSIZE );
           //imshow( "LKpyr_OpticalFlow", imgC );

           imgB.copyTo(imgA);
          }

	  else {
           // fill previous image in case prevgray.empty() == true
           imgB.copyTo(imgA);
	  }        

        waitKey(1000);
        end = clock();

        //double duration = (double)((end - begin)/CLOCKS_PER_SEC);
       // cout<<"FPS(Hz) : "<< (double)(1/duration) <<endl;   
	 }
    
    return EXIT_SUCCESS;

}