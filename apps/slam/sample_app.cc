#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sophus/geometry.hpp>
#include <eigen3/Eigen/Geometry>
#include "live_slam_wrapper.h"

#include "util/settings.h"
#include "util/global_funcs.h"

#include "util/undistorter.h"
#include "io_wrapper/OpenCVImageStreamThread.h"
#include "slam_system.h"
#include "DebugOutput3DWrapper.h"

using namespace std;
using namespace lsd_slam;
char key;

int main(int argc, char** argv) {
	if (argc < 2) {
		printf(
				"Usage: sample_app <camera id>\ncamera id is 0 /dev/video0, 1 for /dev/video1 etc.\n");
		exit(1);
	}

	int cameraId = atoi(argv[1]);

	cvNamedWindow("Camera_Output_Undist", 1); //Create window

    std::string calib_fn = std::string("")
			+ "/data/out_camera_data.xml";
	CvCapture* capture = cvCaptureFromCAM(cameraId); //Capture using the camera identified by cameraId
													 // camera id is 0 for /dev/video0, 1 for /dev/video1 etc

	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
	OpenCVImageStreamThread* inputStream = new OpenCVImageStreamThread();
	inputStream->setCalibration(calib_fn);
	inputStream->setCameraCapture(capture);
	inputStream->run();

	Output3DWrapper* outputWrapper = new DebugOutput3DWrapper(
			inputStream->width(), inputStream->height());
	LiveSLAMWrapper slamNode(inputStream, outputWrapper);

	IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
	printf("wh(%d, %d)\n", frame->width, frame->height);
    cv::Mat mymat = cv::cvarrToMat(frame);
	cv::Mat tracker_display = cv::Mat::ones(640, 480, CV_8UC3);
	cv::circle(mymat, cv::Point(100, 100), 20, cv::Scalar(255, 1, 0), 5);
	cv::imshow("Camera_Output_Undist", mymat);

	slamNode.Loop();

	//Undistorter* undistorter = Undistorter::getUndistorterForFile("out_camera_data.xml");

	//while (1){ //Create infinte loop for live streaming
	//	IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
	//	TimestampedMat bufferItem;
	//	bufferItem.timestamp = Timestamp::now();
	//	
	//	cv::Mat mymat = cv::Mat(frame, true);

	//	
	//	undistorter->undistort(frame, mymat);
	//    
	//	cvShowImage("Camera_Output", frame); //Show image frames on created window
	//	cv::imshow("Camera_Output_Undist", mymat);
	//	key = cvWaitKey(10); //Capture Keyboard stroke
	//	if (char(key) == 27){
	//		break; //If you hit ESC key loop will break.
	//	}
	//}

	if (inputStream != nullptr)
		delete inputStream;
	if (outputWrapper != nullptr)
		delete outputWrapper;

	cvReleaseCapture(&capture); //Release capture.
	//cvDestroyWindow("Camera_Output"); //Destroy Window
	return 0;
}
