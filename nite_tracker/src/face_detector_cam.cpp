#include "opentld_face_detector.h"
#include "opencv2/video/video.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;
String window_name = "Capture - Face detection";

int main()
{
	OpentldFaceDetector faceDetector;

	VideoCapture capture;
	Mat frame;

	capture.open( -1 );
	if ( ! capture.isOpened() ) {
		printf("--(!)Error opening video capture\n");
		return -1;
	}
	namedWindow( window_name, WINDOW_AUTOSIZE );
	while ( capture.read(frame) ) {
		if( frame.empty() ) {
			printf(" --(!) No captured frame -- Break!");
			break;
		}
		faceDetector.analyzeFaces(frame);
		imshow( window_name, frame );
		int c = waitKey(10);
        if( (char)c == 27 ) { break; } // escape
	}
	return 0;
}