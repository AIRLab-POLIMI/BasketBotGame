#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <cv.h>
#include <highgui.h>
#include <tld/TLD.h>
using namespace cv;
using namespace std;
using namespace tld;

bool drawing_box = false;
Rect box;
Point initial_point;
bool ready = false;
// Implement mouse callback
void my_mouse_callback( int event, int x, int y, int flags, void* param )
{
	IplImage* frame = (IplImage*) param;
	Point mousePos(x,y);
	bool old_ready = drawing_box;
	switch( event ) {

	case CV_EVENT_MOUSEMOVE: {

		if( drawing_box ) {
			box = Rect(initial_point,mousePos);
		}
	}
	break;

	case CV_EVENT_LBUTTONDOWN: {
		std::cerr <<"down"<<std::endl;
		drawing_box = true;
		initial_point = mousePos;
		box = Rect(initial_point,mousePos);
	}
	break;

	case CV_EVENT_LBUTTONUP: {
		drawing_box = false;
	}
	break;

	case CV_EVENT_RBUTTONUP: {

	}
	break;

	default:
		break;
	}
	ready = !drawing_box && old_ready;
}
std::string window_name = "test";
int main()
{
	VideoCapture capture;
	Mat frame;

	capture.open( -1 );
	if ( ! capture.isOpened() ) {
		printf("--(!)Error opening video capture\n");
		return -1;
	}
	namedWindow( window_name, WINDOW_AUTOSIZE );
	setMouseCallback(window_name,my_mouse_callback,0);

	TLD tld;


	while ( capture.read(frame) ) {

		if( frame.empty() ) {
			printf(" --(!) No captured frame -- Break!");
			break;
		}

		if( drawing_box ) {
			rectangle(frame, box, Scalar(255,0,0));
		}
		cv::Mat gray;
		cv::cvtColor(frame, gray, CV_BGR2GRAY);
		std::cout << "valid "<<tld.valid <<std::endl;
		if(ready) {
			tld.detectorCascade->imgWidth = gray.cols;
			tld.detectorCascade->imgHeight = gray.rows;
			tld.detectorCascade->imgWidthStep = gray.step;
			tld.selectObject(gray,&box);
			tld.learningEnabled = true;
		}

		tld.processImage(frame);
		std::cerr<<tld.currConf<<std::endl;
		if(tld.valid) {
			cv::Rect r(*tld.currBB);
			rectangle(frame, r, Scalar(255,0,0),3);
		}
		
		std::cout <<"frame"<<std::endl;
		imshow( window_name, frame );
		int c = waitKey(10);
		if( (char)c == 27 ) {
			break;    // escape
		}
	}
	return 0;
}
