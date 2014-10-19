#include "face_detector.h"
#include <tld/TLD.h>


class OpentldFaceDetector: public FaceDetector
{
	tld::TLD tld;
	enum {PREPARING,LEARNING,TRACKING} status;
	int counter;
	
	public:
	int analyzeFaces(cv::Mat image);
	OpentldFaceDetector();
};