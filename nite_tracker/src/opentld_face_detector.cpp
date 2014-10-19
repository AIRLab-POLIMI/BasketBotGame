#include "opentld_face_detector.h"
#include <stdio.h>
int OpentldFaceDetector::analyzeFaces(cv::Mat image_)
{
	switch (status) {
	case PREPARING:
		loadImage(image_);
		findFaces();
		drawDetectedFaces();
		std::cout <<"PREPARING"<<std::endl;

		if(faces_front.size() == 1)
			counter++;
		else
			counter = 0;
		if(counter == 5) {
			status = LEARNING;
			cv::Mat gray;
			cv::cvtColor(image_, gray, CV_BGR2GRAY);
			tld.detectorCascade->imgWidth = gray.cols;
			tld.detectorCascade->imgHeight = gray.rows;
			tld.detectorCascade->imgWidthStep = gray.step;
			tld.selectObject(gray,&faces_front[0]);
			tld.learningEnabled = true;
			counter=0;

			printf("learning\n");
		}

		break;
	case LEARNING:
		printf("confidenza %f\n",tld.currConf);
		if(tld.currConf > 0.85)
			counter++;
		else
			counter = 0;
		std::cout <<tld.detectorCascade->numWindows<<std::endl;
		if(counter > 10 && tld.detectorCascade->numWindows > 30 ) {
			tld.learningEnabled = false;
			printf("end learning\n");
			status = TRACKING;
			counter = 0;
		}
		tld.processImage(image_);
		if(tld.valid) {

			cv::Rect r(*tld.currBB);
			rectangle(image_, r, cv::Scalar(255,0,0),3);
		}
		break;
	case TRACKING:
		tld.processImage(image_);
		if(tld.valid) {
			counter=0;
			cv::Rect r(*tld.currBB);
			rectangle(image_, r, cv::Scalar(255,0,0),3);
		} else {
			counter++;
		}
		if(counter == 10) {
			counter=0;
			status=LEARNING;
		}
		break;

	}
	return 0;
}

OpentldFaceDetector::OpentldFaceDetector()
{
	int counter = 0;
	status = PREPARING;
}
