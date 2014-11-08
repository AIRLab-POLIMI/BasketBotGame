
#include "face_detector.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/lock_guard.hpp>

using namespace cv;
FaceDetector::FaceDetector()
{
	working= true;
	working = working && loadClassifier("haarcascade_frontalface_default.xml",face_front_classifier);
	working = working && loadClassifier("haarcascade_profileface.xml",face_profile_classifier);
}

int FaceDetector::analyze(cv::Mat image_)
{
	loadImage(image_);
	findFaces();
	drawDetectedFaces();
	return 0;
}

void FaceDetector::analyze(cv::Mat image_,cv::Mat depth)
{
	loadImage(image_);
	findFaces();
	drawDetectedFaces(cv::Scalar(255,0,0));
	filterFacesByDepth(depth);
	drawDetectedFaces(cv::Scalar(0,255,0));


}

void FaceDetector::drawDetectedFaces(cv::Scalar color )
{
	//display
	for ( size_t i = 0; i < faces_profile.size(); i++ ) {
		cv::rectangle(image,faces_profile[i], color);
	}
	for ( size_t i = 0; i < faces_front.size(); i++ ) {
		cv::rectangle(image,faces_front[i], color);
	}

}
bool FaceDetector::loadClassifier(std::string name, cv::CascadeClassifier &cascade)
{
	const std::string hdir = "/usr/share/opencv/haarcascades";
	const std::string ldir = "/usr/share/opencv/lbpcascades";

	if(cascade.load(hdir + "/" + name) || cascade.load(ldir + "/" + name))
		return true;
	else
		return false;
}


void FaceDetector::shrinkRectangles(std::vector<cv::Rect> &rectangles,float ratio=0.5)
{
	float right = 0.5+0.5*ratio;
	float left = 0.5 - 0.5*ratio;

	for(size_t i = 0; i < rectangles.size(); i++) {
		cv::Rect sr(cv::Point(rectangles[i].x+left*rectangles[i].width,rectangles[i].y+left*rectangles[i].height),
		            cv::Point(rectangles[i].x+right*rectangles[i].width,rectangles[i].y+right*rectangles[i].height));
		rectangles[i] = sr;
	}
}

void FaceDetector::findFacesProfile(bool reverse)
{
	std::vector<Rect> faces_tmp;

	if(!reverse) {
		face_profile_classifier.detectMultiScale( frame_gray, faces_tmp, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
		for ( size_t i = 0; i < faces_tmp.size(); i++ )
			faces_tmp[i].x = faces_tmp[i].x + 0.1*faces_tmp[i].width;
	} else {
		//detect flipped profile face
		cv::Mat extra;

		flip(frame_gray,extra,1);
		face_profile_classifier.detectMultiScale( extra, faces_tmp, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
		for ( size_t i = 0; i < faces_tmp.size(); i++ )
			faces_tmp[i].x = faces_tmp[i].x + 0.1*faces_tmp[i].width;
		for ( size_t i = 0; i < faces_tmp.size(); i++ ) {
			faces_tmp[i].x = extra.cols - faces_tmp[i].x - faces_tmp[i].width;

		}
	}
	boost::lock_guard<boost::mutex> lock(faces_profile_mutex);
	for ( size_t i = 0; i < faces_tmp.size(); i++ ) {
		faces_profile.push_back(faces_tmp[i]);
	}
}

void FaceDetector::findFaces(bool front_only)
{
	iteration++;
	//iteration = 1;
	faces_front.clear();
	faces_profile.clear();

	bool alternate_mode = true;

	if(front_only || alternate_mode?(iteration%3 == 0):true) {
		face_front_classifier.detectMultiScale( frame_gray, faces_front, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
		shrinkRectangles(faces_front);
	}
	if(!front_only && !alternate_mode)
	{
		boost::thread face_thread(&FaceDetector::findFacesProfile,this,false); //findFacesProfile(false);
		findFacesProfile(true);
		face_thread.join();
		shrinkRectangles(faces_profile,0.4);
	}
	if(!front_only && alternate_mode) {
		if((iteration%3 == 1))
			findFacesProfile(true);
		if((iteration%3 == 2))
			findFacesProfile(false);
		shrinkRectangles(faces_profile,0.4);
		//detect profile face

	}


}
Mat FaceDetector::loadImage(cv::Mat image_)
{
	faces_profile.clear();
	image = image_;
	cvtColor( image, frame_gray, COLOR_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray );
	return frame_gray;
}

void FaceDetector::checkRectangle(cv::Mat image, cv::Rect & rect)
{
	if(rect.x< 0) {
		rect.width +=rect.x;
		rect.x = 0;
	}
	if(rect.y< 0) {
		rect.height +=rect.y;
		rect.y = 0;
	}

	if(rect.x + rect.width > image.cols) {
		rect.width = image.cols-rect.x;
	}
	if(rect.y + rect.height > image.rows)
		rect.height = image.rows-rect.y;
}

float FaceDetector::calculateFaceSize(cv::Rect face, float distance)
{
	return face.width * distance / 50.0;
}

void FaceDetector::filterFacesByDepthInternal(cv::Mat depth,std::vector<cv::Rect> & faces,float min,float max)
{
	cv::Mat mask(depth.rows, depth.cols, CV_8U, Scalar(0));
	cv::inRange(depth, 10, 10000000, mask);
	std::vector<cv::Rect> faces_tmp;
	std::vector<cv::Rect> faces_big = faces;
	shrinkRectangles(faces_big,2.0);


	for ( size_t i = 0; i < faces.size(); i++ ) {
		cv::Rect face = faces[i];
		Point center( face.x + face.width/2, face.y + face.height/2 );
		float distance = cv::mean(depth(face),mask(face))[0];
		float ratio = calculateFaceSize(face,distance);
		bool compatibleFace = ratio > min && ratio < max;
		double bgDistance;
		checkRectangle(depth,faces_big[i]);
		minMaxLoc(depth(faces_big[i]),NULL,&bgDistance,NULL,NULL,mask(faces_big[i]));
		compatibleFace = compatibleFace && bgDistance-distance>200;
		char c[100];
		sprintf(c,"%.2f %.2f",ratio,(float)bgDistance -distance);
		cv::rectangle(image,faces_big[i], cv::Scalar(0,0,255));
		if(compatibleFace)
			putText(image, c , center, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,255,0),1);
		else
			putText(image, c , center, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,255),1);
		if(compatibleFace)
			faces_tmp.push_back(face);
	}
	std::swap(faces_tmp,faces);
}


void FaceDetector::filterFacesByDepth(cv::Mat depth)
{
	filterFacesByDepthInternal(depth,faces_profile,85,105);
	filterFacesByDepthInternal(depth,faces_front,75,95);
}

std::vector<int> FaceDetector::extractHumans(cv::Mat users)
{
	std::vector<int>  humans;
	for ( size_t i = 0; i < faces_front.size(); i++ ) {
		cv::Rect face = faces_front[i];
		Point center( face.x + face.width/2, face.y + face.height/2 );
		int id = users.at<unsigned char>(center);
		if(id > 0)
			humans.push_back (id);
	}
	for ( size_t i = 0; i < faces_profile.size(); i++ ) {
		cv::Rect face = faces_profile[i];
		Point center( face.x + face.width/2, face.y + face.height/2 );
		int id = users.at<unsigned char>(center);
		if(id > 0)
			humans.push_back (id);
	}
	return humans;
}
