#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <message_filters/subscriber.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "image_geometry/pinhole_camera_model.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
class BallRecognition
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	message_filters::Subscriber<Image> rgb_image_sub;
	message_filters::Subscriber<CameraInfo> rgb_info_sub;
	message_filters::Subscriber<Image> depth_image_sub;
	message_filters::Subscriber<CameraInfo> depth_info_sub;
	image_transport::Publisher ballPublisher;
	image_geometry::PinholeCameraModel model;

	typedef sync_policies::ApproximateTime<Image, CameraInfo, Image, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync;
	void imagesCallback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info, const ImageConstPtr& depth_msg,const CameraInfoConstPtr& depth_info);
public:

	BallRecognition();
};

BallRecognition::BallRecognition():
	nh(ros::NodeHandle()),
	it(nh),
	rgb_image_sub(nh, "/camera/rgb/image_raw",2),
	rgb_info_sub(nh, "/camera/rgb/camera_info",2),
	depth_image_sub(nh, "/camera/depth/image_raw",2),
	depth_info_sub(nh, "/camera/depth/camera_info",2),
	sync(MySyncPolicy(10), rgb_image_sub, rgb_info_sub, depth_image_sub,depth_info_sub)
{
	ballPublisher = it.advertise("ball", 1);
	sync.registerCallback(boost::bind(&BallRecognition::imagesCallback,this, _1, _2,_3,_4));
}
void checkRectangle(cv::Mat image, cv::Rect & rect)
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
void BallRecognition::imagesCallback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info, const ImageConstPtr& depth_msg,const CameraInfoConstPtr& depth_info)
{
	//model.fromCameraInfo(depth_info);

	cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(depth_msg);


	depth->image.convertTo(depth->image, CV_8UC1,0.125);
	cv::Mat d = depth->image.clone();
	cv::Canny( d, d, 10, 30, 3);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( d, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	d.setTo(0);
	for( int i = 0; i< contours.size(); i++ )
	 {
	   drawContours( d, contours, i, 255, 2, 8, hierarchy, 0, Point() );
	 }


	
	vector<Vec3f> circles;

	HoughCircles( d, circles, CV_HOUGH_GRADIENT, 2, d.rows/16, 500, 150, 0, 0 );
	std::cerr<<circles.size()<<std::endl;
	//d.setTo(0);

	for( size_t i = 0; i < circles.size(); i++ ) {
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		float radiusF = std::max(1.0f,circles[i][2]);
		int radius = radiusF;


		cv::Rect inside(center.x-radius,center.y-radius,center.x + radius, center.y+radius);
		checkRectangle(d,inside);

		float distance = cv::mean(depth->image(inside),d(inside))[0];
		float size = radiusF*distance;
		char c[100];
		sprintf(c,"%.2f",size);
		circle( d, center, 3, 255, -1, 8, 0 );
putText(d, c , center, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 255,1);
		if(size > 6000 && size < 8000) {
			
			
			// circle outline
			circle( d, center, radius, 255, 3, 8, 0 );

		}
	}



	depth->image = d;
	depth->encoding = "mono8";

	ballPublisher.publish(depth->toImageMsg());


}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "ball_recognition");
	BallRecognition br;
	while(ros::ok())
		ros::spin();

	return 0;


}
