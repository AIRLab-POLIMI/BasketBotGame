#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include "face_detector.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <nite_tracker/HumansData.h>
using namespace sensor_msgs;
using namespace message_filters;


class FaceRecognitionBridge
{
	FaceDetector faceDetector;
	image_transport::Publisher face_pub;
	ros::Publisher humansPublisher;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	message_filters::Subscriber<Image> image_sub;
	message_filters::Subscriber<Image> depth_sub;
	message_filters::Subscriber<Image> users_sub;
	typedef sync_policies::ApproximateTime<Image, Image,Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync;
	void callback(const ImageConstPtr& image_msg, const ImageConstPtr& depth_msg,const ImageConstPtr& users_msg);

public:
	FaceRecognitionBridge();
};

FaceRecognitionBridge::FaceRecognitionBridge():nh(ros::NodeHandle()),it(nh),
users_sub(nh, "users",2),
image_sub(nh, "/camera/rgb/image_raw", 2),
depth_sub(nh, "/camera/depth_registered/hw_registered/image_rect_raw", 2),
sync(MySyncPolicy(5), image_sub, depth_sub,users_sub)
{
	face_pub = it.advertise("faces", 1);
	humansPublisher = nh.advertise<nite_tracker::HumansData>("humans",10);
	sync.registerCallback(boost::bind(&FaceRecognitionBridge::callback,this, _1, _2,_3));
}
void FaceRecognitionBridge::callback(const ImageConstPtr& image_msg, const ImageConstPtr& depth_msg,const ImageConstPtr& users_msg)
{

	ros::Time startTime= ros::Time::now();
	
	cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(image_msg);
	cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(depth_msg);
	cv_bridge::CvImagePtr users = cv_bridge::toCvCopy(users_msg,sensor_msgs::image_encodings::TYPE_8UC1);
	depth->image.convertTo(depth->image, CV_16U, 1000);
	faceDetector.analyze(image->image,depth->image);
	std::vector<int> faces = faceDetector.extractHumans(users->image);
	std::cerr <<"Umani: ";
	for(int i = 0; i<faces.size(); i++)
		std::cerr << " "<<faces[i];
	std::cerr<<std::endl;
	nite_tracker::HumansData hd;
	hd.stamp = image_msg->header.stamp;
	hd.humans = faces;
	humansPublisher.publish(hd);
	ros::Duration duration = ros::Time::now() - startTime;
	startTime= ros::Time::now();
	face_pub.publish(image->toImageMsg());
	std::cerr<<std::fixed<<"step took "<<duration.toSec()<<" "<<depth->encoding<<std::endl;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "nite_tracker");
	FaceRecognitionBridge frb;




	
	
	
	while (ros::ok()) {
		ros::spin();
	}

}
