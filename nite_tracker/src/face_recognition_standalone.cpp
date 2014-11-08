#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include "face_detector.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <nite_tracker/HumansData.h>

#include "image_geometry/pinhole_camera_model.h"

#include <unistd.h>
using namespace sensor_msgs;
using namespace message_filters;

class FaceDetectorCamera: public FaceDetector
{
	image_geometry::PinholeCameraModel model;
	
	float calculateFaceSize(cv::Rect face, float z)
	{
		int x1 = face.x;
		int x2 = face.x+face.width;
		int y = face.y+face.height/2;
		cv::Point3d raggio1 = model.projectPixelTo3dRay(cv::Point2d(x1,y));
		cv::Point3d raggio2 = model.projectPixelTo3dRay(cv::Point2d(x2,y));
		float distanza = raggio2.x*z/raggio2.z - raggio1.x*z/raggio1.z;
		return distanza;
		//cv::Point3d raggio =cam_model.projectPixelTo3dRay(cv::Point2d(u,v));
		return FaceDetector::calculateFaceSize(face,z);
	}
public:
	void fromCameraInfo(const CameraInfoConstPtr& cam_info)
	{
		model.fromCameraInfo(cam_info);
	}
	
};
class FaceRecognitionBridge
{
	FaceDetectorCamera faceDetector;
	image_transport::Publisher face_pub;
	ros::Publisher humansPublisher;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	message_filters::Subscriber<Image> image_sub;
	message_filters::Subscriber<CameraInfo> image_info_sub;
	message_filters::Subscriber<Image> depth_sub;
	message_filters::Subscriber<Image> users_sub;
	typedef sync_policies::ApproximateTime<Image, CameraInfo, Image,Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync;
	void callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info, const ImageConstPtr& depth_msg,const ImageConstPtr& users_msg);

public:
	FaceRecognitionBridge();
};

FaceRecognitionBridge::FaceRecognitionBridge():nh(ros::NodeHandle()),it(nh),
	users_sub(nh, "users",2),
	image_sub(nh, "/camera/rgb/image_raw", 2),
	image_info_sub(nh, "/camera/rgb/camera_info", 2),
	depth_sub(nh, "/camera/depth/image_raw", 2),
	sync(MySyncPolicy(5), image_sub, image_info_sub, depth_sub,users_sub)
{
	face_pub = it.advertise("faces", 1);
	humansPublisher = nh.advertise<nite_tracker::HumansData>("humans",10);
	sync.registerCallback(boost::bind(&FaceRecognitionBridge::callback,this, _1, _2,_3,_4));
}
void FaceRecognitionBridge::callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info, const ImageConstPtr& depth_msg,const ImageConstPtr& users_msg)
{
	faceDetector.fromCameraInfo(cam_info);
	ros::Time startTime= ros::Time::now();

	cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(image_msg);
	cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(depth_msg);
	cv_bridge::CvImagePtr users = cv_bridge::toCvCopy(users_msg,sensor_msgs::image_encodings::TYPE_8UC1);
	if(depth->encoding != sensor_msgs::image_encodings::TYPE_16UC1)
	
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
	//std::cerr<<std::fixed<<"step took "<<duration.toSec()<<" "<<depth->encoding<<std::endl;
}


int main(int argc, char **argv)
{
	
    nice(1);
	ros::init(argc, argv, "nite_tracker");
	FaceRecognitionBridge frb;

	while (ros::ok()) {
		ros::spin();
	}

}
