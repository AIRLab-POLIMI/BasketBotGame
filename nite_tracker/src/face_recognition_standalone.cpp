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
#include <boost/thread.hpp>
#include <unistd.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
using namespace sensor_msgs;
using namespace message_filters;

class FaceDetectorCamera: public FaceDetector
{
	image_geometry::PinholeCameraModel model;

	float calculateFaceSize(cv::Rect face, float z) {
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
	void fromCameraInfo(const CameraInfoConstPtr& cam_info) {
		model.fromCameraInfo(cam_info);
	}
	std::vector<cv::Point3d> getFacesCoords()
	{
		std::vector<cv::Point3d> faces = getFacesCameraCoords();
		std::vector<cv::Point3d> result;
		for(int i = 0;i<faces.size();i++)
		{
			
			cv::Point3d raggio1 = model.projectPixelTo3dRay(cv::Point2d(faces[i].x,faces[i].y));
			raggio1.x *= faces[i].z /raggio1.z /-1000.0;
			raggio1.y *= faces[i].z /raggio1.z /-1000.0;
			raggio1.z *= faces[i].z /raggio1.z /1000.0;
			std::cerr<<"distanza" << raggio1.z<< "   x:"<<raggio1.x<<std::endl;
			result.push_back(raggio1);
		}
		return result;
	}

};
class FaceRecognitionBridge
{
	FaceDetectorCamera faceDetector;
	image_transport::Publisher face_pub;
	ros::Publisher humansPublisher;
	ros::Publisher facesPublisher;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	message_filters::Subscriber<Image> image_sub;
	message_filters::Subscriber<CameraInfo> image_info_sub;
	message_filters::Subscriber<Image> depth_sub;
	message_filters::Subscriber<Image> users_sub;


	boost::mutex processing_mutex;
	boost::mutex io_mutex;
	bool worker_is_done;
	boost::condition_variable m_condition;


	typedef sync_policies::ApproximateTime<Image, CameraInfo, Image,Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync;
	void callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info, const ImageConstPtr& depth_msg,const ImageConstPtr& users_msg);
	void callbackThread(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info, const ImageConstPtr& depth_msg,const ImageConstPtr& users_msg);
	unsigned char active;
	void connectCb();

public:
	FaceRecognitionBridge();
};

void FaceRecognitionBridge::connectCb()
{
	/*
	int ascoltatori = face_pub.getNumSubscribers()+humansPublisher.getNumSubscribers();
	unsigned char oldActive=0;
	if(humansPublisher.getNumSubscribers() > 0)
		active = 2;
	else if(face_pub.getNumSubscribers() > 0)
		active = 1;
	else
		active = 0;
	if(oldActive == active)
		return;
	if(oldActive == 1)
	{
		delete syncSimple;
		std::cerr <<"syncsymple destroyed"<<std::endl;
	}

	if(oldActive == 2)
	{
		delete sync;
		std::cerr <<"sync destroyed"<<std::endl;
	}

	if(active == 1) {
		syncSimple = new Synchronizer<MySyncPolicySimple>(MySyncPolicySimple(2), image_sub, image_info_sub, depth_sub);
		syncSimple->registerCallback(boost::bind(&FaceRecognitionBridge::callbackSimple,this, _1, _2,_3));
		std::cerr<<"syncsimple created"<<std::endl;
	}
	if(active == 2) {
		sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(2), image_sub, image_info_sub, depth_sub,users_sub);
		sync->registerCallback(boost::bind(&FaceRecognitionBridge::callback,this, _1, _2,_3,_4));
		std::cerr <<"sync created"<<std::endl;
	}



	std::cerr <<"facciascoltatori: " << ascoltatori<<"    active: "<<(int)active<<std::endl;*/
	/*
	if(!connected && ascoltatori > 0) {
		connected=true;
	}
	if(connected && ascoltatori == 0) {
		connected=false;

	}*/
}

FaceRecognitionBridge::FaceRecognitionBridge():nh(ros::NodeHandle()),it(nh),
	users_sub(nh, "users",2),
	image_sub(nh, "/camera/rgb/image_raw", 2),
	image_info_sub(nh, "/camera/rgb/camera_info", 2),
	depth_sub(nh, "/camera/depth_registered/sw_registered/image_rect_raw", 2),
	sync(MySyncPolicy(10), image_sub, image_info_sub, depth_sub,users_sub)
{
	image_transport::SubscriberStatusCallback itssc = boost::bind(&FaceRecognitionBridge::connectCb, this);
	ros::SubscriberStatusCallback rssc = boost::bind(&FaceRecognitionBridge::connectCb, this);
	facesPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL> >("FacesPoints",10);
	face_pub = it.advertise("faces", 1,itssc,itssc);
	humansPublisher = nh.advertise<nite_tracker::HumansData>("humans",10,rssc,rssc);
	sync.registerCallback(boost::bind(&FaceRecognitionBridge::callback,this, _1, _2,_3,_4));

	active = 0;
}


void FaceRecognitionBridge::callbackThread(const ImageConstPtr& image_msg, const CameraInfoConstPtr& _cam_info, const ImageConstPtr& depth_msg,const ImageConstPtr& users_msg)
{
	sensor_msgs::CameraInfo::ConstPtr cam_info(new sensor_msgs::CameraInfo(*_cam_info));
	cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(image_msg);
	cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(depth_msg);
	cv_bridge::CvImagePtr users = cv_bridge::toCvCopy(users_msg,sensor_msgs::image_encodings::TYPE_8UC1);

	{
		boost::lock_guard<boost::mutex> lock(io_mutex);
		worker_is_done = true;

	}
	m_condition.notify_all();

	//Ho copiato tutto, posso notificare.

	faceDetector.fromCameraInfo(cam_info);
	ros::Time startTime= ros::Time::now();

	if(depth->encoding != sensor_msgs::image_encodings::TYPE_16UC1)
		depth->image.convertTo(depth->image, CV_16U, 1000);
	faceDetector.analyze(image->image,depth->image);
	ros::Duration duration = ros::Time::now() - startTime;
	startTime= ros::Time::now();
	face_pub.publish(image->toImageMsg());

	std::vector<int> faces = faceDetector.extractHumans(users->image);
	std::cerr <<"Umani: ";
	for(int i = 0; i<faces.size(); i++)
		std::cerr << " "<<faces[i];
	std::cerr<<std::endl;
	nite_tracker::HumansData hd;
	hd.stamp = image_msg->header.stamp;
	hd.humans = faces;
	humansPublisher.publish(hd);
	
	pcl::PointCloud<pcl::PointXYZL>::Ptr pmsg(new pcl::PointCloud<pcl::PointXYZL>);
	pmsg->header.frame_id = "camera_link2";
	pmsg->height = 1;
	std::vector<cv::Point3d> facesC = faceDetector.getFacesCoords();
	for(int i = 0; i<facesC.size(); i++) {
		
		pcl::PointXYZL point;
		point.x = facesC[i].x;
		point.y = facesC[i].y ;
		point.z = facesC[i].z ;
		point.label=i;
		pmsg->points.push_back(point);
	}
	pmsg->width = pmsg->points.size();
	facesPublisher.publish(pmsg);
	
	
	processing_mutex.unlock();
}

void FaceRecognitionBridge::callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info, const ImageConstPtr& depth_msg,const ImageConstPtr& users_msg)
{
	if(!processing_mutex.try_lock()) {
		return;
	}
	worker_is_done=false;
	boost::thread workThread(&FaceRecognitionBridge::callbackThread,this,image_msg,cam_info,depth_msg,users_msg);
	{
		boost::unique_lock<boost::mutex> lock(io_mutex);
		while (!worker_is_done) m_condition.wait(lock);
	}

	//callbackThread(image_msg,cam_info,depth_msg,users_msg);

	//std::cerr<<std::fixed<<"step took "<<duration.toSec()<<" "<<depth->encoding<<std::endl;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "face_detector");
	FaceRecognitionBridge frb;

	while (ros::ok())
		ros::spin();


}
