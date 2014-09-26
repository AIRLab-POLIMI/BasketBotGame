#include "NiteTracker.h"
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma GCC push_options
#pragma GCC optimize ("O0")


static const std::string OPENCV_WINDOW = "Image window";


void NiteTracker::imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
	rgb_cache.push_front(rgb_msg);
	checkCache();
	return;

	try {
		cv_bridge::CvImagePtr im = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
		last_image = *im;
		image_ready=true;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}



}

//NON usare, viene chiamato da un thread esterno a ros
void NiteTracker::onNewFrame(nite::UserTracker & userTracker)
{


	return;


	{
		//questo blocco interno serve ad evitare un double free causato da libreria buggata
		nite::UserTrackerFrameRef userTrackerFrame;
		niteRc = userTracker.readFrame(&userTrackerFrame);
		if (niteRc != nite::STATUS_OK) {
			std::cout <<"nextframe failed"<<std::endl;
			return;
		}
		depth_cache.push_front(userTrackerFrame);
	}
	checkCache();
}

NiteTracker::NiteTracker():nh(), it(nh)
{


	nite::NiTE::initialize();


	initDevice(NULL);
	image_sub = it.subscribe("/camera/rgb/image_raw",2,&NiteTracker::imageCallback,this);
	depth_sub = it.subscribe("/camera/depth_registered/image_raw",2,&NiteTracker::depthCallback,this);
	image_pub = it.advertise("/tracker", 1);
	std::cerr <<"inizialato"<<std::endl;

}


bool NiteTracker::initDevice(openni::Device *device)
{

	niteRc = userTracker.create(
	         );
	if (niteRc != nite::STATUS_OK) {
		std::cerr<<"Couldn't create user tracker\n"<<std::endl;

	} else
		std::cerr << "created"<<std::endl;
	nite::UserTrackerFrameRef userTrackerFrame;

	userTracker.addNewFrameListener(this);

	return true;
}

NiteTracker::~NiteTracker()
{
	cv::destroyWindow(OPENCV_WINDOW);
	nite::NiTE::shutdown();
}
void NiteTracker::analyzeFrame(const sensor_msgs::ImageConstPtr& rgb_msg,nite::UserTrackerFrameRef userTrackerFrame)
{
	try {
		cv_bridge::CvImagePtr im = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
		last_image = *im;
		
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat image = last_image.image;
	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	for (int i = 0; i < users.getSize(); ++i) {
		const nite::UserData& user = users[i];
		float x,y;

		userTracker.convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &x, &y);
		cv::circle(image, cv::Point(x,y), 10, cv::Scalar(255,5,5),8);

	}

	nite::UserMap um = userTrackerFrame.getUserMap();
	cv::Mat userMap(um.getHeight(),um.getWidth(),CV_16S,(void*) um.getPixels());

	userMap.convertTo(userMap,CV_8U);
	inRange(userMap, 0, 0, userMap);

	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 20,20 ));
	cv::morphologyEx( userMap, userMap, cv::MORPH_OPEN, element );
	if(!users.getSize())
		userMap.setTo(1);
	image.setTo(cv::Vec3b(0,0,0),userMap);

	image_pub.publish(last_image.toImageMsg());


}
void NiteTracker::checkCache()
{
	unsigned int rgb_offset = 1, depth_offset = 2;


	if(rgb_cache.size() > rgb_offset)
		rgb_cache.resize(rgb_offset);
	if(depth_cache.size() > depth_offset)
		depth_cache.resize(depth_offset);
	if(rgb_cache.size() < rgb_offset || depth_cache.size() < depth_offset)
		return;

	//rallentare il colore



	nite::UserTrackerFrameRef userTrackerFrame = depth_cache.back();
	sensor_msgs::ImageConstPtr rgb_msg = rgb_cache.back();

	analyzeFrame(rgb_msg,userTrackerFrame);
	depth_cache.pop_back();
	rgb_cache.pop_back();


}

void NiteTracker::depthCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
	/*if (!image_ready)
		return;*/
	
	

	nite::UserTrackerFrameRef userTrackerFrame;
	niteRc = userTracker.readFrame(&userTrackerFrame);
	if (niteRc != nite::STATUS_OK) {
		std::cout <<"nextframe failed"<<std::endl;
		return;
	}
	depth_cache.push_front(userTrackerFrame);
	checkCache();
	

}

void NiteTracker::stepOnce()
{
	nite::UserTrackerFrameRef userTrackerFrame;
	niteRc = userTracker.readFrame(&userTrackerFrame);
	if (niteRc != nite::STATUS_OK) {
		std::cout <<"nextframe failed"<<std::endl;
		return;
	}
	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	for (int i = 0; i < users.getSize(); ++i) {
		const nite::UserData& user = users[i];
		if (user.isNew()) {
			userTracker.startSkeletonTracking(user.getId());
			std::cerr <<"Found a new user."<<std::endl;
		} else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED) {
			std::cerr <<"Now tracking user " << user.getId() <<std::endl;
//					const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
//					if (head.getPositionConfidence() > .5)
//					printf("%d. (%5.2f, %5.2f, %5.2f)\n", user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z);
		}
	}
}

#pragma GCC pop_options
