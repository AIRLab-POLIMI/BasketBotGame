#include "NiteTracker.h"
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <unistd.h>
#include <sstream>
using namespace cv;

UserData g_users[16];

NiteTracker::NiteTracker(ros::NodeHandle &nh):nh(nh), it(nh)
{
	connected = false;
	std::cerr<<getppid()<<std::endl;
	nite::NiTE::initialize();


	image_sub = it.subscribe("/camera/rgb/image_raw",2,&NiteTracker::imageCallback,this);
	depth_sub = it.subscribe("/camera/depth_registered/image_raw",2,&NiteTracker::depthCallback,this);
	initDevice(NULL);
	image_transport::SubscriberStatusCallback itssc = boost::bind(&NiteTracker::connectCb, this);

	image_pub = it.advertise("nite", 1,itssc,itssc);
	face_pub = it.advertise("faces", 1);
	user_pub = it.advertise("users", 1);

	humansSubscriber = nh.subscribe("humans", 1,&NiteTracker::humansCallback,this );
	std::cerr <<"inizialato"<<std::endl;
	ros::SubscriberStatusCallback rssc = boost::bind(&NiteTracker::connectCb, this);

	PointsPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL> >("COMPoints",10,rssc,rssc);
	SkeletonPointsPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL> >("SkeletonPoints",10);


}
void NiteTracker::connectCb()
{
	int ascoltatori = PointsPublisher.getNumSubscribers()+image_pub.getNumSubscribers();
	if(!connected && ascoltatori > 0) {
		userTracker.addNewFrameListener(this);
		connected=true;
	}
	if(connected && ascoltatori == 0) {
		userTracker.removeNewFrameListener(this);
		connected=false;

	}
}

void NiteTracker::humansCallback(nite_tracker::HumansData::ConstPtr humans)
{
	for(int i = 0; i<humans->humans.size(); i++) {
		ros::Duration scarto = g_users[humans->humans[i]].firstSeen - humans->stamp;
		std::cerr << "scarto: "<<scarto.toSec()<<std::endl;
		if(g_users[humans->humans[i]].firstSeen <= humans->stamp)
			g_users[humans->humans[i]].human = true;
	}
}

void NiteTracker::imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	rgb_cache.push_front(rgb_msg);
	checkCache();
	return;

}

void NiteTracker::onNewFrame(nite::UserTracker & userTracker)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	nite::UserTrackerFrameRef userTrackerFrame;

	niteRc = userTracker.readFrame(&userTrackerFrame);
	if (niteRc != nite::STATUS_OK) {
		std::cerr <<"nextframe failed"<<std::endl;
		return;
	}
	depth_cache.push_front(userTrackerFrame);

	checkCache();

	return;
}

bool NiteTracker::initDevice(openni::Device *device)
{
	niteRc = userTracker.create();
	if (niteRc != nite::STATUS_OK) {
		std::cerr<<"Couldn't create user tracker\n"<<std::endl;
		return false;
	} else
		std::cerr << "created"<<std::endl;


	return true;
}

NiteTracker::~NiteTracker()
{
	userTracker.removeNewFrameListener(this);
	std::cerr<<"rimosso listener"<<std::endl;

	userTracker.destroy();
	std::cerr<<"distrutto"<<std::endl;
	std::cerr<<"spegnimento"<<std::endl;
	nite::NiTE::shutdown();
}

void NiteTracker::updateUsers(const nite::Array<nite::UserData>& users)
{
	for (int i = 0; i < users.getSize(); ++i) {
		const nite::UserData& user = users[i];
		UserData & g_user = g_users[user.getId()];
		if (user.isLost() ) {
			ROS_INFO_STREAM("Lost user " << user.getId());
			break;
		}
		if (user.isNew()) {
			g_users[user.getId()] = UserData();
			g_users[user.getId()].firstSeen = ros::Time::now();
			ROS_INFO_STREAM("Found a new user." << user.getId());
			userTracker.startPoseDetection(user.getId(),nite::POSE_PSI);

		}
		if(!g_user.visible && user.isVisible()) {
			ROS_INFO_STREAM("User in scene." << user.getId());
			g_user.visible = true;
		}
		if(g_user.visible && !user.isVisible()) {
			ROS_INFO_STREAM("User out of scene." << user.getId());
			g_user.visible = false;
		}
		if (user.getSkeleton().getState() != nite::SKELETON_TRACKED &&
		    user.getSkeleton().getState() != nite::SKELETON_CALIBRATING) {
			if(g_user.human)
				userTracker.startSkeletonTracking(user.getId());

		}
		if(user.getPose(nite::POSE_PSI).isEntered() ) {
			ROS_INFO_STREAM("Pose Psi detected " << user.getId());
		}

		if(user.getSkeleton().getState() == nite::SKELETON_TRACKED && g_user.human ) {
			ROS_INFO_STREAM("Now tracking user " << user.getId());
			publishSkeletonPoints(user.getId(),user.getSkeleton());
		}
	}

}

std::string NiteTracker::jointToString(nite::JointType jt)
{
	switch(jt) {
	case nite::JOINT_HEAD:
		return "head";
	case nite::JOINT_NECK:
		return "neck";
	case nite::JOINT_LEFT_ELBOW:
		return "left_elbow";
	case nite::JOINT_RIGHT_ELBOW:
		return "right_elbow";
	case nite::JOINT_TORSO :
		return "torso";
	case nite::JOINT_LEFT_HAND :
		return "right_hand";
	case nite::JOINT_RIGHT_HAND :
		return "left_hand";
	case nite::JOINT_LEFT_SHOULDER 	:
		return "left_shoulder";
	case nite::JOINT_RIGHT_SHOULDER 	:
		return "right_shoulder";
	default:
		return "unknown";
	}
}
void printQuaternion(tf::Quaternion& q)
{
	std::cerr << q[0]<<"\t"<<q[1]<<"\t"<<q[2]<<"\t"<<q[3]<<std::endl;
}

void NiteTracker::publishJoint(nite::UserId id ,const nite::SkeletonJoint & joint)
{
	std::stringstream ss;
	ss << jointToString(joint.getType());
	ss << "_" << id;

	nite::Point3f Pos =joint.getPosition();
	nite::Quaternion Or = joint.getOrientation();
	tf::Transform transform;
	transform.setOrigin( nitePointToRealWorld(Pos));
	tf::Quaternion rot = niteQuaternionToRealWorld(Or);
	rot.normalize();

	if(std::isfinite(rot.w()))
		transform.setRotation(niteQuaternionToRealWorld(Or));
	else
		transform.setRotation(tf::Quaternion(0,0,0,1));

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link2", ss.str()));

}

void NiteTracker::publishSkeletonPoints(nite::UserId id,const nite::Skeleton & skeleton)
{
	nite::JointType InterestingPoints[]= {
		nite::JOINT_HEAD,nite::JOINT_NECK,nite::JOINT_LEFT_ELBOW ,nite::JOINT_RIGHT_ELBOW,
		nite::JOINT_TORSO, nite::JOINT_LEFT_HAND,nite::JOINT_RIGHT_HAND,
		nite::JOINT_LEFT_SHOULDER ,
		nite::JOINT_RIGHT_SHOULDER
	};

	pcl::PointCloud<pcl::PointXYZL>::Ptr pmsg(new pcl::PointCloud<pcl::PointXYZL>);
	pmsg->header.frame_id = "camera_link2";
	pmsg->height = 1;
	for(int i = 0; i<sizeof(InterestingPoints)/sizeof(InterestingPoints[0]); i++) {
		publishJoint(id,skeleton.getJoint(InterestingPoints[i]));
		tf::Vector3 com = nitePointToRealWorld(skeleton.getJoint(InterestingPoints[i]).getPosition());
		pcl::PointXYZL point;
		point.x = com.x();
		point.y = com.y() ;
		point.z = com.z() ;
		point.label=id;
		pmsg->points.push_back(point);
	}
	pmsg->width = pmsg->points.size();
	SkeletonPointsPublisher.publish(pmsg);

}




tf::Vector3 NiteTracker::nitePointToRealWorld(nite::Point3f p)
{
	return tf::Vector3(-p.x/1000.0,p.y/1000.0,p.z/1000.0);
}
tf::Quaternion NiteTracker::niteQuaternionToRealWorld(nite::Quaternion q)
{
	return  tf::Quaternion(q.x,-q.y,-q.z,q.w);
}


void NiteTracker::publishCOMs(const nite::Array<nite::UserData>& users)
{
	std::string frame_id = "camera_link2";
	pcl::PointCloud<pcl::PointXYZL>::Ptr pmsg(new pcl::PointCloud<pcl::PointXYZL>);
	pmsg->header.frame_id = frame_id;
	pmsg->height = 1;
	unsigned int contatore = 0;
	for (int i = 0; i < users.getSize(); ++i) {
		const nite::UserData& user = users[i];
		if(!g_users[user.getId()].human)
			break;
		contatore++;
		pcl::PointXYZL point;
		point.label = users[i].getId();
		if(!users[i].isVisible()) {
			point.x = point.y=point.z = 0;
		} else {
			tf::Vector3 com = nitePointToRealWorld(users[i].getCenterOfMass ());
			point.x = com.x();
			point.y = com.y() ;
			point.z = com.z() ;
		}
		pmsg->points.push_back(point);

	}
	pmsg->width = pmsg->points.size();
	PointsPublisher.publish(pmsg);
	std::cerr<<"Compoints: "<<contatore<<std::endl;
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

	nite::UserMap um = userTrackerFrame.getUserMap();
	openni::VideoFrameRef depthRef = userTrackerFrame.getDepthFrame();
	cv::Mat userMap(um.getHeight(),um.getWidth(),CV_16S,(void*) um.getPixels());
	cv::Mat depthMap(depthRef.getHeight(),depthRef.getWidth(),CV_16U,(void*) depthRef.getData());
	userMap.convertTo(userMap,CV_8U);
	cv::Mat userMapMask = userMap.clone();

	cv_bridge::CvImage userImage;
	userImage.image = userMap;
	userImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
	sensor_msgs::ImagePtr userImagePtr = userImage.toImageMsg();
	userImagePtr->header.stamp = rgb_msg->header.stamp;
	user_pub.publish(userImagePtr);



	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();

	updateUsers(users);

	publishCOMs(users);
	int contatore =0;
	for (int i = 0; i < users.getSize(); ++i) {
		const nite::UserData& user = users[i];
		if(!user.isVisible())
			break;
		float x,y;

		userTracker.convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &x, &y);
		char c[100];
		sprintf(c,"%d",user.getId());

		cv::inRange(userMap,user.getId(),user.getId(),userMapMask);
		if(g_users[user.getId()].human) {
			image.setTo(cv::Scalar(5,155,5),userMapMask);
			contatore++;
			putText(image, c , cv::Point(x,y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,5),5);
		} else {
			image.setTo(cv::Scalar(5,5,155),userMapMask);

			putText(image, c , cv::Point(x,y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(5,5,255),5);

		}

		if(user.getSkeleton().getState() == nite::SKELETON_TRACKED) {
			nite::Skeleton skeleton = user.getSkeleton();
			nite::Point3f headPos =skeleton.getJoint(nite::JOINT_HEAD).getPosition();
			nite::Quaternion headOr = skeleton.getJoint(nite::JOINT_HEAD ).getOrientation();
			float X,Y;
			userTracker.convertJointCoordinatesToDepth(headPos.x, headPos.y, headPos.z, &X, &Y);
			putText(image, "H" , cv::Point(X,Y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(5,5,255),5);
			std::cerr << "X: " << X << "    Y:" << Y << "    "<<std::endl;
			std::cerr <<headOr.x << " "<<headOr.y<<" "<<headOr.z<<" "<<headOr.w<<std::endl;
			std::cerr <<"scheletro"<<std::endl;
		}
	}
	image_pub.publish(last_image.toImageMsg());
	std::cerr <<"image: "<<contatore<<std::endl;
}
void NiteTracker::checkCache()
{
	int color_offset = 10;

	unsigned int rgb_offset = 1, depth_offset = 1;

	if(color_offset > 0)
		rgb_offset += color_offset;
	else
		depth_offset += -color_offset;

	if(rgb_cache.size() > rgb_offset)
		rgb_cache.resize(rgb_offset);
	if(depth_cache.size() > depth_offset)
		depth_cache.resize(depth_offset);
	if(rgb_cache.size() < rgb_offset || depth_cache.size() < depth_offset)
		return;

	//rallentare il colore

	std::list<sensor_msgs::ImageConstPtr >::iterator it= rgb_cache.begin();
	++it;

	nite::UserTrackerFrameRef userTrackerFrame = depth_cache.back();
	sensor_msgs::ImageConstPtr rgb_msg = *it;

	analyzeFrame(rgb_msg,userTrackerFrame);
	depth_cache.pop_back();
	rgb_cache.pop_back();

}

void NiteTracker::depthCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
	return;
}
