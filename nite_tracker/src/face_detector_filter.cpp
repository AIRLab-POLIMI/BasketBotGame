#include <ros/ros.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <tf/transform_listener.h>

const unsigned int MAX_COUNT = 20;
const float MAX_SPEED = 0.7;
const float MAX_AGE = 2.0;
class FaceDetectorFilter
{
	struct Point {
		ros::Time lastSeen;
		float x;
		float y;
		float z;
	};
	std::map<unsigned int,Point> users;
	ros::NodeHandle nh;
	ros::Subscriber faceSubscriber;
	ros::Publisher facePublisher;
	tf::TransformListener transformListener;

	void facesCallback(const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& msg);
	unsigned int getAvailableId(std::set<unsigned int> &);
	float getDistance(Point&,Point&);
	std::set<unsigned int> deleteOld();
	int fillList(std::list<Point> & ,pcl::PointCloud<pcl::PointXYZL>::ConstPtr,const tf::Transform);
	std::pair<unsigned int,std::list<Point>::iterator> findClosest(std::list<Point> & list);
	Point & transformPoint(Point & p,tf::Transform cameraTransform,bool reverse = false);

public:
	FaceDetectorFilter();
};

FaceDetectorFilter::Point& FaceDetectorFilter::transformPoint(Point & p,tf::Transform cameraTransform,bool reverse)
{
	tf::Transform faceTransform;
	faceTransform.setOrigin(tf::Vector3(p.x,p.y,p.z));
	faceTransform.setRotation(tf::Quaternion(0,0,0,1));
	if(!reverse)
		faceTransform = cameraTransform * faceTransform;
	else
		faceTransform = cameraTransform.inverse() * faceTransform;
	p.x = faceTransform.getOrigin().x();
	p.y = faceTransform.getOrigin().y();
	p.z = faceTransform.getOrigin().z();
	return p;
}
int FaceDetectorFilter::fillList(std::list<Point> & list,pcl::PointCloud<pcl::PointXYZL>::ConstPtr msg,const tf::Transform cameraTransform)
{
	ros::Time now = ros::Time::now();


	for(pcl::PointCloud<pcl::PointXYZL>::const_iterator it= msg->points.begin(); it!= msg->points.end(); ++it) {
		Point p;
		p.x = it->x;
		p.y = it->y;
		p.z = it->z;
		p.lastSeen = now;
		transformPoint(p,cameraTransform,false);
		list.push_back(p);
	}

	return msg->points.size();
}
std::pair<unsigned int,std::list<FaceDetectorFilter::Point>::iterator> FaceDetectorFilter::findClosest(std::list<Point> & list)
{
	std::pair<unsigned int,std::list<Point>::iterator> result(0,list.end());
	float maximum = MAX_SPEED*2.0;
	for(std::map<unsigned int,Point>::iterator it = users.begin(); it != users.end(); ++it) {
		for(std::list<Point>::iterator it2 = list.begin(); it2!=list.end(); ++it2) {
			if(getDistance(it->second,*it2) < maximum) {
				result = std::pair<unsigned int,std::list<Point>::iterator>(it->first,it2);
				maximum = getDistance(it->second,*it2);
			}

		}
	}

	return result;
}
std::set<unsigned int> FaceDetectorFilter::deleteOld()
{
	std::set<unsigned int> result;
	ros::Time now = ros::Time::now();
	for(std::map<unsigned int,Point>::iterator it = users.begin(); it != users.end();) {
		if((now - it->second.lastSeen).toSec() > MAX_AGE) {
			std::cerr<<"deleted user: "<<it->first<<std::endl;
			result.insert(it->first);
			users.erase(it++);

		} else
			++it;
	}
	return result;
}
unsigned int FaceDetectorFilter::getAvailableId(std::set<unsigned int> & usedUsers)
{
	for(unsigned int i = 1; i <= MAX_COUNT; i++)
		if(users.find(i) == users.end() && usedUsers.find(i) == usedUsers.end())
			return i;
	return 0;
}
float FaceDetectorFilter::getDistance(FaceDetectorFilter::Point& a,FaceDetectorFilter::Point& b)
{
	float euclideanDistance = sqrt((b.x-a.x)* (b.x-a.x) + (b.y-a.y)*(b.y-a.y));
	float elapsed = fabs((b.lastSeen - a.lastSeen).toSec());
	return  euclideanDistance/elapsed;
}
void FaceDetectorFilter::facesCallback(const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& msg)
{
	tf::Transform cameraTransform;
	try {
		tf::StampedTransform tr;
		transformListener.lookupTransform ( "odom","camera_link2",ros::Time(0),tr);
		cameraTransform=tr;
	} catch(...) {
		return;
	}
	std::set<unsigned int> usedUsers = deleteOld();
	std::list<Point> incomingUsers;
	fillList(incomingUsers, msg,cameraTransform);

	while(1) {
		std::pair<unsigned int,std::list<Point>::iterator> match = findClosest(incomingUsers);
		if(match.first == 0)
			break;
		float distance = getDistance(users[match.first],*match.second);
		if(distance> MAX_SPEED)
			break;

		if(usedUsers.find(match.first) == usedUsers.end()) {
			users[match.first] = *match.second;
			usedUsers.insert(match.first);
			std::cerr<<"user updated: "<<match.first<<", distance:" <<distance<<std::endl;
		} else {
			std::cerr<<"user ignored: "<<match.first<<", distance:" <<distance<<std::endl;

		}
		incomingUsers.erase(match.second);
	}

	for(std::list<Point>::iterator it = incomingUsers.begin(); it!=incomingUsers.end(); ++it) {
		unsigned int newId = getAvailableId(usedUsers);
		users[newId] = *it;
		std::cerr<<"added user: "<<newId<<std::endl;

	}

	pcl::PointCloud<pcl::PointXYZL>::Ptr pmsg(new pcl::PointCloud<pcl::PointXYZL>);
	pmsg->header.frame_id = "camera_link2";
	pmsg->height = 1;
	for(std::map<unsigned int,Point>::iterator it = users.begin(); it != users.end(); ++it) {
		pcl::PointXYZL point;
		Point p(it->second);
		transformPoint(p,cameraTransform,true);
		
		point.label = it->first;
		point.x=p.x;
		point.y=p.y;
		point.z = p.z;
		pmsg->points.push_back(point);
	}
	pmsg->width = pmsg->points.size();
	facePublisher.publish(pmsg);

}
FaceDetectorFilter::FaceDetectorFilter()
{
	faceSubscriber = nh.subscribe("/tracker/FacesPoints", 2, &FaceDetectorFilter::facesCallback,this);
	facePublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL> >("/tracker/FacesFiltered",10);

}
int main(int argc, char **argv)
{

	ros::init(argc, argv, "face_detector_filter");
	FaceDetectorFilter fdf;

	while (ros::ok())
		ros::spin();


}
