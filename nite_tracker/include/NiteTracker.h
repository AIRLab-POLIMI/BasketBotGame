#include <NiTE.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <list>
#include <cv_bridge/cv_bridge.h>
#include "face_detector.h"
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <nite_tracker/HumansData.h>

struct UserData {
	bool visible;
	bool human;
	UserData():visible(false),human(false) {};
	ros::Time firstSeen;
};

class NiteTracker : public nite::UserTracker::NewFrameListener
{
	boost::mutex m_mutex;
	boost::mutex face_mutex;
	boost::thread face_thread;
	cv_bridge::CvImage last_image;
	bool image_ready;
	ros::NodeHandle &nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	image_transport::Publisher image_pub;
	image_transport::Publisher face_pub;
	image_transport::Publisher user_pub;
	ros::Subscriber humansSubscriber;
	ros::Publisher PointsPublisher;
	ros::Publisher SkeletonPointsPublisher;
	tf::TransformBroadcaster br;


	nite::UserTracker userTracker;
	nite::Status niteRc;
	std::list<sensor_msgs::ImageConstPtr> rgb_cache;
	std::list<nite::UserTrackerFrameRef > depth_cache;


	void humansCallback(nite_tracker::HumansData::ConstPtr);
	void checkCache();
	void analyzeFrame(const sensor_msgs::ImageConstPtr& orig_msg,nite::UserTrackerFrameRef);
	void updateUsers(const nite::Array<nite::UserData>&);
	bool initDevice(openni::Device *device);
	void onNewFrame(nite::UserTracker &);
	void connectCb();
	bool connected;
	void imageCallback(const sensor_msgs::ImageConstPtr& orig_msg	);
	void publishCOMs(const nite::Array<nite::UserData>&);
	void publishSkeletonPoints(nite::UserId,const nite::Skeleton &);
	void publishJoint(nite::UserId id ,const nite::SkeletonJoint & joint);
	static std::string jointToString(nite::JointType jt);
	static tf::Vector3 nitePointToRealWorld(nite::Point3f);
	static tf::Quaternion niteQuaternionToRealWorld(nite::Quaternion);
	void publishUsersView(cv::Mat ,const nite::Array<nite::UserData>& users);
public:


	NiteTracker(ros::NodeHandle &);
	~NiteTracker();

};
