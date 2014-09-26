#include <NiTE.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <list>
#include <cv_bridge/cv_bridge.h>


class NiteTracker : public nite::UserTracker::NewFrameListener
{
	cv_bridge::CvImage last_image;
	bool image_ready;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	image_transport::Subscriber depth_sub;
	image_transport::Publisher image_pub;
	
	nite::UserTracker userTracker;
	nite::Status niteRc;
	std::list<sensor_msgs::ImageConstPtr> rgb_cache;
	std::list<nite::UserTrackerFrameRef > depth_cache;

	void checkCache();
	void analyzeFrame(const sensor_msgs::ImageConstPtr& orig_msg,nite::UserTrackerFrameRef);

public:
	void imageCallback(const sensor_msgs::ImageConstPtr& orig_msg	);
		void depthCallback(const sensor_msgs::ImageConstPtr& orig_msg	);

	void onNewFrame(nite::UserTracker &);
	NiteTracker();
	~NiteTracker();
	bool initDevice(openni::Device *device);
	void stepOnce();

};
