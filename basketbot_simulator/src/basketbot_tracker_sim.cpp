#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <openni_tracker/COMList.h>
#include "vrep_common/VrepInfo.h"
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"

void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	bool simulationRunning=(info->simulatorState.data&1)!=0;
	if(!simulationRunning)
		ros::shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "basketbot_bill_sim");
	ros::NodeHandle nh;
	tf::TransformListener transformListener;
	tf::StampedTransform transform;
	ros::Subscriber subInfo=nh.subscribe("/vrep/info",1,infoCallback);
	ros::Publisher	PointsPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZL> >("/tracker/COMPoints",10);

	ros::Rate rate(30.0);
	while(ros::ok()) {
		rate.sleep();
		ros::spinOnce();
		try {
			transformListener.lookupTransform("base_link_respondable","bill_base_simulated", ros::Time(0), transform);

		} catch(...) {
			continue;
		}


		std::string frame_id = "camera_link2";
		pcl::PointCloud<pcl::PointXYZL>::Ptr pmsg(new pcl::PointCloud<pcl::PointXYZL>);
		pmsg->header.frame_id = frame_id;
		pmsg->height = 1;
		unsigned int contatore = 0;
		tf::Vector3 o = transform.getOrigin();
		
		pcl::PointXYZL point;
		point.label = 1;
		float angolo = atan2(transform.getOrigin().y(),transform.getOrigin().x());
		if(false && angolo*angolo >= 0.6) {

			point.x = point.y=point.z = 0;
		} else {



			point.x = o.y();
			point.y = o.z() ;
			point.z = o.x() ;
		}
		pmsg->points.push_back(point);


		pmsg->width = pmsg->points.size();
		PointsPublisher.publish(pmsg);


		

	}


}
