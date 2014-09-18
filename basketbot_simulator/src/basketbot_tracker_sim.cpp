#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <openni_tracker/COMList.h>
#include "vrep_common/VrepInfo.h"

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
	ros::Publisher COMPublisher = nh.advertise<openni_tracker::COMList>("COMList",10);
	ros::Subscriber subInfo=nh.subscribe("/vrep/info",1,infoCallback);

	ros::Rate rate(30.0);
	while(ros::ok()) {
		rate.sleep();
		ros::spinOnce();
		try {
			transformListener.lookupTransform("base_link_respondable","bill_base_simulated", ros::Time(0), transform);

		} catch(...) {
			continue;
		}
		openni_tracker::COMList comList;
		openni_tracker::COMData data;
		data.id = 0;
		tf::Vector3 o = transform.getOrigin();
		data.z = o[0];
		data.x =  -o[1];
		data.y = -o[2];
		float angolo = atan2(transform.getOrigin().y(),transform.getOrigin().x());

		if(angolo*angolo >= 0.6)
			data.x=data.y=data.z = 0;
		comList.list.push_back(data);
		COMPublisher.publish(comList);

	}


}
