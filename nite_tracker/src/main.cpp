#include "NiteTracker.h"
#include <ros/ros.h>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "nite_tracker");
	ros::NodeHandle n;
	NiteTracker tracker(n);
	while (ros::ok()) {
		ros::spinOnce();
	}

}
