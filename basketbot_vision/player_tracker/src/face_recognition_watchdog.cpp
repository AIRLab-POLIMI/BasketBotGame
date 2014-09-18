#include <ros/ros.h>
#include <stdlib.h> 
#include <actionlib_msgs/GoalStatusArray.h>

ros::Time lastUpdate;
void facePosCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
	lastUpdate = ros::Time::now();
	std::cout <<"updated "<<std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "face_recognition_watchdog");
	ros::NodeHandle node;
	ros::Subscriber faceSubscriber =node.subscribe("/face_detector/status", 2, facePosCallback);
	ros::Rate r(10);
	lastUpdate = ros::Time::now();
	while(ros::ok() )
	{
		ros::spinOnce();
		
		float elapsed = (ros::Time::now() - lastUpdate).toSec(); 
		
		if(elapsed > 5.0)
		{
			std::cout << "crash" <<std::endl;
			lastUpdate = ros::Time::now();
			system("killall face_detector");
			
		}
		
		
	}
	ros::Duration(5.0).sleep();
	system("killall XnSensorServer");
}