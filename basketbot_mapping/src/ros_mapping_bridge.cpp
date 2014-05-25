#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <basketbot_mapping/PosPrediction.h>
#include "playerEKF.h"

class PlayerPredictor
{
	playerEKF playerFilter;
	ros::NodeHandle node;
	ros::Subscriber odometrySubscriber;
	ros::Publisher predictionPublisher;
	tf::TransformListener transformListener;
	tf::TransformBroadcaster transformBroadcaster;
	
	void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void publishPrediction();
	public:
	PlayerPredictor();
	void spin();
};

void PlayerPredictor::publishPrediction()
{
	std::vector<float> res = playerFilter.getStatus(); 
	tf::Transform tr;
	tr.setOrigin(tf::Vector3(res[0],res[1],0.0));
	transformBroadcaster.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "base_link_respondable", "bill_filtered"));
	tr.setOrigin(tf::Vector3(res[2]*2.0,res[3]*2.0,0.0));
	transformBroadcaster.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "bill_filtered", "bill_filtered_vel"));
	
	basketbot_mapping::PosPrediction pred;
	pred.position.x = res[0];
	pred.position.y = res[1];
	pred.velocity.x = res[2];
	pred.velocity.y = res[3];
	pred.unreliability = sqrt(res[4] + res[5]);
	

	predictionPublisher.publish(pred);
}

void PlayerPredictor::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	float v = msg->twist.twist.linear.x;
	float a = msg->twist.twist.angular.z;
	
	playerFilter.updateOdometry(v*0.27,-a*0.28,ros::Time::now().toSec());
	
	publishPrediction();
}

PlayerPredictor::PlayerPredictor()
{
	odometrySubscriber = node.subscribe("/odom", 2, &PlayerPredictor::odometryCallback,this);
	predictionPublisher = node.advertise<basketbot_mapping::PosPrediction>("PosPrediction",10);
	
}

void PlayerPredictor::spin()
{
	ros::Rate rate(20.0);
	while (node.ok()) {
		tf::StampedTransform transform;
		try {
			transformListener.waitForTransform ("base_link_respondable","Bill_base",ros::Time::now(),ros::Duration(3.0));
			transformListener.lookupTransform("base_link_respondable","Bill_base", ros::Time(0), transform);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}
		float angolo = atan2(transform.getOrigin().y(),transform.getOrigin().x());
		float distanza = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));

		if(angolo*angolo < 0.4)
		playerFilter.updatePlayerPos(distanza,angolo,ros::Time::now().toSec());
		publishPrediction();
		std::vector<float> res = playerFilter.getStatus();
		std::cout <<"distanza: "<<distanza<<"   angolo: "<<angolo<<std::endl;
		ros::spinOnce();
		rate.sleep();
	}
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "basketbot_kfilter");
	PlayerPredictor playerPredictor;
	playerPredictor.spin();
}
