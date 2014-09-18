#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <player_tracker/PosPrediction.h>
#include "playerEKF.h"

class PlayerPredictor
{
	float last_odometry_v,last_odometry_alpha;
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
	tr.setRotation( tf::Quaternion(0, 0, 0) );
	tr.setOrigin(tf::Vector3(res[0],res[1],0.0));
	transformBroadcaster.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "base_link_respondable", "bill_filtered"));
	tr.setOrigin(tf::Vector3(res[2]*2.0,res[3]*2.0,0.0));
	transformBroadcaster.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "bill_filtered", "bill_filtered_vel"));
	
	player_tracker::PosPrediction pred;
	pred.position.x = res[0];
	pred.position.y = res[1];
	pred.velocity.x = res[2];
	pred.velocity.y = res[3];
	pred.unreliability = sqrt(res[4] + res[5]);

	std::cout <<"unrel: "<<pred.unreliability<<" "<<res[4]<<" "<<res[5]<<std::endl;
	predictionPublisher.publish(pred);
}

void PlayerPredictor::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	float v = msg->twist.twist.linear.x;
	float a = msg->twist.twist.angular.z;
	last_odometry_v = v;
	last_odometry_alpha = a;

}

PlayerPredictor::PlayerPredictor() : node(ros::NodeHandle()), transformListener(node,ros::Duration(0.1))
{
	odometrySubscriber = node.subscribe("/odom", 2, &PlayerPredictor::odometryCallback,this);
	predictionPublisher = node.advertise<player_tracker::PosPrediction>("PosPrediction",10);
	//transformListener = tf::TransformListener(node,ros::Duration(0.1));node(ros::NodeHandle()),
	last_odometry_v = 0;
	last_odometry_alpha = 0;
}

void PlayerPredictor::spin()
{
	ros::Rate rate(20.0);
	while (node.ok()) {
		playerFilter.updateOdometry(last_odometry_v,last_odometry_alpha);

		tf::StampedTransform transform;
		try {


			//transformListener.waitForTransform ("base_link_respondable","Bill_base",ros::Time(0),ros::Duration(3.0));
			transformListener.lookupTransform("base_link_respondable","Bill_base", ros::Time(0), transform);
			//transformListener.clear();
			std::cout <<ros::Time::now() - transform.stamp_ << std::endl;
			if(ros::Time::now() - transform.stamp_ <ros::Duration(0.5)) {


				float angolo = atan2(transform.getOrigin().y(),transform.getOrigin().x());
				float distanza = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));

					playerFilter.updatePlayerPos(distanza,angolo);
				std::cout <<"distanza: "<<distanza<<"   angolo: "<<angolo<<std::endl;

			}
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}

		publishPrediction();
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
