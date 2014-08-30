#ifndef ROS_BRIAN_BRIDGE_H
#define ROS_BRIAN_BRIDGE_H

#include <ros/ros.h>
#include <basketbot_mapping/PosPrediction.h>
#include <r2p/Velocity.h>
#include <geometry_msgs/Twist.h>
#include "basketbot_brain.h"

class RosBrianBridge 
{
	float last_odometry_v;
	float last_odometry_alpha;

	ros::NodeHandle node;
	ros::Subscriber predictionSubscriber;
	ros::Publisher commandsPublisher;
	ros::Publisher commandsPublisherTiltone;
	ros::Subscriber suggestedCmdVel;
	ros::Subscriber suggestedCmdVelJ;
	BasketBotBrain brain;
	void predictionCallback(const basketbot_mapping::PosPrediction::ConstPtr& msg);
	void desiredCmdVelKeyCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void desiredCmdVelJoyCallback(const r2p::Velocity::ConstPtr& msg);
	
	//STATE
	float currentUnreliability;
	float suggestedLinearSpeed;
	float suggestedAngularSpeed;
	//float distance;
	//float angle;
	
	float predictedX;
	float predictedY;
	float predictedVX;
	float predictedVY;
	
public:
	void setSpeed(float v, float rot);
	void spin();
	float getPlayerDistance();
	float getPlayerOrientation();
	float getPlayerVelocityY();
	float getPlayerVelocityX();
	
	
	RosBrianBridge();
	~RosBrianBridge() {}
};
#endif