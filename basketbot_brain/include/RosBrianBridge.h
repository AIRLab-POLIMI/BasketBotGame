#ifndef ROS_BRIAN_BRIDGE_H
#define ROS_BRIAN_BRIDGE_H

#include <ros/ros.h>
#include <player_tracker/PosPrediction.h>
#include <r2p/Velocity.h>
#include <geometry_msgs/Twist.h>
#include "basketbot_brain.h"
#include "obstacles_listener.h"
#include <userpose_recognition/UserPose.h>
#include <nav_msgs/Odometry.h>

class RosBrianBridge 
{
public:
	typedef std::pair<float,float> BiFloat;
	private:
	float last_odometry_v;
	float last_odometry_alpha;

	ros::NodeHandle node;
	ros::Subscriber predictionSubscriber;
	ros::Subscriber userPoseSubscriber;
	ros::Publisher commandsPublisher;
	ros::Publisher commandsPublisherTiltone;
	ros::Subscriber suggestedCmdVelKey;
	ros::Subscriber suggestedCmdVelJoy;
	ros::Subscriber odomSubscriber;
	
	static float maxSpeeds;
	BasketBotBrain brain;
	ObstaclesListener obstaclesListener;
	void predictionCallback(const player_tracker::PosPrediction::ConstPtr& msg);
	void desiredCmdVelKeyCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void desiredCmdVelJoyCallback(const r2p::Velocity::ConstPtr& msg);
	void userPoseCallback(const userpose_recognition::UserPoseConstPtr &);
	void odomCallback(const nav_msgs::Odometry::ConstPtr &);
	ros::Time last_joy_suggestion;
	
	
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
	float getPlayerPositionUnreliability();
	float getSuggestedLinearSpeed();
	float getSuggestedAngularSpeed();
	BiFloat getRobotSpeed();
	std::vector<float> getObstacles();
	RosBrianBridge();
	~RosBrianBridge() {}
};
#endif