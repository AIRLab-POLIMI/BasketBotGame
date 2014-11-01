#include <ros/ros.h>

#include <ros/package.h>
#include "RosBrianBridge.h"
#include <boost/math/constants/constants.hpp>

std::string brian_config_path = ros::package::getPath("basketbot_brain") + "/config";

float RosBrianBridge::maxSpeeds = 0.25;
void RosBrianBridge::setSpeed(float v, float rot)
{
	geometry_msgs::Twist vel_msg;
	vel_msg.angular.z = rot;
	vel_msg.linear.x = v;
	commandsPublisher.publish(vel_msg);


	//limit values
	float maxV = maxSpeeds,maxR = maxSpeeds;
	v=std::min(std::max(v,-maxV),maxV);
	rot=std::min(std::max(rot,-maxR),maxR);


	r2p::Velocity ve;
	ve.x = v;
	ve.w = rot * 10.0;
	commandsPublisherTiltone.publish(ve);
}

void RosBrianBridge::userPoseCallback(const userpose_recognition::UserPoseConstPtr & msg)
{
	std::cerr << "rilevata posa: "<<msg->poseName<<std::endl;
	if(msg->poseName=="surrender")
		brain.freeze(5.0);
	
}

void RosBrianBridge::odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
	last_odometry_v = msg->twist.twist.linear.x;
	last_odometry_alpha= msg->twist.twist.angular.z;
	
}

RosBrianBridge::RosBrianBridge():brain(this,brian_config_path)
{
	predictionSubscriber = node.subscribe("PosPrediction", 2, &RosBrianBridge::predictionCallback,this);
	commandsPublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	commandsPublisherTiltone = node.advertise<r2p::Velocity>("/tiltone/velocity", 10);
	userPoseSubscriber = node.subscribe("UserPose", 2, &RosBrianBridge::userPoseCallback,this);
	suggestedCmdVelKey = node.subscribe("/suggested_cmd_vel", 2,&RosBrianBridge::desiredCmdVelKeyCallback,this);
	suggestedCmdVelJoy = node.subscribe("/tiltone/velocityD", 2,&RosBrianBridge::desiredCmdVelJoyCallback,this);
	odomSubscriber = node.subscribe("/odom", 2,&RosBrianBridge::odomCallback,this);

	suggestedLinearSpeed = suggestedAngularSpeed = 0;
	last_odometry_v = last_odometry_alpha = 0;

}

void RosBrianBridge::desiredCmdVelJoyCallback(const r2p::Velocity::ConstPtr& msg)
{
	float r = msg->w / 10.0;
	float v = msg->x;
	if(r!=suggestedAngularSpeed && v != suggestedLinearSpeed)
		last_joy_suggestion = ros::Time::now();
	suggestedLinearSpeed=v;
	suggestedAngularSpeed=r;



}
void RosBrianBridge::desiredCmdVelKeyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	float v = 5.0*msg->linear.x;
	float r = msg->angular.z;

	suggestedLinearSpeed = v;
	suggestedAngularSpeed=r;
}


void RosBrianBridge::predictionCallback(const player_tracker::PosPrediction::ConstPtr& msg)
{
	float x = msg->position.x;
	float y = msg->position.y;
	float vx = msg->velocity.x;
	float vy = msg->velocity.y;


	predictedX = x;
	predictedY = y;
	predictedVX = vx;
	predictedVY = vy;
	currentUnreliability = msg->unreliability;
	std::cout << "predictionCallback "<<x<<" " << y <<std::endl;
}

void RosBrianBridge::spin()
{
	ros::Rate r(30);
	while(ros::ok()) {
		ros::spinOnce();
		brain.spinOnce();
		obstaclesListener.debugStep();
		r.sleep();
	}
}

std::vector<float> RosBrianBridge::getObstacles()
{
	const double angleStep =  boost::math::constants::pi<double>()/4.0;
	std::vector<float> result;
	for(int i = 0; i<8; i++) {
		double currentAngle = angleStep*i;
		float cost = obstaclesListener.rayTrace(currentAngle,1.0);
		result.push_back(cost);
	}
	return result;
}

float RosBrianBridge::getPlayerDistance()
{
	return sqrt(predictedX*predictedX+predictedY*predictedY);
}
float RosBrianBridge::getPlayerOrientation()
{
	return atan2(predictedY,predictedX);
}
float RosBrianBridge::getPlayerVelocityY()
{
	return predictedVY;
}
float RosBrianBridge::getPlayerVelocityX()
{
	return predictedVX;
}

RosBrianBridge::BiFloat RosBrianBridge::getRobotSpeed()
{
	BiFloat result;
	result.first = last_odometry_v;
	result.second = last_odometry_alpha;
	return result;
	
}
float RosBrianBridge::getPlayerPositionUnreliability()
{
	return currentUnreliability;
}

float RosBrianBridge::getSuggestedLinearSpeed()
{
	if(ros::Time::now() - last_joy_suggestion > ros::Duration(1.0))
		return 0;
	return suggestedLinearSpeed;
}
float RosBrianBridge::getSuggestedAngularSpeed()
{
	if(ros::Time::now() - last_joy_suggestion > ros::Duration(1.0))
		return 0;
	return suggestedAngularSpeed;
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "basketbot_brain");
	RosBrianBridge messenger;
	messenger.spin();
}
