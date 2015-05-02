#include <ros/ros.h>

#include <ros/package.h>
#include "RosBrianBridge.h"
#include <std_msgs/Empty.h>
std::string brian_config_path = ros::package::getPath("basketbot_brain") + "/config";


void RosBrianBridge::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	goal = *msg;
	goal.header.stamp = ros::Time::now();
}


void RosBrianBridge::setSpeed(float v, float rot)
{
	geometry_msgs::Twist vel_msg;
	vel_msg.angular.z = rot;
	vel_msg.linear.x = v;
	commandsPublisher.publish(vel_msg);


	//limit values
	float maxV = maxSpeed,maxR = maxRotSpeed;
	v *= maxV;
	rot*= maxR;
	v=std::min(std::max(v,-maxV),maxV);
	rot=std::min(std::max(rot,-maxR),maxR);

	static float vecchio = 0.0;
	const float ratto = 0.1;
	vecchio = vecchio*(1.0-ratto) + ratto*rot;
	rot=vecchio;
	
	r2p::Velocity ve;
	ve.x = v;
	ve.w = rot * 10.0;
	commandsPublisherTiltone.publish(ve);
}

void RosBrianBridge::userPoseCallback(const userpose_recognition::UserPoseConstPtr & msg)
{
	ROS_DEBUG_STREAM( "rilevata posa: "<<msg->poseName);
	strategy.poseDetected(msg->poseName);

}

void RosBrianBridge::odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
	last_odometry_v = msg->twist.twist.linear.x;
	last_odometry_alpha= msg->twist.twist.angular.z;

}
void RosBrianBridge::canestroCallback(const std_msgs::Empty::ConstPtr &)
{
	strategy.canestro();
}
static void die(std::string message)
{
	std::cerr<<message<<std::endl;
	ros::shutdown();
	exit(1);
}
#define GET_PARAM(x,y)  if(!pnh.getParam(x,y)) die("missing param" x)
RosBrianBridge::RosBrianBridge():node(),pnh("~"),brain(this,brian_config_path),strategy(&brain,this),obstaclesListener()
{
	maxSpeed = 0.70;
    maxRotSpeed = 0.70;
	commandsPublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	commandsPublisherTiltone = node.advertise<r2p::Velocity>("/tiltone/Velocity", 10);
	
	predictionSubscriber = node.subscribe("PosPrediction", 2, &RosBrianBridge::predictionCallback,this);
	userPoseSubscriber = node.subscribe("UserPose", 2, &RosBrianBridge::userPoseCallback,this);
	suggestedCmdVelKey = node.subscribe("/suggested_cmd_vel", 2,&RosBrianBridge::desiredCmdVelKeyCallback,this);
	suggestedCmdVelJoy = node.subscribe("/tiltone/velocityD", 2,&RosBrianBridge::desiredCmdVelJoyCallback,this);
	odomSubscriber = node.subscribe("/odom", 2,&RosBrianBridge::odomCallback,this);
	goalSubscriber = node.subscribe("goal",2,&RosBrianBridge::goalCallback,this);
	canestroSubscriber = node.subscribe("/canestro",2,&RosBrianBridge::canestroCallback,this);
	
	suggestedLinearSpeed = suggestedAngularSpeed = 0;
	last_odometry_v = last_odometry_alpha = 0;
	currentUnreliability = 10000.0;
	GET_PARAM("max_speed",maxSpeed);
	GET_PARAM("max_rot_speed",maxRotSpeed);
	
}
#undef GET_PARAM
void RosBrianBridge::desiredCmdVelJoyCallback(const r2p::Velocity::ConstPtr& msg)
{
	float r = msg->w / 10.0;
	float v = msg->x;
	if(r!=suggestedAngularSpeed || v != suggestedLinearSpeed || suggestedAngularSpeed > 2 || suggestedLinearSpeed > 0.2)
		last_joy_suggestion = ros::Time::now();
	suggestedLinearSpeed=v;
	suggestedAngularSpeed=r;



}
void RosBrianBridge::desiredCmdVelKeyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	float v = 5.0*msg->linear.x;
	float r = msg->angular.z;
last_joy_suggestion = ros::Time::now();
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

std::vector<float> RosBrianBridge::getObstacles(float distance)
{
	return obstaclesListener.getObstaclesArray(distance);
	
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

RosBrianBridge::Goal RosBrianBridge::getGoal()
{
	Goal result;
	float age = (ros::Time::now()-goal.header.stamp).toSec();
	
	if(age > 2)
		age = -1;
	result.age = age;
	try {
		tf::StampedTransform tr;
		transformListener.lookupTransform ( "odom","base_footprint",ros::Time(0),tr);
		tf::Transform robotTransform=tr;

		tf::Transform userTransform;
		userTransform.setOrigin(tf::Vector3(goal.pose.position.x,goal.pose.position.y,0));
		userTransform.setRotation(tf::Quaternion(0,0,0,1));

		tf::Transform transform = robotTransform.inverse() * userTransform;

		result.distance =  sqrt(transform.getOrigin().x() * transform.getOrigin().x() +
		                        transform.getOrigin().y() * transform.getOrigin().y());
		result.angle = atan2(transform.getOrigin().y(),transform.getOrigin().x());

	} catch(...) {
		result.age = -1;
	}

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
