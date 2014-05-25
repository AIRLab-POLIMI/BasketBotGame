#include <ros/ros.h>
#include <basketbot_mapping/PosPrediction.h>
#include <geometry_msgs/Twist.h>
#include "basketbot_brain.h"
#include <ros/package.h>

std::string brian_config_path = ros::package::getPath("basketbot_brain") + "/config";
class RosMessenger : public MessengerInterface
{
	ros::NodeHandle node;
	ros::Subscriber predictionSubscriber;
	ros::Publisher commandsPublisher;
	ros::Subscriber suggestedCmdVel;
	BasketBotBrain brain;
	void predictionCallback(const basketbot_mapping::PosPrediction::ConstPtr& msg);
	void desiredCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	public:
	void sendCommands(float v, float rot);
	RosMessenger();
	~RosMessenger(){}
};


void RosMessenger::sendCommands(float v, float rot)
{
	geometry_msgs::Twist vel_msg;
	vel_msg.angular.z = rot;
	vel_msg.linear.x = v;
	commandsPublisher.publish(vel_msg);
}

RosMessenger::RosMessenger():brain(this,brian_config_path)
{
	 predictionSubscriber = node.subscribe("PosPrediction", 2, &RosMessenger::predictionCallback,this);
	 commandsPublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	 suggestedCmdVel = node.subscribe("suggested_cmd_vel", 2,&RosMessenger::desiredCmdVelCallback,this);
	 
}
void RosMessenger::desiredCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	float v = 5.0*msg->linear.x;
	float r = -msg->angular.z;

	brain.setSuggestedCmdVel(v,r);
	
}


void RosMessenger::predictionCallback(const basketbot_mapping::PosPrediction::ConstPtr& msg)
{
	float x = msg->position.x;
	float y = msg->position.y;
	float vx = msg->velocity.x;
	float vy = msg->velocity.y;
	
	brain.setPlayerPosUnreliability(msg->unreliability);
	
	brain.setPlayerPosPrediction(x,y,vx,vy);
	
	
	std::cout << "predictionCallback "<<x<<" " << y <<std::endl;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "basketbot_brain");
	RosMessenger messenger;
	while(ros::ok())
		ros::spin();
}