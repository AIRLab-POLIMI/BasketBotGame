#include <ros/ros.h>
#include <basketbot_mapping/PosPrediction.h>
#include <r2p/Velocity.h>
#include <geometry_msgs/Twist.h>
#include "basketbot_brain.h"
#include <ros/package.h>

std::string brian_config_path = ros::package::getPath("basketbot_brain") + "/config";
class RosMessenger : public MessengerInterface
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
	void desiredCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void desiredCmdVelCallbackJ(const r2p::Velocity::ConstPtr& msg);
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
	
	r2p::Velocity ve;
	
	ve.x = v;
	ve.w = rot * 10.0;
	commandsPublisherTiltone.publish(ve);
}

RosMessenger::RosMessenger():brain(this,brian_config_path)
{
	 predictionSubscriber = node.subscribe("PosPrediction", 2, &RosMessenger::predictionCallback,this);
	 commandsPublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	 commandsPublisherTiltone = node.advertise<r2p::Velocity>("/tiltone/velocity", 10);

	 suggestedCmdVel = node.subscribe("suggested_cmd_vel", 2,&RosMessenger::desiredCmdVelCallback,this);
	 suggestedCmdVelJ = node.subscribe("/tiltone/velocityD", 2,&RosMessenger::desiredCmdVelCallbackJ,this);
	 
}

	void RosMessenger::desiredCmdVelCallbackJ(const r2p::Velocity::ConstPtr& msg)
	{
			float r = -msg->w / 10.0;

		float v = msg->x;
		
		float maxV = 0.25, maxR=2.5;
		v=v>maxV?maxV:v;
		v=v<-maxV?-maxV:v;
		r=r>maxV?maxV:r;
		r=r<-maxV?-maxV:r;
		
		

	brain.setSuggestedCmdVel(v,r);
		
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
	float vx = msg->velocity.x/2.0;
	float vy = msg->velocity.y/2.0;
	
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