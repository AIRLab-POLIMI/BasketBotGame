#include <ros/ros.h>
#include <tiltone/Tilt.h>
#include <r2p/Velocity.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <boost/math/constants/constants.hpp>
#include <geometry_msgs/TwistStamped.h>

bool inverti_tilt = true;
double moltiplicatore_rotazione = 1 + (1.0/7.0 + 1.0/63.0);
class BasketbotOdom
{
	double x;
	double y;
	double th;

	double vx;
	double vy;
	double vth;

	double tilt;

	ros::NodeHandle nh;
	ros::NodeHandle pnh;
	ros::Subscriber encoderSubscriber;
	ros::Subscriber tiltSubscriber;
	ros::Subscriber velocitySubscriber;
	ros::Subscriber simOdomSubscriber;
	ros::Time lastOdomTime;
	ros::Timer timer;
	ros::Time lastVelocityMessage;
	void timerCallback(const ros::TimerEvent&);
	tf::TransformBroadcaster tfBroadcaster;
	ros::Publisher odomPublisher;
	ros::Publisher tiltoneOdometryPublisher;
	ros::Publisher tiltoneTiltPublisher;
	ros::Publisher tiltoneVelocityPublisher;

	void encoderCallback(r2p::Velocity::ConstPtr );
	void tiltCallback(tiltone::Tilt::ConstPtr );
	void velocityCallback(r2p::Velocity::ConstPtr velMsg);
	void simEncoderCallback(geometry_msgs::TwistStamped::ConstPtr sim_vel);
	bool odometria_funzionante;
	void sottoscriviNodiCritici();
public:
	BasketbotOdom();
};

void BasketbotOdom::sottoscriviNodiCritici()
{
	encoderSubscriber = ros::Subscriber();
		encoderSubscriber = nh.subscribe("/tiltone/odometry", 1,&BasketbotOdom::encoderCallback,this );
		tiltoneVelocityPublisher = ros::Publisher();
		tiltoneVelocityPublisher =nh.advertise<r2p::Velocity>("/tiltone/velocity", 2);
		tiltSubscriber =ros::Subscriber();
		tiltSubscriber = nh.subscribe("/tiltone/tilt", 1,&BasketbotOdom::tiltCallback,this );
	
}
void BasketbotOdom::timerCallback(const ros::TimerEvent& te)
{
	double elapsed = (te.last_real - lastOdomTime).toSec();
	if(elapsed > 5.0) {
		ROS_WARN("Error, no odom received");
		ROS_ERROR_STREAM("no odom");
		sottoscriviNodiCritici();
		lastOdomTime = ros::Time::now();
		odometria_funzionante = false;
	}
	{
		tf::Quaternion tfq(tf::Vector3(0,1,0),-tilt);
		geometry_msgs::Quaternion odom_quat;
		tf::quaternionTFToMsg(tfq,odom_quat);

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "base_stabilized";
		odom_trans.child_frame_id = "base_link";
		odom_trans.transform.translation.x = 0;
		odom_trans.transform.translation.y = 0;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		tfBroadcaster.sendTransform(odom_trans);

	}
	{
		tf::Quaternion tfq(tf::Vector3(0,1,0),tilt);
		geometry_msgs::Quaternion odom_quat;
		tf::quaternionTFToMsg(tfq,odom_quat);

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "camera_link";
		odom_trans.child_frame_id = "laserscan_stabilized";
		odom_trans.transform.translation.x = 0;
		odom_trans.transform.translation.y = 0;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		tfBroadcaster.sendTransform(odom_trans);

	}
	{
		tf::Quaternion tfq(tf::Vector3(0,0,1),th);
		geometry_msgs::Quaternion odom_quat;
		tf::quaternionTFToMsg(tfq,odom_quat);

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		tfBroadcaster.sendTransform(odom_trans);

	}
	{
		tf::Quaternion tfq(tf::Vector3(0,0,1),th);
		geometry_msgs::Quaternion odom_quat;
		tf::quaternionTFToMsg(tfq,odom_quat);
		nav_msgs::Odometry odom;
		odom.header.stamp = lastOdomTime;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odomPublisher.publish(odom);
	}
	if((te.last_real - lastVelocityMessage).toSec() > 5.0) {
		lastVelocityMessage = ros::Time::now();
		r2p::Velocity::Ptr vel(new r2p::Velocity);
		vel->x = 0;
		vel->y = 0;
		vel->w = 0;
		if(odometria_funzionante)
		tiltoneVelocityPublisher.publish(vel);
	}
}

void BasketbotOdom::velocityCallback(r2p::Velocity::ConstPtr velMsg)
{
	if(odometria_funzionante)
	tiltoneVelocityPublisher.publish(velMsg);
	lastVelocityMessage = ros::Time::now();
}
void BasketbotOdom::simEncoderCallback(geometry_msgs::TwistStamped::ConstPtr sim_vel)
{
	r2p::Velocity::Ptr velMsg(new r2p::Velocity() );
	velMsg->x = sim_vel->twist.linear.x;
	velMsg->w = sim_vel->twist.angular.z;
	//encoderCallback(velMsg);
}


void BasketbotOdom::encoderCallback(r2p::Velocity::ConstPtr vv)
{
	r2p::Velocity::Ptr velMsg (new r2p::Velocity(*vv));
	velMsg->w *= moltiplicatore_rotazione;
	
	
	tiltoneOdometryPublisher.publish(velMsg);
	ros::Time now = ros::Time::now();
	ros::Duration elapsedD = now - lastOdomTime;
	lastOdomTime = now;
	double elapsed = elapsedD.toSec();

	double deltaxMedio = (vx + velMsg->x)*elapsed/2.0;
	double deltathMedio = (vth + velMsg->w)*elapsed/2.0;
	double thMedio = th+deltathMedio/2.0;
	vx = velMsg->x;
	vth = velMsg->w;
	vy = 0;


	th = th + deltathMedio;
	x = x + deltaxMedio*std::cos(thMedio);
	y = y + deltaxMedio*std::sin(thMedio);
	odometria_funzionante=true;



}

void BasketbotOdom::tiltCallback(tiltone::Tilt::ConstPtr tt)
{
	
	tiltone::Tilt::Ptr tiltMsg(new tiltone::Tilt(*tt));
	if(inverti_tilt)
		tiltMsg->angle = tiltMsg->angle * -1.0;
	
	tilt = tiltMsg->angle* boost::math::constants::pi<double>()/180.0;
	
	tiltoneTiltPublisher.publish(tiltMsg);

	return;
}
#define GET_PARAM(x, y)     \
	if(!pnh.getParam(x, y)) \
		exit(1)
BasketbotOdom::BasketbotOdom():pnh("~")
{
	odomPublisher = nh.advertise<nav_msgs::Odometry>("/odom", 50);
	lastOdomTime = ros::Time::now();
	
	lastOdomTime = ros::Time::now();
	velocitySubscriber = nh.subscribe("/tiltone/Velocity", 1,&BasketbotOdom::velocityCallback,this );
	simOdomSubscriber = nh.subscribe("/e2/twist",1,&BasketbotOdom::simEncoderCallback,this);
	x = y = th = 0;
	vx = vy = vth = 0;
	tilt = 0;
	tiltoneOdometryPublisher =nh.advertise<r2p::Velocity>("/tiltone/Odometry", 2);
	sottoscriviNodiCritici();
	tiltoneTiltPublisher = nh.advertise<tiltone::Tilt>("/tiltone/Tilt", 2);
	timer = nh.createTimer(ros::Duration(1.0/30.0), &BasketbotOdom::timerCallback,this);
	GET_PARAM("inverti_tilt",inverti_tilt);
	GET_PARAM("moltiplicatore_rotazione",moltiplicatore_rotazione);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "basketbot_odom");
	BasketbotOdom basketbotOdom;
	ros::spin();

}
