#include <ros/ros.h>
#include <tiltone/Tilt.h>
#include <r2p/Velocity.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <boost/math/constants/constants.hpp>

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
	ros::Subscriber encoderSubscriber;
	ros::Subscriber tiltSubscriber;
	ros::Time lastOdomTime;
	ros::Timer timer;
	void timerCallback(const ros::TimerEvent&);
	tf::TransformBroadcaster tfBroadcaster;
	ros::Publisher odomPublisher;
	ros::Publisher tiltoneOdometryPublisher;
	ros::Publisher tiltoneTiltPublisher;

	void encoderCallback(r2p::Velocity::ConstPtr );
	void tiltCallback(tiltone::Tilt::ConstPtr );
public:
	BasketbotOdom();
};

void BasketbotOdom::timerCallback(const ros::TimerEvent&)
{
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

}

void BasketbotOdom::encoderCallback(r2p::Velocity::ConstPtr velMsg)
{
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




}

void BasketbotOdom::tiltCallback(tiltone::Tilt::ConstPtr tiltMsg)
{
	tilt = tiltMsg->angle* boost::math::constants::pi<double>()/180.0;
	tiltoneTiltPublisher.publish(tiltMsg);

	return;
}

BasketbotOdom::BasketbotOdom()
{
	odomPublisher = nh.advertise<nav_msgs::Odometry>("/odom", 50);
	encoderSubscriber = nh.subscribe("/tiltone/odometry", 1,&BasketbotOdom::encoderCallback,this );
	tiltSubscriber = nh.subscribe("/tiltone/tilt", 1,&BasketbotOdom::tiltCallback,this );
	lastOdomTime = ros::Time::now();
	x = y = th = 0;
	vx = vy = vth = 0;
	tilt = 0;
	tiltoneOdometryPublisher =nh.advertise<r2p::Velocity>("/tiltone/Odometry", 2);
	tiltoneTiltPublisher = nh.advertise<tiltone::Tilt>("/tiltone/Tilt", 2);
	timer = nh.createTimer(ros::Duration(0.1), &BasketbotOdom::timerCallback,this);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "basketbot_odom");
	BasketbotOdom basketbotOdom;
	ros::spin();

}
