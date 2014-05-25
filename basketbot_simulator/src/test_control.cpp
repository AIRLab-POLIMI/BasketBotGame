#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

 

 
  tf::TransformListener listener;
  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
	   listener.waitForTransform ("base_link_respondable","Bill_base",ros::Time::now(),ros::Duration(3.0));
       listener.lookupTransform("base_link_respondable","Bill_base", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
	float angolo = atan2(transform.getOrigin().y(),transform.getOrigin().x());
    float distanza = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    std::cout <<"distanza: "<<distanza<<"   angolo: "<<angolo<<std::endl;
	geometry_msgs::Twist vel_msg;
	vel_msg.angular.z = angolo*5;
	vel_msg.linear.x = distanza-1;
	turtle_vel.publish(vel_msg);
    rate.sleep();
  }
  return 0;
};
