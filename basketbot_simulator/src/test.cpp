/*
 * e2_depthbuffer_to_img.cpp
 * 
 * convert depthbuffer from vrep to simulate depth image and rgb of kinect. Used for simulation purpose
 *
 *  Author:  Lorenzo Ripani 
 *  Email: ripani.lorenzo@gmail.com
 *
 *  Created on: 08/apr/2014
 *
 */

#define ROS_NODE_RATE	10
#define ROS_NODE_NAME	"depth_to_img"

#include <ros/ros.h>

#include "vrep_common/VrepInfo.h"
#include "vrep_common/v_repConst.h"
#include "vrep_common/VisionSensorDepthBuff.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define KINECT_MAX_DISTANCE 15000 // mm max distance revealed by vrep vision sensor.

#include <string>

using namespace sensor_msgs;
static const char WINDOW[] = "Image Processed";

class DepthToImage
{

public:
	void depth_cb (const vrep_common::VisionSensorDepthBuffConstPtr& msg)
	{
		cv::Mat depth(480,640, CV_16UC1);
		cv::Mat depth_rotate(480,640, CV_16UC1);
		// Fill Matrix with depth data
		std::vector<float>::const_iterator it = msg->data.data.begin();
		for (int h=0; h<depth.rows; h++)
		{
			for (int w=0; w<depth.cols; w++)
			{
				//Depth value from vrep are in [0,1] range so we adapt for the kinect in this way
				depth.at<uint16_t>(h,w) =*it*KINECT_MAX_DISTANCE;
				++it;
			}
		}

		cv::flip(depth,depth_rotate,0);

		//cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
		//cv::imshow(WINDOW, depth_rotate);
		//cv::waitKey();

		cv_bridge::CvImage out_msg;

		out_msg.header.frame_id = "/kinect_visionSensor";
		out_msg.header.stamp = ros::Time::now();
		out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
		out_msg.image    = depth_rotate;

		image_pub_depth_.publish(out_msg.toImageMsg());
	}

	DepthToImage () :
		depth_buff_topic_("input"),
		image_depth_topic_("output")
	{
		sub_ = nh_.subscribe (depth_buff_topic_, 10, &DepthToImage::depth_cb, this);
		image_pub_depth_ = nh_.advertise<sensor_msgs::Image> (image_depth_topic_, 10);

		//print some info about the node
		std::string r_ct = nh_.resolveName (depth_buff_topic_);
		std::string r_it_depth = nh_.resolveName (image_depth_topic_);

		ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
		ROS_INFO_STREAM("Publishing image_depth on topic " << r_it_depth );
	}

private:
	  ros::NodeHandle nh_;
	  std::string depth_buff_topic_; //default input
	  std::string image_depth_topic_; //default output

	  ros::Subscriber sub_; //depth buffer subscriber
	  ros::Publisher image_pub_depth_; 	//image message publisher for depth
};

int main (int argc, char **argv)
{
	  ros::init (argc, argv, ROS_NODE_NAME);
	  DepthToImage depth_node; //this loads up the node

	  ROS_INFO("["ROS_NODE_NAME"]:: Node started");
	  ros::Rate r(ROS_NODE_RATE);

	  while(ros::ok())
	  {
		  ros::spinOnce();
		  r.sleep();
	  }

}
