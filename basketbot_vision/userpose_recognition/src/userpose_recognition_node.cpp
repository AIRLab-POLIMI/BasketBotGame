#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <sstream>
class UserposeRecognition
{
	tf::TransformListener transformListener;
	tf::TransformBroadcaster transformBroadcaster;
	ros::Publisher debugPublisher;
	ros::NodeHandle nh;
	void readBodyTransform(unsigned int,std::string,std::string,std::string,tf::StampedTransform&);
public:
	UserposeRecognition();
	void readFrames();
	
};
void UserposeRecognition::readBodyTransform(unsigned int user,std::string bodySide,std::string srcFrame,std::string dstFrame,tf::StampedTransform& tfs)
{
	std::ostringstream srcss;
	srcss << bodySide <<"_"<<srcFrame<<"_"<<user;
	std::string source = srcss.str();
	
	std::ostringstream dstss;
	dstss << bodySide <<"_"<<dstFrame<<"_"<<user;
		std::string destination = dstss.str();
	transformListener.lookupTransform ( source,destination,ros::Time(0),tfs);
	
}
void printQuaternion(tf::Quaternion& q)
{
	std::cout << q[0]<<"\t"<<q[1]<<"\t"<<q[2]<<"\t"<<q[3]<<std::endl;
	
	
}
void UserposeRecognition::readFrames()
{
	geometry_msgs::Twist t;
	tf::StampedTransform elbow;
	
	try{
		//readBodyTransform(1,"left","shoulder","elbow",elbow);
		transformListener.lookupTransform ( "torso_1","left_shoulder_1",ros::Time(0),elbow);
	}
	catch(...)
	{
		return;
	}
	
	
	tf::Quaternion resto, q = elbow.getRotation();
	geometry_msgs::Vector3 debug;
	resto = q;
	resto[1] = resto[2] = 0;
	resto.normalize();
	q = q* resto.inverse() ;
	
	debug.x = resto.getAngle();
	
	resto = q;
	resto[1] = resto[0] = 0;
	resto.normalize();
	q = q* resto.inverse() ;
	
	debug.y = resto.getAngle();
	
	resto = q;
	resto[2] = resto[0] = 0;
	resto.normalize();
	q = q* resto.inverse() ;
	//printQuaternion(spalla1.inverse());
	
	debug.z = resto.getAngle();
	debugPublisher.publish(debug);
		std::cout << std::fixed <<debug.x<<debug.y<<debug.z<<std::endl;

	tf::Matrix3x3(tf::Quaternion(q[1],q[0],q[2],q[3])).getRPY( debug.x,debug.y,debug.z,2);
	std::cout << std::fixed <<debug.x<<debug.y<<debug.z<<std::endl;
	tf::Matrix3x3(tf::Quaternion(q[1],q[2],q[0],q[3])).getRPY( debug.x,debug.y,debug.z,2);
	std::cout << std::fixed <<debug.x<<debug.y<<debug.z<<std::endl;
	tf::Matrix3x3(tf::Quaternion(q[0],q[2],q[1],q[3])).getRPY( debug.x,debug.y,debug.z,2);
	std::cout << std::fixed <<debug.x<<debug.y<<debug.z<<std::endl;
	tf::Matrix3x3(tf::Quaternion(q[2],q[1],q[0],q[3])).getRPY( debug.x,debug.y,debug.z,2);
	std::cout << std::fixed <<debug.x<<debug.y<<debug.z<<std::endl;
	tf::Matrix3x3(tf::Quaternion(q[2],q[0],q[1],q[3])).getRPY( debug.x,debug.y,debug.z,2);
	std::cout << std::fixed <<debug.x<<debug.y<<debug.z<<std::endl;
	return;
	elbow.setRotation(q);
	
	transformBroadcaster.sendTransform(tf::StampedTransform(elbow, ros::Time::now(), "torso_1","sh"));
	return;
	
	
	/*if(fabs(q[0]) < 0.1)
		q[0] = 0;
	if(fabs(q[2]) < 0.1)
		q[2] = 0;
	q.normalize();*//*
	double roll, pitch, yaw;
	
	
	
	tf::Matrix3x3(q).getRPY( roll,pitch,yaw,0);
	if(fabs(roll) > 3.1)
			;//tf::Matrix3x3(q).getRPY( roll,pitch,yaw,1);
	
	tf::Vector3 v = q.getAxis();
	
	printQuaternion(q);
			std::cout <<roll<<" "<<pitch << " "<< yaw<<std::endl;
	std::cout <<elbow.getOrigin().length()<<std::endl;*/

}




UserposeRecognition::UserposeRecognition()
{
		debugPublisher = nh.advertise<geometry_msgs::Vector3>("debug",10);


}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "basketbot_pose_recognition");
	UserposeRecognition u;
	ros::Rate r(12);
	while (ros::ok()) {
		
		ros::spinOnce();
		u.readFrames();
		r.sleep();
	}

}
