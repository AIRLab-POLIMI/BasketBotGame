#include "ros/ros.h"
#include "vrep_common/simRosGetObjectHandle.h"
#include "vrep_common/simRosGetObjectPose.h"
#include "vrep_common/simRosSetObjectQuaternion.h"

#include "geometry_msgs/Quaternion.h"

class BasketBotSimulator
{
	public:
	
	bool getRobotHandles();
	void debugStatus();
private:
	ros::NodeHandle n;
	int robotHandle;
	int leftJoint;
	int rightJoint;
	bool getHandle(std::string name,int& variable);
};

bool BasketBotSimulator::getHandle(std::string name,int& variable)
{
  
  ros::ServiceClient client = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
  vrep_common::simRosGetObjectHandle robot_handle;
  robot_handle.request.objectName = name;
  if(!client.call(robot_handle))
  {
	  std::cout <<"error in service call"<<std::endl;
	  return false;
  }
  if(robot_handle.response.handle < 0)
  {
	  std::cout <<"error, unable to get the handle of the vrep object "<<name<<std::endl;
	  return false;
  }
  variable = robot_handle.response.handle;
  
  std::cout << name<<" handle: " <<robot_handle.response.handle<<std::endl;
	
	return true;
	
}


bool BasketBotSimulator::getRobotHandles()
{
  if ( 
  getHandle("left_wheel_joint",leftJoint) &&
  getHandle("right_wheel_joint",rightJoint) &&
  getHandle("base_link_respondable",robotHandle)
  )
	return true;
     return false;
	
}

void BasketBotSimulator::debugStatus()
{
	/*ros::ServiceClient client = n.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");
	vrep_common::simRosSetObjectPose robot_orientation;
	robot_orientation.request.handle = -1;
	robot_orientation.request.relativeToObjectHandle = robotHandle;
	robot_orientation.request.quaternion = geometry_msgs::Quaternion();
	robot_orientation.request.quaternion.x = 0.0;
	robot_orientation.request.quaternion.y = 0.5;
	robot_orientation.request.quaternion.z = 0.0;
	robot_orientation.request.quaternion.w = 1.0;
	client.call(robot_orientation);*/
	
	
	ros::ServiceClient client = n.serviceClient<vrep_common::simRosSetObjectQuaternion>("/vrep/simRosSetObjectQuaternion");
	vrep_common::simRosSetObjectQuaternion robot_orientation;
	robot_orientation.request.handle = robotHandle;
	robot_orientation.request.relativeToObjectHandle = -1;
	robot_orientation.request.quaternion = geometry_msgs::Quaternion();
	robot_orientation.request.quaternion.x = 0.0;
	robot_orientation.request.quaternion.y = 0.0;
	robot_orientation.request.quaternion.z = 1.0;
	robot_orientation.request.quaternion.w = 1.0;
	client.call(robot_orientation);
	
	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");

  BasketBotSimulator basketBotSimulator;
  basketBotSimulator.getRobotHandles();
  
  
  
  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    basketBotSimulator.debugStatus();

    ros::spinOnce();

    loop_rate.sleep();

  }
  
  /*
  beginner_tutorials::AddTwoInts srv;
 
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
*/
  return 0;
}
