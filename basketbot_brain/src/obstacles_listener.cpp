#include <ros/ros.h>

#include "obstacles_listener.h"
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
void ObstaclesListener::mapCallback(nav_msgs::OccupancyGrid::ConstPtr ptr)
{
	currentMap = ptr;
	std::cerr<<"new map"<<std::endl;
}

ObstaclesListener::ObstaclesListener()
{
	mapSubscriber =nh.subscribe("/costmap_node/costmap/costmap", 2,&ObstaclesListener::mapCallback,this);
	mapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("debugMap", 50);
}

char ObstaclesListener::getMapCost(float x,float y)
{
	std::pair <unsigned int,unsigned int> coords;
	if(!currentMap)
		return 0;
	try
	{
	coords = localToCostmapCoordinates(x,y);
	
	
	}catch(...)
	{
		return 0;
	}
	unsigned int robotXmap = coords.first;
	unsigned int robotYmap = coords.second;
	unsigned int cellnum = currentMap->info.width * (int) robotYmap + (int)robotXmap;
	char prob = currentMap->data[cellnum];
	
	nav_msgs::OccupancyGrid::Ptr newMap(new nav_msgs::OccupancyGrid(*currentMap));
	if(prob > 50)
		newMap->data[cellnum] = 0;
	else
		newMap->data[cellnum] = 100;
	mapPublisher.publish(newMap);
	
	return prob;

}

char ObstaclesListener::rayTrace(float angle, float maxDistance)
{
	char maxCost = 0;
	float distances[] = {1.0,0.6,0.8};
	
	for(int i = 0;i<sizeof(distances)/sizeof(distances[0]); i++ )
	{
		float targetX = std::cos(angle) * distances[i] * maxDistance;
		float targetY = std::sin(angle) * distances[i] * maxDistance;
		char currentCost = getMapCost(targetX,targetY);
		maxCost = std::max(maxCost,currentCost);
	}
	return maxCost;
}
std::pair <unsigned int,unsigned int> ObstaclesListener::localToCostmapCoordinates(float x,float y)
{
	tf::StampedTransform transform;
	listener.lookupTransform("/odom", "/base_footprint",ros::Time(0), transform);

	tf::Pose mapPose;
	geometry_msgs::Pose mapOrigin= currentMap->info.origin;
	tf::poseMsgToTF(mapOrigin,mapPose);

	tf::Transform mapTransform(mapPose),robotTransform(transform);
	tf::Transform localTarget(tf::Quaternion(0,0,0,1),tf::Vector3(x,y,0));

	tf::Transform target = mapTransform.inverse() * robotTransform * localTarget;
	float robotXmap = target.getOrigin().x() / currentMap->info.resolution;
	float robotYmap = target.getOrigin().y()/ currentMap->info.resolution;
	return std::pair <unsigned int,unsigned int> (robotXmap,robotYmap);

}

void ObstaclesListener::debugStep()
{
	/*if(!currentMap)
		return;
	if(!listener.canTransform("/odom", "/base_footprint",ros::Time(0)))
		return;
	char cost= rayTrace(0,1.0);



	std::cerr<< (int)cost<<std::endl;*/
}
