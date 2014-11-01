
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

class ObstaclesListener
{
	tf::TransformListener listener;
	ros::Subscriber mapSubscriber;
	nav_msgs::OccupancyGrid::ConstPtr currentMap;
	ros::NodeHandle nh;
	ros::Publisher mapPublisher;
	void mapCallback(nav_msgs::OccupancyGrid::ConstPtr ptr);
	std::pair <unsigned int,unsigned int> localToCostmapCoordinates(float x,float y);
	char getMapCost(float x,float y);
public:
	ObstaclesListener();
	void debugStep();
	char rayTrace(float angle, float maxDistance);


};
