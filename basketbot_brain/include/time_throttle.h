#include <map>
#include <ros/ros.h>
#include <string>
class TimeThrottle
{
	std::map<std::string,ros::Time> timers;
	public:
	TimeThrottle();
	bool checkElapsedNamed(std::string,float);
	
	
};
