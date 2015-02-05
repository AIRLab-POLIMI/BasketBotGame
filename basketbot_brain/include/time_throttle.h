#include <map>
#include <ros/ros.h>
#include <string>
#ifndef TIME_THROTTLE_H
#define TIME_THROTTLE_H
class TimeThrottle
{
	std::map<std::string,ros::Time> timers;
	public:
	TimeThrottle();
	bool checkElapsedNamed(std::string,float);
	
	
};

#endif