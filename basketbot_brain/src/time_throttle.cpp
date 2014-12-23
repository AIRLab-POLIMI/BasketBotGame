#include "time_throttle.h"

TimeThrottle::TimeThrottle()
{
	
}

bool TimeThrottle::checkElapsedNamed(std::string name,float requested)
{
	ros::Time now = ros::Time::now();
	if(timers.find(name) == timers.end())
		timers[name] = now;
	
	ros::Duration el(now - timers[name]);
	float elapsed = el.toSec();
	if(elapsed > requested)
	{
		timers[name] = now;
		return true;
	}
	else
	{
		return false;
	}
	
	
	
}