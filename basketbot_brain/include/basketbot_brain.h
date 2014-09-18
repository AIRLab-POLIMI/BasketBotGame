#ifndef BASKETBOT_BRAIN_H
#define BASKETBOT_BRAIN_H
#include <ros/ros.h>

#include "BrianWrapper.h"
class RosBrianBridge;


class BasketBotBrain
{
	RosBrianBridge *messenger;
	BrianWrapper brian;
	BrianWrapper::DataContainer input, output;
	void runBrian();
	float applyShape(float,float);
	
	ros::Time lastUpdate;
	ros::Duration elapsedTime;
	//state 
	enum State {NORMAL=0,FROZEN,SEARCH_LEFT, SEARCH_RIGHT} currentState;
	
	float stateDuration;
	
	
	//PARAMETERS
	float distanceOffset;
	float distanceSensitivity;
	float rotationCommandSensitivity;

	float playerSpeedSensitivity;
	float reverseRotationThreshold;
	float playerNotVisibleThreshold;
	float playerLostThreshold;
	float outputSnappiness;
	
	
public:
	BasketBotBrain(RosBrianBridge *,std::string);
	void freeze(float milliseconds);
	void spinOnce();
	void rotateAndSearch(float direction);
	
};

#endif