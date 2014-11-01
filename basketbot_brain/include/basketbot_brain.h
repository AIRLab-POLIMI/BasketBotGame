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
	void checkUnreliability();
	ros::Time lastUpdate;
	ros::Time latestPlayerLoss;
	ros::Time latestStateChange;
	ros::Duration elapsedTime;
	//state 
	enum State {NONE,NORMAL,FROZEN,SEARCH_LEFT, SEARCH_RIGHT} currentState;
	
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
	void setState(State, float seconds = 5.0);
	void generateObstaclesData();
	
public:
	BasketBotBrain(RosBrianBridge *,std::string);
	void freeze(float milliseconds);
	void spinOnce();
	void rotateAndSearch(float direction);
	
};

#endif