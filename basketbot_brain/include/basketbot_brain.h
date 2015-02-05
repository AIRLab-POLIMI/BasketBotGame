#ifndef BASKETBOT_BRAIN_H
#define BASKETBOT_BRAIN_H
#include <ros/ros.h>

#include "BrianWrapper.h"
#include <tf/transform_listener.h>
#include <player_tracker/PosPrediction.h>
#include <geometry_msgs/PointStamped.h>

	enum BrainState {NONE,NORMAL,FROZEN,SEARCH_LEFT, SEARCH_RIGHT,EXPLORE};


#include "strategy.h"
#include "time_throttle.h"
class RosBrianBridge;
class BasketBotBrain
{
public:

private:
	RosBrianBridge *messenger;
	BrianWrapper brian;
	BrianWrapper::DataContainer input, output;
	TimeThrottle timeThrottle;
	void runBrian();
	float applyShape(float,float);
	void checkUnreliability();
	ros::Time lastUpdate;
	ros::Time latestPlayerLoss;
	ros::Time latestStateChange;
	ros::Duration elapsedTime;
	//state
	BrainState currentState;

	float stateDuration;


	//PARAMETERS
	float orientationOffset;
	float distanceOffset;
	float distanceSensitivity;
	float rotationCommandSensitivity;

	float playerSpeedSensitivity;
	float obstaclesRadarDistance;
	float reverseRotationThreshold;
	float playerNotVisibleThreshold;
	float playerLostThreshold;
	float outputSnappiness;
	void setState(float, float seconds = 5.0);
	void generateObstaclesData();

public:
	BasketBotBrain(RosBrianBridge *,std::string);
	BrainState getState() {
		return currentState;
	}
	void freeze(float milliseconds);
	void spinOnce();
	void rotateAndSearch(float direction);
	void setParameter(std::string, float);
	float getCurrentStateElapsed();
	float getParameter(std::string name);
	bool dangerCollision();
	bool isVisible();
};

#endif
