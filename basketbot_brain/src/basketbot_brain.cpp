#include "basketbot_brain.h"
#include <iostream>
#include <math.h>
#include "RosBrianBridge.h"

const unsigned int rate = 30;
bool showBrianDebug = true;
bool autoMode = true;
float freezeTime = 1.5;
bool BasketBotBrain::isVisible()
{
	return messenger->getPlayerPositionUnreliability()<playerNotVisibleThreshold;
}
void BasketBotBrain::checkUnreliability()
{
	float newUnreliability = messenger->getPlayerPositionUnreliability();
	float oldUnreliability = input["Unreliability"];

	if(newUnreliability < playerNotVisibleThreshold and oldUnreliability > 1.0) {
		input["Unreliability"] = 0.5;
	} else if(newUnreliability > playerNotVisibleThreshold and oldUnreliability < 1.0) {
		input["Unreliability"] = 1.5;
		latestPlayerLoss = ros::Time::now();
		if(currentState == NORMAL)
			;//rotateAndSearch(-input["PlayerVelocityY"]);
	} else if(newUnreliability > playerNotVisibleThreshold and (ros::Time::now()-latestPlayerLoss).toSec() > 5.0
	          and oldUnreliability < 2.0) {
		std::cout <<"torno indietro"<< (ros::Time::now()-latestPlayerLoss).toSec()<<std::endl;
		input["Unreliability"] = 2.5;
		//currentState=NORMAL;
	}

}
void BasketBotBrain::rotateAndSearch(float direction)
{
	setState((direction<0)?SEARCH_RIGHT:SEARCH_LEFT,5.0);

}
void BasketBotBrain::generateObstaclesData()
{
	std::vector<float> obstacles = messenger->getObstacles(obstaclesRadarDistance);
	input["ObstacleFront"] = obstacles[0];
	input["ObstacleFrontLeft"] = obstacles[1];
	input["ObstacleLeft"] = obstacles[2];
	input["ObstacleRearLeft"] = obstacles[3];
	input["ObstacleRear"] = obstacles[4];
	input["ObstacleRearRight"] = obstacles[5];
	input["ObstacleRight"] = obstacles[6];
	input["ObstacleFrontRight"] = obstacles[7];
}
float BasketBotBrain::getCurrentStateElapsed()
{
	return (ros::Time::now()-latestStateChange).toSec();
}
void BasketBotBrain::runBrian()
{
	/*if(currentState != NORMAL) {
		stateDuration -= elapsedTime.toSec();
		if(stateDuration <= 0)
			currentState = NORMAL;
	}*/
	bool debugPrints = showBrianDebug && timeThrottle.checkElapsedNamed("brianDebug",0.1);

	generateObstaclesData();
if(debugPrints)
	std::cout << "unr: "<<messenger->getPlayerPositionUnreliability()<<std::endl;

	//create brian variables

	RosBrianBridge::Goal goal = messenger->getGoal();
	input["AutoMode"] = autoMode?1.0:0;
	input["GoalDistance"] = goal.distance;
	input["GoalAngle"] = goal.angle;
	input["GoalAge"] = goal.age;
	RosBrianBridge::BiFloat robotSpeeds = messenger->getRobotSpeed();
	if(robotSpeeds.second == 0.0)
		robotSpeeds.second = 0.01;
	input["RobotLinearSpeed"] = robotSpeeds.first;
	

	input["RobotAngularSpeed"] = robotSpeeds.second;
	input["PlayerDistance" ] = (messenger->getPlayerDistance() - distanceOffset) * distanceSensitivity;
	if(messenger->getPlayerDistance()<0.5)
		input["PlayerDistance" ] = -100.0;
	input["PlayerOrientation"] = messenger->getPlayerOrientation()+orientationOffset;
	input["PlayerVelocityX"] = messenger->getPlayerVelocityX()*playerSpeedSensitivity;
	input["PlayerVelocityY"] = messenger->getPlayerVelocityY()*playerSpeedSensitivity/messenger->getPlayerDistance();
	input["DebugFlag"] = 1;

	input["SuggestedLinearSpeed" ] = messenger->getSuggestedLinearSpeed();
	input["SuggestedAngularSpeed"] = messenger->getSuggestedAngularSpeed();

	checkUnreliability();


	float elapsed = getCurrentStateElapsed();
	input["StateElapsed"] = elapsed;
    if(currentState == FROZEN)
		input["FreezeCountdown"] = freezeTime - elapsed;

	if(debugPrints)
		brian.setVerbosity(1);
	else
		brian.setVerbosity(0);
	output = brian.execute(input);



	float speed = 0,rotation=0;
	speed = output["SpeedModule"];
	rotation = output["RotSpeed"];
	speed=applyShape(speed,outputSnappiness);
	rotation=applyShape(rotation,outputSnappiness);
	messenger->setSpeed(speed,rotation);


	setState( output["RobotStatus"]);
	output["RobotStatus"] = (int) currentState;
	if(debugPrints) {
		std::cerr << (int) currentState<<std::endl;
		std::cerr<<"obstacle range: "<<obstaclesRadarDistance<<std::endl;

	}



}


BasketBotBrain::BasketBotBrain(RosBrianBridge *messenger,std::string config_path): messenger(messenger),brian(config_path,1),currentState(NONE)
{
	orientationOffset = 0;


	playerSpeedSensitivity = 0.2;
	reverseRotationThreshold = 0.5;
	outputSnappiness = 0;
	playerNotVisibleThreshold = 8.0;

	setState(EXPLORE);
}

float BasketBotBrain::getParameter(std::string name)
{
	if(name == "distanceOffset")
		return distanceOffset;
	else if(name == "distanceSensitivity")
		return distanceSensitivity;
	else if(name == "playerSpeedSensitivity")
		return playerSpeedSensitivity;
	else if(name == "outputSnappiness")
		return outputSnappiness;
	else if(name == "orientationOffset")
		return orientationOffset;
	else if(name == "obstaclesRadarDistance")
		return obstaclesRadarDistance;
	else {
		std::cerr<<"invalid brian parameter"<<std::endl;
		exit(1);
	}
}
void BasketBotBrain::setParameter(std::string name, float value)
{
	if(name == "distanceOffset")
		distanceOffset = value;
	else if(name == "distanceSensitivity")
		distanceSensitivity = value;
	else if(name == "playerSpeedSensitivity")
		playerSpeedSensitivity = value;
	else if(name == "outputSnappiness")
		outputSnappiness = value;
	else if(name == "orientationOffset")
		orientationOffset = value;
	else if(name == "obstaclesRadarDistance")
		obstaclesRadarDistance = value;
	else if(name == "brianDebug")
		showBrianDebug = value>0.5;
	else if(name == "autoMode")
		autoMode= value>0.5;
	else if(name == "freezeTime")
		freezeTime= value;
	else {
		std::cerr<<"invalid brian parameter"<<std::endl;
		exit(1);
	}
}
float BasketBotBrain::applyShape(float input,float snappiness)
{
	
	float result = input;
	if(snappiness < 0)
		result = copysign(pow(fabs(input),1.0-snappiness),input);
	if(snappiness > 0)
		result = copysign(pow(fabs(input),1/(1+snappiness)),input);
	
	//std::cerr<<"input: "<<input<<"  snappiness: "<<snappiness<<"  result: "<<result<<std::endl;	
	return result;

}
void BasketBotBrain::freeze(float seconds)
{
//	stateDuration = ros::Duration(seconds);
	setState(FROZEN,seconds);


}

void BasketBotBrain::setState(float _s, float seconds)
{
	BrainState s = BrainState((int) round(_s));
	if(s == NONE)
		return;
	if(currentState != s)
		latestStateChange = ros::Time::now();
	currentState = s;
	stateDuration = seconds;
	input["RobotStatus"] = _s;

}
bool BasketBotBrain::dangerCollision()
{
	return output["ObstacleDanger"]>0.5;
}

void BasketBotBrain::spinOnce()
{
	ros::Time n =ros::Time::now();
	elapsedTime = n - lastUpdate;
	lastUpdate = n;


	runBrian();
}
