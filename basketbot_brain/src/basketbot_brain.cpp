#include "basketbot_brain.h"
#include <iostream>
#include <math.h>
#include "RosBrianBridge.h"

const unsigned int rate = 30;

void BasketBotBrain::checkUnreliability()
{
	float newUnreliability = messenger->getPlayerPositionUnreliability();
	float oldUnreliability = input["Unreliability"];
	
	if(newUnreliability < playerNotVisibleThreshold and oldUnreliability > 1.0)
	{
		input["Unreliability"] = 0.5;
	}
	else if(newUnreliability > playerNotVisibleThreshold and oldUnreliability < 1.0)
	{
		input["Unreliability"] = 1.5;
		latestPlayerLoss = ros::Time::now();
		if(currentState == NORMAL)
			;//rotateAndSearch(-input["PlayerVelocityY"]);
	}
	else if(newUnreliability > playerNotVisibleThreshold and (ros::Time::now()-latestPlayerLoss).toSec() > 5.0 
	and oldUnreliability < 2.0)
	{
		std::cout <<"torno indietro"<< (ros::Time::now()-latestPlayerLoss).toSec()<<std::endl;
		input["Unreliability"] = 2.5;
		currentState=NORMAL;
	}
	
}
void BasketBotBrain::rotateAndSearch(float direction)
{
	setState((direction<0)?SEARCH_RIGHT:SEARCH_LEFT,5.0);

}
void BasketBotBrain::generateObstaclesData()
{
	std::vector<float> obstacles = messenger->getObstacles();
	input["ObstacleFront"] = obstacles[0];
	input["ObstacleFrontLeft"] = obstacles[1];
	input["ObstacleLeft"] = obstacles[2];
	input["ObstacleRearLeft"] = obstacles[3];
	input["ObstacleRear"] = obstacles[4];
	input["ObstacleRearRight"] = obstacles[5];
	input["ObstacleRight"] = obstacles[6];
	input["ObstacleFrontRight"] = obstacles[7];
}
void BasketBotBrain::runBrian()
{
	if(currentState != NORMAL) {
		stateDuration -= elapsedTime.toSec();
		if(stateDuration <= 0)
			currentState = NORMAL;
	}
	
	generateObstaclesData();
	
	std::cout << "unr: "<<messenger->getPlayerPositionUnreliability()<<std::endl;

	//create brian variables

	RosBrianBridge::BiFloat robotSpeeds = messenger->getRobotSpeed();
	input["RobotLinearSpeed"] = robotSpeeds.first;
	input["RobotAngularSpeed"] = robotSpeeds.second;
	input["PlayerDistance" ] = (messenger->getPlayerDistance() - distanceOffset) * distanceSensitivity;
	input["PlayerOrientation"] = messenger->getPlayerOrientation();
	input["PlayerVelocityX"] = messenger->getPlayerVelocityX()*playerSpeedSensitivity;
	input["PlayerVelocityY"] = messenger->getPlayerVelocityY()*playerSpeedSensitivity;
	input["DebugFlag"] = 1;

	input["SuggestedLinearSpeed" ] = messenger->getSuggestedLinearSpeed();
	input["SuggestedAngularSpeed"] = messenger->getSuggestedAngularSpeed();

	checkUnreliability();

	input["RobotStatus"] = currentState;

	input["StateElapsed"] = (ros::Time::now()-latestStateChange).toSec();
	output = brian.execute(input);

	float speed = 0,rotation=0;
	speed = output["SpeedModule"];
	rotation = output["RotSpeed"];
	speed=applyShape(speed,outputSnappiness);
	rotation=applyShape(rotation,outputSnappiness);
	messenger->setSpeed(speed,rotation);


	setState(BasketBotBrain::State ( (int) round(output["RobotStatus"])));
	
	

}


BasketBotBrain::BasketBotBrain(RosBrianBridge *messenger,std::string config_path): messenger(messenger),brian(config_path,1),currentState(NONE)
{
	distanceOffset = 1;
	distanceSensitivity = 0.2 ;
	playerSpeedSensitivity = 0.2;
	reverseRotationThreshold = 0.5;
	outputSnappiness = 0;
	playerNotVisibleThreshold = 5.0;
	
	setState(NORMAL);
}

float BasketBotBrain::applyShape(float input,float shape)
{
	if(shape > 0)
		return pow(input,shape+1.0);
	if(shape < 0)
		return pow(input,1/(1-shape));
	return input;

}
void BasketBotBrain::freeze(float seconds)
{
//	stateDuration = ros::Duration(seconds);
	setState(FROZEN,seconds);


}

void BasketBotBrain::setState(State s, float seconds)
{
	if(s == NONE)
		return;
	if(currentState != s)
		latestStateChange = ros::Time::now();
	currentState = s;
	stateDuration = seconds;
	
	
	
}
void BasketBotBrain::spinOnce()
{
	ros::Time n =ros::Time::now();
	elapsedTime = n - lastUpdate;
	lastUpdate = n;


	runBrian();
}
