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
		//currentState=NORMAL;
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
	
	generateObstaclesData();
	
	std::cout << "unr: "<<messenger->getPlayerPositionUnreliability()<<std::endl;

	//create brian variables

	RosBrianBridge::Goal goal = messenger->getGoal();
	input["GoalDistance"] = goal.distance;
	input["GoalAngle"] = goal.angle;
	input["GoalAge"] = goal.age;
	RosBrianBridge::BiFloat robotSpeeds = messenger->getRobotSpeed();
	input["RobotLinearSpeed"] = robotSpeeds.first;
	input["RobotAngularSpeed"] = robotSpeeds.second;
	input["PlayerDistance" ] = (messenger->getPlayerDistance() - distanceOffset) * distanceSensitivity;
	input["PlayerOrientation"] = messenger->getPlayerOrientation()+orientationOffset;
	input["PlayerVelocityX"] = messenger->getPlayerVelocityX()*playerSpeedSensitivity;
	input["PlayerVelocityY"] = messenger->getPlayerVelocityY()*playerSpeedSensitivity*3.0;
	input["DebugFlag"] = 1;

	input["SuggestedLinearSpeed" ] = messenger->getSuggestedLinearSpeed();
	input["SuggestedAngularSpeed"] = messenger->getSuggestedAngularSpeed();

	checkUnreliability();

	

	input["StateElapsed"] = getCurrentStateElapsed();
	output = brian.execute(input);

	float speed = 0,rotation=0;
	speed = output["SpeedModule"];
	rotation = output["RotSpeed"];
	speed=applyShape(speed,outputSnappiness);
	rotation=applyShape(rotation,outputSnappiness);
	messenger->setSpeed(speed,rotation);


	setState( output["RobotStatus"]);
	output["RobotStatus"] = (int) currentState;
	std::cerr << (int) currentState<<std::endl;
	/*if(input["StateElapsed"] > 5.0 && currentState != NORMAL)
	{
		setState(NORMAL);
		std::cerr <<"reset"<<std::endl;
	}*/
	
}


BasketBotBrain::BasketBotBrain(RosBrianBridge *messenger,std::string config_path): messenger(messenger),brian(config_path,1),currentState(NONE),strategy(this)
{
	orientationOffset = 0;
	distanceOffset = 1;
	distanceSensitivity = 0.2 ;
	playerSpeedSensitivity = 0.2;
	reverseRotationThreshold = 0.5;
	outputSnappiness = 0;
	playerNotVisibleThreshold = 8.0;
	
	setState(NORMAL);
}
void BasketBotBrain::setParameter(std::string name, float value)
{
	if(name == "distanceOffset")
		distanceOffset = value;
	else if(name == "distanceSensitivity")
		distanceSensitivity = value;
	else if(name == "playerSpeedSensitivity")
		playerSpeedSensitivity = 0.2;
	else if(name == "outputSnappiness")
		outputSnappiness = value;
	else if(name == "orientationOffset")
		orientationOffset = value;
	else
	{
		std::cerr<<"invalid brian parameter"<<std::endl;
		exit(1);
	}
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


void BasketBotBrain::spinOnce()
{
	ros::Time n =ros::Time::now();
	elapsedTime = n - lastUpdate;
	lastUpdate = n;


	runBrian();
}
