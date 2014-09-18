#include "basketbot_brain.h"
#include <iostream>
#include <math.h>
#include "RosBrianBridge.h"

const unsigned int rate = 30;


void BasketBotBrain::rotateAndSearch(float direction)
{
	currentState = (direction<0)?SEARCH_RIGHT:SEARCH_LEFT;
	stateDuration = 5.0; 

}

void BasketBotBrain::runBrian()
{
	if(currentState != NORMAL) {
		stateDuration -= elapsedTime.toSec();
		if(stateDuration <= 0)
			currentState = NORMAL;
	}
	
	
	std::cout << "unr: "<<messenger->getPlayerPositionUnreliability()<<std::endl;

	//create brian variables

	input["PlayerDistance" ] = (messenger->getPlayerDistance() - distanceOffset) * distanceSensitivity;
	input["PlayerOrientation"] = messenger->getPlayerOrientation();
	input["PlayerVelocityX"] = messenger->getPlayerVelocityX()*playerSpeedSensitivity;
	input["PlayerVelocityY"] = messenger->getPlayerVelocityY()*playerSpeedSensitivity;
	input["DebugFlag"] = 1;

	input["SuggestedLinearSpeed" ] = messenger->getSuggestedLinearSpeed();
	input["SuggestedAngularSpeed"] = messenger->getSuggestedAngularSpeed();



	float currentUnreliability = messenger->getPlayerPositionUnreliability();
	if(input["Unreliability"]< playerNotVisibleThreshold && currentUnreliability >= playerNotVisibleThreshold)
		rotateAndSearch(-input["PlayerVelocityY"]);

	input["RobotStatus"] = currentState;


	input["Unreliability"] = (currentUnreliability -playerNotVisibleThreshold) +1;//currentUnreliability;

	output = brian.execute(input);

	float speed = 0,rotation=0;
	speed = output["SpeedModule"];
	rotation = output["RotSpeed"];
	speed=applyShape(speed,outputSnappiness);
	rotation=applyShape(rotation,outputSnappiness);
	messenger->setSpeed(speed,rotation);

	input["RobotLinearSpeed"] = speed;
	input["RobotAngularSpeed"] = rotation;
	currentState = BasketBotBrain::State ( (float) output["RobotStatus"]);

}


BasketBotBrain::BasketBotBrain(RosBrianBridge *messenger,std::string config_path): messenger(messenger),brian(config_path,1)
{
	distanceOffset = 2;
	distanceSensitivity = 0.5 ;
	playerSpeedSensitivity = 0.5;
	reverseRotationThreshold = 0.5;
	outputSnappiness = 0;
	playerNotVisibleThreshold = 1.8;
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
	currentState = FROZEN;
	stateDuration = seconds;


}
void BasketBotBrain::spinOnce()
{
	ros::Time n =ros::Time::now();
	elapsedTime = n - lastUpdate;
	lastUpdate = n;


	runBrian();
}
