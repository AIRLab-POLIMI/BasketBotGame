#include "basketbot_brain.h"
#include <iostream>
#include <math.h>
#include "RosBrianBridge.h"

void BasketBotBrain::runBrian()
{
	//create brian variables
	
	input["PlayerDistance" ] = (messenger->getPlayerDistance() - distanceOffset) * distanceSensitivity;
	input["PlayerOrientation"] = angle;
	input["PlayerVelocityX"] = predictedVX;
	input["PlayerVelocityY"] = predictedVY;
	input["Unreliability"] = 0;//currentUnreliability;
	input["DebugFlag"] = 1;

	input["SuggestedLinearSpeed" ] = suggestedLinearSpeed;
	input["SuggestedAngularSpeed"] = suggestedAngularSpeed;


	output = brian.execute(input);

	float speed = 0,rotation=0;
	//if(output.find("SpeedModule") != output.end())
	speed = output["SpeedModule"]*outputSensitivity;
	//if(output.find("RotSpeed") != output.end())
	rotation = output["RotSpeed"]*outputSensitivity;

	messenger->setSpeed(speed,rotation);

}

void BasketBotBrain::setSuggestedCmdVel(float speed,float twist)
{
	suggestedLinearSpeed = speed;
	suggestedAngularSpeed = twist;
}

BasketBotBrain::BasketBotBrain(RosBrianBridge *messenger,std::string config_path): messenger(messenger),brian(config_path,1)
{
	distanceOffset = 2;
	distanceSensitivity = 0.5 ;
	outputSensitivity = 1;
	reverseRotationThreshold = 0.5;
	suggestedLinearSpeed=0;
	suggestedAngularSpeed=0;
}

void BasketBotBrain::setPlayerPosUnreliability(float u)
{
	currentUnreliability = u;

}
void BasketBotBrain::spinOnce()
{
	
	runBrian();
}
void BasketBotBrain::setPlayerPosPrediction(float x,float y,float vx,float vy)
{
	//use predictions
	float predictedX = x;
	float predictedY  = y ;
	predictedVX = vx;
	predictedVY = vy;

	distance = sqrt(predictedX*predictedX+predictedY*predictedY) - distanceOffset;
	angle = atan2(predictedY,predictedX);
	std::cout << "distanza: " << distance << std::endl;




	runBrian();


}
