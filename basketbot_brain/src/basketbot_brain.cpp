#include "basketbot_brain.h"
#include <iostream>
#include <math.h>

void BasketBotBrain::runBrian()
{
	//create brian variables
	
	float distance = sqrt(predictedX*predictedX+predictedY*predictedY) - distanceOffset;
	float angle = atan2(predictedY,predictedX);
	
	
	
	std::cout << "distanza: " << distance << std::endl;
		input ["SuggestedLinearSpeed" ] = suggestedLinearSpeed;
	input ["SuggestedAngularSpeed"] = suggestedAngularSpeed;
	input ["PlayerDistance" ] = distance * distanceSensitivity;
	input ["PlayerOrientation"] = angle;
	input["DebugFlag"] = 1;
	
	
	
	
	output = brian.execute(input);
	input.clear();
	

	if(output.size() >= 2)
		messenger->sendCommands(output["SpeedModule"],-rotationCommandSensitivity*output["RotSpeed"]);
	
}

void BasketBotBrain::setSuggestedCmdVel(float speed,float twist)
{
	suggestedLinearSpeed = speed;
	suggestedAngularSpeed = twist;

}

BasketBotBrain::BasketBotBrain(MessengerInterface *messenger,std::string config_path): messenger(messenger),brian(config_path,1)
{
	distanceOffset = 1.0;
	distanceSensitivity = 4.0;
	rotationCommandSensitivity = 8.0;
	
	
}

void BasketBotBrain::setPlayerPosUnreliability(float u)
{
		currentUnreliability = u;
		std::cout <<"unreliability "<<u<<std::endl;
	
}

void BasketBotBrain::setPlayerPosPrediction(float x,float y,float vx,float vy)
{
	//use predictions
	predictedX = x + 2.0*vx;
	predictedY  = y + 2.0*vy;
	predictedVX = vx;
	predictedVY = vy;
	
	runBrian();
	
	
}