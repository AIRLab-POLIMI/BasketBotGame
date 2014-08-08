#include "basketbot_brain.h"
#include <iostream>
#include <math.h>

void BasketBotBrain::runBrian()
{
	//create brian variables




	output = brian.execute(input);

	/*BrianWrapper::DataContainer temp;
	temp["PlayerDistance" ] = input["PlayerDistance" ];
	temp["PlayerOrientation" ] = input["PlayerOrientation" ];
	temp["DebugFlag" ] = input["DebugFlag"];
	temp["PlayerVelocityX"] = input ["PlayerVelocityX"];
	temp["PlayerVelocityY"] = input ["PlayerVelocityY"];
	temp["SuggestedLinearSpeed"] = input ["SuggestedLinearSpeed"];
	temp["SuggestedAngularSpeed"] = input ["SuggestedAngularSpeed"];
	std::swap(temp,input);*/


	float speed = 0,rotation=0;
	//if(output.find("SpeedModule") != output.end())
		speed = output["SpeedModule"]*outputSensitivity;
	//if(output.find("RotSpeed") != output.end())
		rotation = output["RotSpeed"]*outputSensitivity;
	
		messenger->sendCommands(speed,rotation);

}

void BasketBotBrain::setSuggestedCmdVel(float speed,float twist)
{
	input["SuggestedLinearSpeed" ] = speed;
	input["SuggestedAngularSpeed"] = twist;
	if(speed != 0 ||twist != 0)
		input["SuggestionAvailable"] = 1;
	else
		input["SuggestionAvailable"] = 0;
}

BasketBotBrain::BasketBotBrain(MessengerInterface *messenger,std::string config_path): messenger(messenger),brian(config_path,1)
{
	distanceOffset = 2;
	distanceSensitivity = 0.5 ;
	outputSensitivity = 1;
	input["DebugFlag"] = 1;

}

void BasketBotBrain::setPlayerPosUnreliability(float u)
{
	currentUnreliability = u;
	std::cout <<"unreliability "<<u<<std::endl;
	input["unreliability"] = u;
}

void BasketBotBrain::setPlayerPosPrediction(float x,float y,float vx,float vy)
{
	//use predictions
	float predictedX = x;
	float predictedY  = y ;
	float predictedVX = vx;
	float predictedVY = vy;

	float distance = sqrt(predictedX*predictedX+predictedY*predictedY) - distanceOffset;
	float angle = atan2(predictedY,predictedX);
	std::cout << "distanza: " << distance << std::endl;
	input["PlayerDistance" ] = distance * distanceSensitivity;
	input["PlayerOrientation"] = angle;
	input["PlayerVelocityX"] = predictedVX;
	input["PlayerVelocityY"] = predictedVY;
	
	runBrian();


}
