#ifndef BASKETBOT_BRAIN_H
#define BASKETBOT_BRAIN_H

#include "BrianWrapper.h"
class RosBrianBridge;


class BasketBotBrain
{
	RosBrianBridge *messenger;
	BrianWrapper brian;
	BrianWrapper::DataContainer input, output;
	void runBrian();
	
	
	//STATE
	float currentUnreliability;
	float suggestedLinearSpeed;
	float suggestedAngularSpeed;
	float distance;
	float angle;
	float predictedVX;
	float predictedVY;
	
	//PARAMETERS
	float distanceOffset;
	float distanceSensitivity;
	float rotationCommandSensitivity;

	float outputSensitivity;
	float reverseRotationThreshold;
	float playerNotVisibleThreshold;
	float playerLostThreshold;
	
	
	
public:
	BasketBotBrain(RosBrianBridge *,std::string);
	void setPlayerPosPrediction(float x,float y,float vx,float vy);
	void setPlayerPosUnreliability(float u);
	void setSuggestedCmdVel(float speed,float twist);
	void spinOnce();
	
};

#endif