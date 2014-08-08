#include "BrianWrapper.h"

class MessengerInterface
{
		public:
	virtual void sendCommands(float v, float rot) =0;
	
};
class BasketBotBrain
{
	MessengerInterface *messenger;
	BrianWrapper brian;
	BrianWrapper::DataContainer input, output;
	void runBrian();
	
	
	//STATE
	float currentUnreliability;
	
	
	//PARAMETERS
	float distanceOffset;
	float distanceSensitivity;
	float rotationCommandSensitivity;
	float suggestedLinearSpeed;
	float suggestedAngularSpeed;
	float outputSensitivity;
	
	
	
public:
	BasketBotBrain(MessengerInterface *,std::string);
	void setPlayerPosPrediction(float x,float y,float vx,float vy);
	void setPlayerPosUnreliability(float u);
	void setSuggestedCmdVel(float speed,float twist);
	
	
};