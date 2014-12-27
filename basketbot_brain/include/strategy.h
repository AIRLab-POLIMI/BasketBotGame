#ifndef STRATEGY_H
#define STRATEGY_H
//Questo header ha dipendenze non ancora risolte

class BasketBotBrain;
class RosBrianBridge;

#include "time_throttle.h"
class Strategy
{
	enum StrategyState {NONE,LAST_POSITION,STAY_AWAY,TILT_LEFT,TILT_RIGHT,RANDOM};
	ros::NodeHandle nh;
	ros::Timer timer;
	BasketBotBrain* brain;
	RosBrianBridge* bridge;
	BrainState brainState;
	BrainState previousBrainState;
	StrategyState strategyState;
	tf::TransformListener transformListener;
	ros::Publisher goalPublisher;
	ros::Subscriber predictionSubscriber;
	ros::Time lastUpdate;
	TimeThrottle timeThrottle;

    bool userJustAppeared;
	bool playerSlow;
	bool userSeenAtLeastOnce;
	std::list<geometry_msgs::PointStamped> lastPlayerPositions;
	geometry_msgs::PointStamped lastPlayerPos;
	std::list<float> lastDistances;
	std::list<float> lastSpeeds;
	ros::Time timeUserSeenChange;
	player_tracker::PosPrediction lastPrediction;
	player_tracker::PosPrediction lastPredictionWhenDisappeared;


	void setBrianParameter(std::string name, float value);
	void strategyLoop(const ros::TimerEvent&);
	void publishGoalAbsolute(float x,float y);
	void publishGoalRelative(float x,float y);
	void predictionCallback(const player_tracker::PosPrediction::ConstPtr& msg);
	void setStrategyState(StrategyState);
	float calcAverage(std::list<float> &);
	bool isPlayerSlow();
	float elapsedFromStrategyChange();
	void  analyzeBrianState();
	void  analyzeUser();
	void  processEvent(std::string eventName, float eventSize);
	bool isEventRelevant(std::string eventName, float eventSize);
	void applyStrategy();
	void printDebugInfo();
public:
	Strategy(BasketBotBrain*,RosBrianBridge*);
};


#endif
