#ifndef STRATEGY_H
#define STRATEGY_H
//Questo header ha dipendenze non ancora risolte

class BasketBotBrain;


class Strategy
{
	enum StrategyState {NONE,LAST_POSITION,STAY_AWAY,TILT_LEFT,TILT_RIGHT,RANDOM};
	ros::NodeHandle nh;
	ros::Timer timer;
	BasketBotBrain* brain;
	BrainState brainState;
	StrategyState strategyState;
	tf::TransformListener transformListener;
	ros::Publisher goalPublisher;
	ros::Subscriber predictionSubscriber;
	ros::Time lastUpdate;

	bool playerSlow;

	std::list<geometry_msgs::PointStamped> lastPlayerPositions;
	geometry_msgs::PointStamped lastPlayerPos;

	void setBrianParameter(std::string name, float value);
	void timerCallback(const ros::TimerEvent&);
	void publishGoalAbsolute(float x,float y);
	void publishGoalRelative(float x,float y);
	void predictionCallback(const player_tracker::PosPrediction::ConstPtr& msg);
	void setState(StrategyState);
	bool isPlayerSlow();
	float elapsedFromStrategyChange();
public:
	Strategy(BasketBotBrain*);
};


#endif