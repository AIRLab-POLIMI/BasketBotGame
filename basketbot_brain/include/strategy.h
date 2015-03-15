#ifndef STRATEGY_H
#define STRATEGY_H
//Questo header ha dipendenze non ancora risolte

class BasketBotBrain;
class RosBrianBridge;

#include "time_throttle.h"
#include "HotSpots.h"
#include "strategy_math.h"
#include "strategy_analyzer.h"
class Strategy
{
	//roba della strategia
	enum StrategyState {STOPPED,NONE,SLOW_ROTATION,PARTENZA,FREEZE,LAST_POSITION,STAY_AWAY,STAY_AWAY_SLOW,TILT_LEFT,TILT_RIGHT,RANDOM,PREVIOUS_STATE,THIS_STATE,GUESSED_POSITION,LATERALE_1,LATERALE_2,SCARTO_SINISTRA,SCARTO_DESTRA,AVVICINAMENTO_SINISTRA,AVVICINAMENTO_DESTRA,GIRO_DESTRA};
	ros::NodeHandle nh;
	ros::NodeHandle pnh;
	ros::Timer timer;
	BasketBotBrain* brain;
	RosBrianBridge* bridge;
	BrainState brainState;
	BrainState previousBrainState;
	StrategyState strategyState;
	StrategyState oldStrategyState;
	tf::TransformListener transformListener;
	ros::Publisher goalPublisher;
	ros::Subscriber predictionSubscriber;
	ros::Time lastUpdate;
	TimeThrottle timeThrottle;
	StrategyAnalyzer strategyAnalyzer;
	ros::Timer finePartitaTimer;

	//stato utente, robot
	bool userJustAppeared;
	bool playerSlow;
	bool userSeenAtLeastOnce;
	bool canestroDuranteFreeze;
	bool sguardoFisso;
	bool justUnfrozen;
	unsigned int canestri;
	
	std::list<geometry_msgs::PointStamped> lastPlayerPositions;
	geometry_msgs::PointStamped lastPlayerPos;
	std::list<float> lastDistances;
	std::list<float> lastOrientations;
	std::list<float> lastXorientations;
	std::list<float> lastRobotRotSpeeds;
	ros::Time timeUserSeenChange;
	ros::Time dataUltimoCanestro;
	player_tracker::PosPrediction lastPrediction;
	player_tracker::PosPrediction lastPredictionWhenDisappeared;
	void endGameCallback(const ros::TimerEvent& event);
	//metodi utili
	bool addAndCheckDataPoint(float,std::list<float> &,unsigned int);

	bool isPlayerSlow();

	
	HotSpots hotSpotsReturningVisible;
	
	void setBrianParameter(std::string name, float value);
	void strategyLoop(const ros::TimerEvent&);
	void publishGoalAbsolute(float x,float y);
	void publishGoalRelative(float x,float y);
	void predictionCallback(const player_tracker::PosPrediction::ConstPtr& msg);
	void setStrategyState(StrategyState);
	float elapsedFromStrategyChange();
	void  analyzeBrianState();
	void  analyzeUser();
	void  analyzeRobot();
	void  processEvent(std::string eventName, float eventSize);
	bool isEventRelevant(std::string eventName, float eventSize);
	void applyStrategy();
	void printDebugInfo();
	void start_stop();
	
	void loadParameters();
public:
	Strategy(BasketBotBrain*,RosBrianBridge*);
	void canestro();
	void poseDetected(std::string);
};


#endif
