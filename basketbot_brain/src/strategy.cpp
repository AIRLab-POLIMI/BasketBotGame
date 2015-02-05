#include <ros/ros.h>
#include "basketbot_brain.h"
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include "RosBrianBridge.h"
double defaultDistanceOffset = 1.2;
double defaultDistanceSensitivity = 0.5 ;
double defaultObstaclesRadarDistance = 1.0;
double defaultPlayerSpeedSensitivity = 0.2;

double minSlowThresh = 0.4;
double maxSlowThresh = 0.5;
float minDistErrorThresh = 0.4;
float maxDistErrorThresh = 0.5;

bool strategia_anti_annoiamento=false;
bool autoStart = true;

void die(std::string message)
{
	std::cerr<<message<<std::endl;
	ros::shutdown();
	exit(1);
}
#define GET_PARAM(x,y)  if(!pnh.getParam(x,y)) die("missing param" x)
Strategy::Strategy(BasketBotBrain* brain,RosBrianBridge* bridge) :nh(),pnh("~"),brain(brain),bridge(bridge),hotSpotsReturningVisible()
{
	goalPublisher = nh.advertise<geometry_msgs::PoseStamped >("/brain/goal",10);
	predictionSubscriber = nh.subscribe("PosPrediction", 2, &Strategy::predictionCallback,this);


	GET_PARAM("default_distance_offset",defaultDistanceOffset);
	GET_PARAM("default_distance_sensitivity",defaultDistanceSensitivity);
	GET_PARAM("default_obstacles_radar_distance",defaultObstaclesRadarDistance);
	GET_PARAM("min_slow_threshold",minSlowThresh);
	GET_PARAM("max_slow_threshold",maxSlowThresh);
GET_PARAM("player_speed_sensitivity",defaultPlayerSpeedSensitivity);
	hotSpotsReturningVisible.resetGrid(25,25,0.2);


	std::cerr<<"distoff: "<<defaultDistanceOffset<<std::endl;

	timer = nh.createTimer(ros::Duration(1.0/10.0), &Strategy::strategyLoop,this);
	srand(time(NULL));
	brain->setParameter("distanceOffset",defaultDistanceOffset);
	brain->setParameter("distanceSensitivity",defaultDistanceSensitivity);
	brain->setParameter("obstaclesRadarDistance",defaultObstaclesRadarDistance);
	brain->setParameter("playerSpeedSensitivity",defaultPlayerSpeedSensitivity);
	userSeenAtLeastOnce = false;
	brainState =brain->getState();
	if(!autoStart)
		setStrategyState(STOPPED);
	else
		setStrategyState(NONE);






}
#undef GET_PARAM

bool Strategy::isPlayerSlow()
{
	return playerSlow;


}
void Strategy::predictionCallback(const player_tracker::PosPrediction::ConstPtr& msg)
{

	if(brainState != NORMAL)
		return;

	//salvo l'ultima posizione nota
	tf::Transform transform;
	try {
		tf::StampedTransform tr;
		transformListener.lookupTransform ( "odom","base_footprint",ros::Time(0),tr);
		transform=tr;
	} catch(...) {
		return;
	}
	tf::Transform userTransform;
	userTransform.setOrigin(tf::Vector3(msg->position.x,msg->position.y,0));
	userTransform.setRotation(tf::Quaternion(0,0,0,1));

	transform = transform * userTransform;

	lastPlayerPos.point.x = transform.getOrigin().x();
	lastPlayerPos.point.y = transform.getOrigin().y();
	lastPlayerPos.header.stamp = ros::Time::now();

	lastPlayerPositions.push_back(lastPlayerPos);
	if(lastPlayerPositions.size() > 100)
		lastPlayerPositions.pop_front();
	lastPrediction = *msg;
	float speedSquared = msg->velocity.x*msg->velocity.x + msg->velocity.y*msg->velocity.y;
	float speed = sqrt(speedSquared);

	processEvent("user_position",0.0);



}
void Strategy::publishGoalRelative(float x,float y)
{
	tf::Transform transform;
	try {
		tf::StampedTransform tr;
		transformListener.lookupTransform ( "odom","base_footprint",ros::Time(0),tr);
		transform=tr;
	} catch(...) {
		return;
	}
	tf::Transform userTransform;
	userTransform.setOrigin(tf::Vector3(x,y,0));
	userTransform.setRotation(tf::Quaternion(0,0,0,1));

	transform = transform * userTransform;

	publishGoalAbsolute(transform.getOrigin().x(),transform.getOrigin().y());
}
void Strategy::setBrianParameter(std::string name, float value)
{
	brain->setParameter(name,value);
}
void Strategy::publishGoalAbsolute(float x,float y)
{
	geometry_msgs::PoseStamped goal;
	goal.pose.position.x = x;
	goal.pose.position.y = y;
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "odom";
	goalPublisher.publish(goal);
}
float Strategy::elapsedFromStrategyChange()
{
	return (ros::Time::now()-lastUpdate).toSec();
}

void  Strategy::analyzeBrianState()
{
	if(brain->dangerCollision())
		processEvent("danger_collision",1.0);


	BrainState newState = brain->getState();
	previousBrainState = brainState;
	if(newState == previousBrainState)
		return;


	brainState = newState;
	if(newState == NORMAL || previousBrainState == NORMAL) {
		ros::Time now = ros::Time::now();
		float stateElapsed = (now-timeUserSeenChange).toSec();
		timeUserSeenChange = now;


		if(newState ==NORMAL && !userSeenAtLeastOnce) {
			userSeenAtLeastOnce = true;
		} else if(newState ==NORMAL && userSeenAtLeastOnce) {
			processEvent("user_visible",stateElapsed);

		} else if(previousBrainState == NORMAL && userSeenAtLeastOnce) {
			lastPredictionWhenDisappeared = lastPrediction;
			processEvent("user_lost",stateElapsed);

		}
	}

	switch(newState) {
	case NORMAL:

		setStrategyState(NONE);
		break;

	case EXPLORE:
		setStrategyState(LAST_POSITION);

		break;
	case FROZEN:
		setStrategyState(FREEZE);
		break;

	default:
		setStrategyState(NONE);
		break;
	}





}


bool Strategy::isEventRelevant(std::string eventName, float eventSize)
{

	if(eventName == "speed" ) {
		if(!timeThrottle.checkElapsedNamed(eventName,3.0))
			return false;
		if(eventSize < minSlowThresh && !playerSlow) { //playerSlow
			return true;
		}
		if(eventSize > maxSlowThresh && playerSlow) { //player fast
			return true;
		}

	}


	else if(eventName == "distance_error") {
		if(!timeThrottle.checkElapsedNamed(eventName,4.0))
			return false;

		if(eventSize > maxDistErrorThresh ) {
			return true;
		}
		if(eventSize < minDistErrorThresh ) {
			return true;
		}


		//
	} else if(eventName == "user_visible") {
		return true;
	} else if(eventName == "user_lost") {
		return true;
	} else if(eventName == "danger_collision") {
		if(!timeThrottle.checkElapsedNamed(eventName,4.0))
			return false;
		return true;
	} else if(eventName == "freeze") {
		return true;
	} else if(eventName == "orientation_error") {
		if(!timeThrottle.checkElapsedNamed(eventName,2.0))
			return false;
		return true;
	} else if(eventName == "cross_orientation_velocity") {
		if(!timeThrottle.checkElapsedNamed(eventName,2.0))
			return false;
		return true;
	} else if(eventName == "user_position") {
		return userJustAppeared;

	} else {
		return true;
	}


	return false;

}

bool Strategy::canestroDuranteFreeze()
{
	return (strategyState == FREEZE && lastUpdate < dataUltimoCanestro);

}

void  Strategy::processEvent(std::string eventName, float eventSize)
{
	//ROS_INFO_STREAM_THROTTLE_NAMED(1,eventName,"EVENT: "<<eventName<<"\t"<<eventSize);
	if(!isEventRelevant(eventName,eventSize))
		return;
	if(eventName == "speed" ) {

		if(eventSize < minSlowThresh && !playerSlow) { //playerSlow
			ROS_INFO_STREAM("playerSlow \t"<<eventSize);
			playerSlow = true;

		}
		if(eventSize > maxSlowThresh && playerSlow) { //player fast
			ROS_INFO_STREAM("player fast \t"<<eventSize);
			playerSlow = false;
		}



	} else if(eventName == "distance_error") {

		if(eventSize > maxDistErrorThresh ) {
			float ds = brain->getParameter("distanceSensitivity");
			if(ds < defaultDistanceSensitivity * 1.5)
				brain->setParameter("distanceSensitivity",ds*1.1);
			ROS_INFO_STREAM("cannot keep distance \t"<<eventSize);
		}
		if(eventSize < minDistErrorThresh ) {
			float ds = brain->getParameter("distanceSensitivity");
			if(ds > defaultDistanceSensitivity / 1.5)
				brain->setParameter("distanceSensitivity",ds/1.1);
			ROS_INFO_STREAM("distance kept too aggressively \t"<<eventSize);
		}
		ROS_INFO_STREAM("EVENT: "<<eventName<<"\t"<<eventSize);

		//
	} else if(eventName == "user_visible") {
		userJustAppeared = true;
		ROS_INFO_STREAM("EVENT: "<<eventName<<" "<<eventSize<<"    y: "<<lastPredictionWhenDisappeared.velocity.y);
	} else if(eventName == "user_position") { //triggered after user_visible
		userJustAppeared = false;

		hotSpotsReturningVisible.recordPoint(lastPlayerPos.point.x,lastPlayerPos.point.y);
		ROS_INFO_STREAM("EVENT!: "<<eventName);
	} else if(eventName == "user_lost") {
		ROS_INFO_STREAM("EVENT: "<<eventName<<"\t"<<eventSize<<"  "<<lastPrediction.position.y);

	} else if(eventName == "freeze") {
		ROS_INFO_STREAM("robot frozen");

	} else if(eventName == "unfreeze") {
		if(canestroDuranteFreeze()) {
			sguardoFisso = false;
		} else {

		}
		//controlla se e' stato fatto un canestro oppure no
	} else if(eventName == "canestro") {
		ROS_INFO_STREAM("canestro");
		if(strategyState==FREEZE) {


		} else {
			sguardoFisso = true;
		}
		//gestisci se era freezato oppure no
		//gestisci se canestro laterale oppure no
	} else if(eventName == "orientation_error") {
		ROS_INFO_STREAM("EVENT: "<<eventName<<"\t"<<eventSize<<"  "<<lastPrediction.position.y);
	} else if(eventName == "cross_orientation_velocity") {
		ROS_INFO_STREAM("EVENT: "<<eventName<<"\t"<<eventSize<<"  "<<lastPrediction.position.y);
	} else if(eventName == "danger_collision") {

		float obsRange = brain->getParameter("obstaclesRadarDistance");
		if(obsRange < defaultObstaclesRadarDistance*2.0) {
			brain->setParameter("obstaclesRadarDistance",obsRange*1.1);
			ROS_INFO_STREAM("Increased radar range to: "<<obsRange*1.1);
		}

		ROS_INFO_STREAM("EVENT: "<<eventName<<"\t"<<eventSize);
	} else {
		ROS_FATAL_STREAM("unrecognized event");
		exit(0);
	}

}

bool Strategy::addAndCheckDataPoint(float point,std::list<float> & lista,unsigned int maxNum)
{
	lista.push_back(point);
	if(lista.size()<=maxNum)
		return false;
	lista.pop_front();
	return true;

}
void  Strategy::analyzeUser()
{
	if(brainState == NORMAL) {
		float vx =bridge->getPlayerVelocityX();
		float vy = bridge->getPlayerVelocityY();
		float speed = sqrt(vx*vx+vy*vy);
		float distance = bridge->getPlayerDistance();

		float orientation = bridge->getPlayerOrientation();
		float Xorientation = orientation * vy;

		lastOrientations.push_back(orientation);
		lastDistances.push_back(distance);


		lastXorientations.push_back(Xorientation);
		if(lastOrientations.size()>100) {
			lastOrientations.pop_front();
			float media = StrategyMath::avgDistance(lastOrientations);

			processEvent("orientation_error",media);
		}
		if(lastXorientations.size()>100) {
			lastXorientations.pop_front();
			float media = StrategyMath::calcAverage(lastXorientations);
			processEvent("cross_orientation_velocity",media);

		}
		if(lastDistances.size() > 100) {
			lastDistances.pop_front();
			float media = StrategyMath::avgDistance(lastDistances,defaultDistanceOffset);

			processEvent("distance_error",media);


		}
		if(strategyAnalyzer.recordSpeed(speed))
			processEvent("speed",strategyAnalyzer.getAvgSpeed());




	}


}

/*  Qui viene gestito l'albero delle strategie e variazioni
 *
 *
 *
 * */

void Strategy::setStrategyState(StrategyState newState)
{
	if(newState == THIS_STATE) {
		newState = strategyState;
	} else if(newState == PREVIOUS_STATE) {
		if(oldStrategyState!= PREVIOUS_STATE)
			setStrategyState(oldStrategyState);
		return;
	} else if(strategyState == newState) {
		return;
	}


	ROS_INFO_STREAM("StrategyState: "<<newState);

	oldStrategyState = strategyState;
	strategyState = newState;
	if(oldStrategyState == STAY_AWAY) {
		brain->setParameter("distanceOffset",defaultDistanceOffset);
	}
	if(oldStrategyState == TILT_LEFT || oldStrategyState == TILT_RIGHT) {
		brain->setParameter("orientationOffset",0);
	}
	if(oldStrategyState == LATERALE_1 || oldStrategyState == LATERALE_2) {
		brain->setParameter("orientationOffset",0);
		brain->setParameter("distanceOffset",defaultDistanceOffset);
	}
	if(oldStrategyState == FREEZE) {
		double el = (ros::Time::now()-lastUpdate).toSec();
		processEvent("unfreeze",el);
	}

	if(oldStrategyState == SCARTO_SINISTRA) {
		brain->setParameter("orientationOffset",0);
		brain->setParameter("distanceOffset",defaultDistanceOffset);
		float ds = brain->getParameter("distanceSensitivity");
		brain->setParameter("distanceSensitivity",ds/1.5);
	}
	if(oldStrategyState == SCARTO_DESTRA) {
		brain->setParameter("orientationOffset",0);
		brain->setParameter("distanceOffset",defaultDistanceOffset);
		float ds = brain->getParameter("distanceSensitivity");
		brain->setParameter("distanceSensitivity",ds/1.5);
	}
	if(oldStrategyState == AVVICINAMENTO_SINISTRA) {
		brain->setParameter("orientationOffset",0);
		brain->setParameter("distanceOffset",defaultDistanceOffset);
	}
	if(oldStrategyState == AVVICINAMENTO_DESTRA) {
		brain->setParameter("orientationOffset",0);
		brain->setParameter("distanceOffset",defaultDistanceOffset);
	}

	if(newState == SCARTO_SINISTRA) {
		if(lastPrediction.velocity.y > 0)
			brain->setParameter("orientationOffset",0.3);
		brain->setParameter("distanceOffset",defaultDistanceOffset*3.0);
		float ds = brain->getParameter("distanceSensitivity");
		brain->setParameter("distanceSensitivity",ds*1.5);
	}
	if(newState == SCARTO_DESTRA) {
		if(lastPrediction.velocity.y < 0)
			brain->setParameter("orientationOffset",-0.3);
		brain->setParameter("distanceOffset",defaultDistanceOffset*3.0);
		float ds = brain->getParameter("distanceSensitivity");
		brain->setParameter("distanceSensitivity",ds*1.5);
	}
	if(newState == AVVICINAMENTO_SINISTRA) {
		brain->setParameter("orientationOffset",-0.3);
		brain->setParameter("distanceOffset",defaultDistanceOffset*0.7);
	}
	if(newState == AVVICINAMENTO_DESTRA) {
		brain->setParameter("orientationOffset",0.3);
		brain->setParameter("distanceOffset",defaultDistanceOffset*0.7);
	}


	if(newState == TILT_LEFT) {
		brain->setParameter("orientationOffset",0.2);
	}
	if(newState == TILT_RIGHT) {
		brain->setParameter("orientationOffset",-0.2);
	}
	if(newState ==STAY_AWAY) {
		brain->setParameter("distanceOffset",defaultDistanceOffset*2.0);
	}

	if(newState == LATERALE_1) {
		brain->setParameter("orientationOffset",-0.2);
		brain->setParameter("distanceOffset",defaultDistanceOffset*2.0);
	}
	if(newState == LATERALE_2) {
		brain->setParameter("orientationOffset",0.2);
	}
	if(newState == FREEZE) {
		processEvent("freeze",1.0);
	}
	lastUpdate = ros::Time::now();
}
void Strategy::start_stop()
{

	exit(1);
}
void Strategy::applyStrategy()
{

	float obsRange = brain->getParameter("obstaclesRadarDistance");
	if(obsRange > defaultObstaclesRadarDistance) {
		brain->setParameter("obstaclesRadarDistance",defaultObstaclesRadarDistance +0.95*(obsRange-defaultObstaclesRadarDistance));

	}

	bool slow = isPlayerSlow();
	std::cerr<<"slow: "<<slow<<std::endl;

	switch(strategyState) {
	case STOPPED:
		brain->freeze(5000);
		break;
	case NONE:
		if(!brain->isVisible())
			break;
		if( elapsedFromStrategyChange()> 3.0 && slow && strategia_anti_annoiamento) {
			ROS_INFO_STREAM("Player is bored? let's try something...");
			int rn = rand() % 3;
			if(rn==0)//due poss. su tre che si allontani
				setStrategyState(STAY_AWAY);
			else if(rn==1)
				setStrategyState(LATERALE_1);
			else
				setStrategyState(TILT_LEFT);
			break;
		}
		if( elapsedFromStrategyChange()> 0.5 && !slow && !sguardoFisso ) {
			if(lastPrediction.velocity.y > 0)
				setStrategyState(SCARTO_SINISTRA);
			else
				setStrategyState(SCARTO_DESTRA);
			break;
		}
		if( elapsedFromStrategyChange()> 0.5 && !slow && sguardoFisso ) {
			setStrategyState(STAY_AWAY);

		}
		break;


	case SCARTO_SINISTRA:
		if( elapsedFromStrategyChange()> 3.0) {
			setStrategyState(AVVICINAMENTO_SINISTRA);
		}

		break;
	case SCARTO_DESTRA:
		if( elapsedFromStrategyChange()> 3.0) {
			setStrategyState(AVVICINAMENTO_DESTRA);

		}
	case AVVICINAMENTO_SINISTRA:
		if( elapsedFromStrategyChange()> 3.0) {
			setStrategyState(SCARTO_SINISTRA);
		}

		break;
	case AVVICINAMENTO_DESTRA:
		if( elapsedFromStrategyChange()> 3.0) {
			setStrategyState(SCARTO_DESTRA);

		}
		break;
	case GIRO_DESTRA:
		publishGoalRelative(-1,0.5);
		if( elapsedFromStrategyChange()> 3.0) {


			setStrategyState(PREVIOUS_STATE);
		}
		break;
	case STAY_AWAY:
		if( elapsedFromStrategyChange()> 0.5 && !slow) {
			ROS_INFO_STREAM("Ehi, Wait!");
			setStrategyState(NONE);
		} else if(elapsedFromStrategyChange()>4.0) {
			ROS_INFO_STREAM("I'm bored too");
			setStrategyState(NONE);

		}
		break;
	case TILT_LEFT:
		if( elapsedFromStrategyChange()> 0.5 && !slow)
			setStrategyState(NONE);
		else if(elapsedFromStrategyChange()>2.0 && slow)
			setStrategyState(TILT_RIGHT);

		break;
	case TILT_RIGHT:
		if( elapsedFromStrategyChange()> 0.5 && !slow)
			setStrategyState(NONE);
		else if(elapsedFromStrategyChange()>1.0 && rand()%10 == 0)
			setStrategyState(NONE);
		else if(elapsedFromStrategyChange()>2.0 && slow)
			setStrategyState(TILT_LEFT);

		break;
	case LATERALE_1:
		if( elapsedFromStrategyChange()> 0.5 && !slow)
			setStrategyState(NONE);
		else if(elapsedFromStrategyChange()>2.0 && slow)
			setStrategyState(LATERALE_2);

		break;
	case LATERALE_2:
		if( elapsedFromStrategyChange()> 0.5 && !slow)
			setStrategyState(NONE);
		else if(elapsedFromStrategyChange()>2.0 && slow)
			setStrategyState(LATERALE_1);
		break;
	case LAST_POSITION:
		if((ros::Time::now() - lastPlayerPos.header.stamp).toSec() < 10.0) {
			publishGoalAbsolute(lastPlayerPos.point.x,lastPlayerPos.point.y);
			std::cerr<<"X: "<<lastPlayerPos.point.x<<"   Y:  "<<lastPlayerPos.point.y<<std::endl;
		} else if(elapsedFromStrategyChange() > 1.0) {
			if(hotSpotsReturningVisible.getConfidence()>0.5)
				setStrategyState(GUESSED_POSITION);
			else
				setStrategyState(RANDOM);

		}
		break;

	case GUESSED_POSITION: {
		std::pair<float,float> posit = hotSpotsReturningVisible.getBestMatch();
		publishGoalAbsolute(posit.first,posit.second);
		if(elapsedFromStrategyChange() > 1.0)
			setStrategyState(RANDOM);
		break;

	}
	case RANDOM:
		if(!userSeenAtLeastOnce)
			break;
		if(elapsedFromStrategyChange() > 1.0) {
			float rd = rand();
			float radius = 0.5;
			publishGoalAbsolute(lastPlayerPos.point.x+radius*sin(rd),lastPlayerPos.point.y+radius*cos(rd));
			setStrategyState(LAST_POSITION);
		}

		break;
	}

}
void Strategy::canestro()
{
	/*tre situazioni:
	  canestro durante freeze,
	   * canestro senza freeze
	   * unfreeze senza canestro*/

	dataUltimoCanestro = ros::Time::now();
	if(strategyState==FREEZE) {
		processEvent("canestro",1.0);

	} else
		processEvent("canestro",-1.0);
}
void Strategy::printDebugInfo()
{
	if(strategyState==STAY_AWAY)
		std::cerr<<"SS: STAY_AWAY"<<std::endl;
	else if(strategyState==TILT_LEFT)
		std::cerr<<"SS: TILT_LEFT"<<std::endl;
	else
		std::cerr<<"SS: "<<strategyState<<std::endl;
}



void Strategy::analyzeRobot()
{
	RosBrianBridge::BiFloat speeds = bridge->getRobotSpeed();
	lastRobotRotSpeeds.push_back(speeds.second);

	float ampiezza_max = 1.0;
	int frequenza_max = 5;

	if(lastRobotRotSpeeds.size() > 100) {
		lastRobotRotSpeeds.pop_front();

		unsigned int inversioni = StrategyMath::misuraInversioni(lastRobotRotSpeeds,ampiezza_max);

		std::cerr<<"inversioni: "<<inversioni<<std::endl;
		if(inversioni > 5) {
			//brain->freeze(5000);

		}

	}

}
void  Strategy::strategyLoop(const ros::TimerEvent&)
{
	analyzeRobot();

	analyzeBrianState();

	analyzeUser();

	applyStrategy();

	printDebugInfo();

}
