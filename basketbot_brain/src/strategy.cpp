#include <ros/ros.h>
#include "basketbot_brain.h"
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include "RosBrianBridge.h"
float defaultDistanceOffset = 1.5;
float defaultDistanceSensitivity = 0.5 ;
float defaultObstaclesRadarDistance = 1.0;

float minSlowThresh = 0.4;
float maxSlowThresh = 0.5;
float minDistErrorThresh = 0.4;
float maxDistErrorThresh = 0.5;
Strategy::Strategy(BasketBotBrain* brain,RosBrianBridge* bridge) :brain(brain),bridge(bridge)
{
	goalPublisher = nh.advertise<geometry_msgs::PoseStamped >("/brain/goal",10);
	predictionSubscriber = nh.subscribe("PosPrediction", 2, &Strategy::predictionCallback,this);

	timer = nh.createTimer(ros::Duration(1.0/10.0), &Strategy::strategyLoop,this);
	srand(time(NULL));
	brain->setParameter("distanceOffset",defaultDistanceOffset);
	brain->setParameter("distanceSensitivity",defaultDistanceSensitivity);
	brain->setParameter("obstaclesRadarDistance",defaultObstaclesRadarDistance);
	userSeenAtLeastOnce = false;
	brainState =brain->getState();
	setStrategyState(NONE);
}

void Strategy::setStrategyState(StrategyState newState)
{
	if(strategyState == newState)
		return;

	StrategyState oldState = strategyState;
	strategyState = newState;
	if(oldState == STAY_AWAY) {
		brain->setParameter("distanceOffset",defaultDistanceOffset);
	}
	if(oldState == TILT_LEFT || oldState == TILT_RIGHT) {
		brain->setParameter("orientationOffset",0);
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
	lastUpdate = ros::Time::now();
}
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
	BrainState newState = brain->getState();
	if(newState != brainState) {
		previousBrainState = brainState;
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
			processEvent("freeze",1.0);


			break;

		default:
			setStrategyState(NONE);
			break;
		}
	}
	if(brain->dangerCollision())
		processEvent("danger_collision",1.0);



}
float Strategy::calcAverage(std::list<float> & lista)
{
	if(lista.size() == 0)
		return 0;
	float media = 0;

	for(std::list<float>::iterator it = lista.begin(); it!=lista.end(); ++it) {
		float val = *it;
		media += val;

	}
	media /=lista.size();
	return media;
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
	} else if(eventName == "user_position") {
		if(userJustAppeared)

			return true;
		
		else
			return false;
	} else {
		ROS_FATAL_STREAM("unrecognized event");
		exit(0);
	}


	return false;

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
		ROS_INFO_STREAM("EVENT: "<<eventName);
	} else if(eventName == "user_lost") {
		ROS_INFO_STREAM("EVENT: "<<eventName<<"\t"<<eventSize<<"  "<<lastPrediction.position.y);
		
	} else if(eventName == "freeze") {
		ROS_INFO_STREAM("robot frozen");
	} else if(eventName == "danger_collision") {

		float obsRange = brain->getParameter("obstaclesRadarDistance");
		if(obsRange < defaultObstaclesRadarDistance*2.0) {
			brain->setParameter("obstaclesRadarDistance",obsRange*1.1);
			ROS_INFO_STREAM("Increased radar range to: "<<obsRange*1.1);
		}

		ROS_INFO_STREAM("EVENT: "<<eventName<<"\t"<<eventSize);
	}

}
void  Strategy::analyzeUser()
{
	if(brainState == NORMAL) {
		float vx =bridge->getPlayerVelocityX();
		float vy = bridge->getPlayerVelocityY();
		float speed = sqrt(vx*vx+vy*vy);
		float distance = bridge->getPlayerDistance();
		lastDistances.push_back(distance);
		lastSpeeds.push_back(speed);

		if(lastDistances.size() > 100) {
			lastDistances.pop_front();
			float media = 0;

			for(std::list<float>::iterator it = lastDistances.begin(); it!=lastDistances.end(); ++it) {
				float val = fabs(*it - defaultDistanceOffset);
				media += val;

			}
			media /=lastDistances.size();
			processEvent("distance_error",media);


		}
		if(lastSpeeds.size() > 20) {
			lastSpeeds.pop_front();
			float avg_speed = calcAverage(lastSpeeds);

			processEvent("speed",speed);

			//la varianza e' data da matrice P

		}



	}


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
	case NONE:
		if( elapsedFromStrategyChange()> 3.0 && slow) {
			ROS_INFO_STREAM("Player is bored? let's try something...");
			if(rand()%3)//due poss. su tre che si allontani
				setStrategyState(STAY_AWAY);
			else
				setStrategyState(TILT_LEFT);
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

	case LAST_POSITION:
		if((ros::Time::now() - lastPlayerPos.header.stamp).toSec() < 10.0) {
			publishGoalAbsolute(lastPlayerPos.point.x,lastPlayerPos.point.y);
			std::cerr<<"X: "<<lastPlayerPos.point.x<<"   Y:  "<<lastPlayerPos.point.y<<std::endl;
		} else if(elapsedFromStrategyChange() > 1.0)
			setStrategyState(RANDOM);
		break;
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

void Strategy::printDebugInfo()
{
	if(strategyState==STAY_AWAY)
		std::cerr<<"SS: STAY_AWAY"<<std::endl;
	else if(strategyState==TILT_LEFT)
		std::cerr<<"SS: TILT_LEFT"<<std::endl;
	else
		std::cerr<<"SS: "<<strategyState<<std::endl;
}

void  Strategy::strategyLoop(const ros::TimerEvent&)
{

	analyzeBrianState();

	analyzeUser();

	applyStrategy();

	printDebugInfo();

}
