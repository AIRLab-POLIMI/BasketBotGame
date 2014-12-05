#include <ros/ros.h>
#include "basketbot_brain.h"
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>  

Strategy::Strategy(BasketBotBrain* brain) :brain(brain)
{
	goalPublisher = nh.advertise<geometry_msgs::PoseStamped >("/brain/goal",10);
	predictionSubscriber = nh.subscribe("PosPrediction", 2, &Strategy::predictionCallback,this);

	timer = nh.createTimer(ros::Duration(1.0/10.0), &Strategy::timerCallback,this);
	srand(time(NULL));
}

void Strategy::setState(StrategyState newState)
{
	if(strategyState == newState)
		return;

	StrategyState oldState = strategyState;
	strategyState = newState;
	if(oldState == STAY_AWAY) {
		brain->setParameter("distanceOffset",1.0);
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
		brain->setParameter("distanceOffset",3.0);
	}
	lastUpdate = ros::Time::now();
}
bool Strategy::isPlayerSlow()
{
	return playerSlow;
	if(lastPlayerPositions.size() < 10)
		return false;
	if((ros::Time::now() - lastPlayerPos.header.stamp).toSec() > 0.5)
		return false;
	if((lastPlayerPos.header.stamp - lastPlayerPositions.front().header.stamp).toSec()<=1.0)
		return false;



	geometry_msgs::PointStamped previousPos;
	previousPos.header.stamp = ros::Time::now();
	if((lastPlayerPos.header.stamp - lastPlayerPositions.front().header.stamp).toSec()>1.0) {


		while((lastPlayerPos.header.stamp - lastPlayerPositions.front().header.stamp).toSec()>1.0) {
			previousPos = lastPlayerPositions.front();
			lastPlayerPositions.pop_front();
		}


	}


	//float playerSpeed = sqrt(msg->velocity.x*msg->velocity.x + msg->velocity.y+msg->velocity.y);
	float timeBetweenPositions = (lastPlayerPos.header.stamp - previousPos.header.stamp).toSec();
	if(timeBetweenPositions<0.5 || timeBetweenPositions >1.5)
		return false;



	float distanceSquared = (lastPlayerPos.point.x-previousPos.point.x)*(lastPlayerPos.point.x-previousPos.point.x)
	                        +(lastPlayerPos.point.y-previousPos.point.y)*(lastPlayerPos.point.y-previousPos.point.y);
	float playerSpeed = sqrt(distanceSquared)/timeBetweenPositions;
	bool playerSlow = playerSpeed < 0.8;
	return playerSlow;

}
void Strategy::predictionCallback(const player_tracker::PosPrediction::ConstPtr& msg)
{
	std::cerr<<"strategy prediction"<<std::endl;
	if(msg->unreliability>8.0)
		return;
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
		
	float speedSquared = msg->velocity.x*msg->velocity.x + msg->velocity.y*msg->velocity.y;
		float speed = sqrt(speedSquared);
	std::cerr<<"speed: "<<speed<<std::endl;
	playerSlow = speed < 0.4;
	//pulizia e tiro fuori informazioni. funzione brutta, da rifare


	//bool playerSlow = playerSpeed < 2.0;
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
void  Strategy::timerCallback(const ros::TimerEvent&)
{
	bool slow = isPlayerSlow();
	std::cerr<<"slow: "<<slow<<std::endl;
	BrainState newState = brain->getState();
	if(newState != brainState)
		switch(newState) {
		case NORMAL:
			setState(NONE);
			break;

		case EXPLORE:
			setState(LAST_POSITION);

			break;

		default:
			setState(NONE);
			break;
		}


	brainState = newState;


	switch(strategyState) {
	case NONE:
		if( elapsedFromStrategyChange()> 3.0 && slow)
		{
			
			if(rand()%2)
			setState(TILT_LEFT);
			else
				setState(STAY_AWAY);
		}
			
		break;
		case STAY_AWAY:
		if( elapsedFromStrategyChange()> 0.5 && !slow)
		{
			setState(NONE);
		}
		else if(elapsedFromStrategyChange()>4.0)
			setState(NONE);
		break;
	case TILT_LEFT:
		if( elapsedFromStrategyChange()> 0.5 && !slow)
			setState(NONE);
		else if(elapsedFromStrategyChange()>2.0 && slow)
			setState(TILT_RIGHT);
		break;
	case TILT_RIGHT:
		if( elapsedFromStrategyChange()> 0.5 && !slow)
			setState(NONE);
		else if(elapsedFromStrategyChange()>2.0 && slow)
			setState(TILT_LEFT);
		break;

	case LAST_POSITION:
		if((ros::Time::now() - lastPlayerPos.header.stamp).toSec() < 10.0) {
				publishGoalAbsolute(lastPlayerPos.point.x,lastPlayerPos.point.y);

			}
		else if(elapsedFromStrategyChange() > 1.0)
			setState(RANDOM);
		
	case RANDOM:
	
		if(elapsedFromStrategyChange() > 1.0)
		{
			float rand = (ros::Time::now() - lastPlayerPos.header.stamp).toSec();
			
			publishGoalAbsolute(lastPlayerPos.point.x+2.0*sin(rand),lastPlayerPos.point.y+2.0*cos(rand));
			setState(LAST_POSITION);
		}
		

	}
	std::cerr<<"SS: "<<strategyState<<std::endl;
	/*	if(brainState ==EXPLORE)
			if((ros::Time::now() - lastPlayerPos.header.stamp).toSec() < 10.0) {
				publishGoalAbsolute(lastPlayerPos.point.x,lastPlayerPos.point.y);

			}

	}
	 */
}
