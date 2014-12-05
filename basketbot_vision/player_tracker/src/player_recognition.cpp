#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <player_tracker/PosPrediction.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include "playerEKF.h"
#include "pcl_ros/point_cloud.h"
#include <limits>
#include <geometry_msgs/PoseStamped.h>
float threshold = 1.0;
unsigned int RATE = 30;
const unsigned int MAX_ID = 16;
const float USERLOST_TIMEOUT=2.0;

float applySlope(double input, double zero,double max)
{
	double l = (input - zero)/(max-zero);
	l = std::max<double>(l,0.0);
	l = std::min<double>(l,1.0);
	return l;

}


struct PlayerInfo {
	bool valid;
	bool oddIteration;
	float score;
	ros::Time lastUpdate;
	playerEKF playerFilter;
	std::vector<float> speeds;
	PlayerInfo():valid(false),score(-1.0) {};
};

class PlayerTracker
{
	ros::NodeHandle node;
	ros::Subscriber playerSubscriber2;

	ros::Subscriber faceSubscriber;
	ros::Subscriber odometrySubscriber;

	ros::Publisher predictionPublisher;
	ros::Publisher PosePredictionPublisher;

	tf::TransformBroadcaster transformBroadcaster;

	std::vector<PlayerInfo> potentialPlayers;
	void handlePoints(const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& msg,unsigned int offset);
	void playerPosCallback2(const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& msg);
	void facesCallback(const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& msg);
	void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void updateOdometry(const ros::TimerEvent&);
	float calculateSpeed(unsigned int playerID);
	float calculateFaceDistance(unsigned int playerID);
	float calculateScore(unsigned int playerID);
	void publishPlayerInfo(int player);
	int getBestPlayer();

	int currentPlayer;
	bool oddIteration;
	bool oddIterationF;
	void deleteOldUsers();
	float robotLinearSpeed;
	float robotAngularSpeed;

public:
	void spin();
	PlayerTracker();
};

void PlayerTracker::spin()
{
	ros::Timer timer = node.createTimer(ros::Duration(1.0/RATE), &PlayerTracker::updateOdometry,this);
	ros::Rate r(RATE);
	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}

void PlayerTracker::updateOdometry(const ros::TimerEvent&)
{
	deleteOldUsers();
	//update odometry
	for(unsigned int i = 0; i < potentialPlayers.size(); i++)
		if(potentialPlayers[i].valid)
		{	
			potentialPlayers[i].playerFilter.updateOdometry(robotLinearSpeed, robotAngularSpeed);
			potentialPlayers[i].score = calculateScore(i);
		
		
		}

	//publish player info
	int bestPlayer = getBestPlayer();
	if(bestPlayer>=0 && potentialPlayers[bestPlayer].score > threshold  ) {
		if(currentPlayer == -1 || potentialPlayers[bestPlayer].score > potentialPlayers[currentPlayer].score) {
			currentPlayer=bestPlayer;
			ROS_INFO("tracking player %d",currentPlayer);
		}
	}
	publishPlayerInfo(currentPlayer);
}

PlayerTracker::PlayerTracker()
{
	playerSubscriber2 = node.subscribe("/tracker/COMPoints", 2, &PlayerTracker::playerPosCallback2,this);
	faceSubscriber = node.subscribe("/tracker/FacesFiltered", 2, &PlayerTracker::facesCallback,this);
	odometrySubscriber = node.subscribe("/odom", 2, &PlayerTracker::odometryCallback,this);
	predictionPublisher = node.advertise<player_tracker::PosPrediction>("PosPrediction",10);
	PosePredictionPublisher = node.advertise<geometry_msgs::PoseStamped>("PosePrediction",10);

	currentPlayer = -1;
	oddIteration = false;
	potentialPlayers.resize(2*MAX_ID+2);

	robotLinearSpeed = robotAngularSpeed = 0.0;
}

void PlayerTracker::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	robotLinearSpeed = msg->twist.twist.linear.x;
	robotAngularSpeed = msg->twist.twist.angular.z;

}

float PlayerTracker::calculateSpeed(unsigned int playerID)
{
	PlayerInfo &it = potentialPlayers[playerID];
	std::vector<float> res = it.playerFilter.getStatus();
	if(sqrt(res[4] + res[5])>=4.8)
		return 0;
	float speed = sqrt(res[2]*res[2] + res[3]*res[3]);
	return speed;
}



float PlayerTracker::calculateScore(unsigned int playerID)
{

	float speed = calculateSpeed(playerID);

	float speedScore = 1.0 * applySlope(speed,0.2,0.3);
	if(playerID <= MAX_ID)
		 speedScore += 0.5;
	
	return speedScore;
		



}
int PlayerTracker::getBestPlayer()
{
	int ret = -1;
	float bestScore=0;
	for(unsigned int i = 0; i < potentialPlayers.size(); i++) {
		PlayerInfo &it = potentialPlayers[i];
		if(it.score > bestScore && it.valid && (i<=MAX_ID || ret == -1)) {
			ret = i;
			bestScore = it.score;
		}
	}
	return ret;

}
void PlayerTracker::deleteOldUsers()
{
	ros::Time now = ros::Time::now();
	for(unsigned int i = 0; i < potentialPlayers.size(); i++)
		if(potentialPlayers[i].valid && (now - potentialPlayers[i].lastUpdate).toSec() > USERLOST_TIMEOUT)
		{
			if(currentPlayer == i)
				currentPlayer = -1;
			potentialPlayers[i].valid = false;
			
			
			
			
		}
}
void PlayerTracker::handlePoints(const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& msg,unsigned int offset)
{
	ros::Time now = ros::Time::now();
	for(pcl::PointCloud<pcl::PointXYZL>::const_iterator it= msg->points.begin(); it!= msg->points.end(); ++it) {
		int id = it->label+offset;
		if(!potentialPlayers[id].valid) {
			std::cout <<"added a new player"<<std::endl;
			playerEKF e;
			std::swap(potentialPlayers[id].playerFilter, e);
			potentialPlayers[id].valid = true;
		}
		float distanza = sqrt(it->x * it->x +it->z*it->z);
		float angolo = atan2(it->x,it->z);
		if(distanza > 0.1)
			potentialPlayers[id].playerFilter.updatePlayerPos(distanza,angolo);
		potentialPlayers[id].lastUpdate = now;
		
	}
	
	//TODO continua
	
	
}

void PlayerTracker::facesCallback(const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& msg)
{
	handlePoints(msg,MAX_ID+1);
	return;
	
}
void PlayerTracker::playerPosCallback2(const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& msg)
{
	handlePoints(msg,0);
	return;
	

}



void PlayerTracker::publishPlayerInfo(int player)
{

	player_tracker::PosPrediction pred;


	if(player > -1) {
		PlayerInfo  &it = potentialPlayers[currentPlayer];
		std::vector<float> res = it.playerFilter.getStatus();
		tf::Transform tr;
		tr.setRotation( tf::Quaternion(0,0 , 0, 1) );
		tr.setOrigin(tf::Vector3(res[0],res[1],0.0));
		transformBroadcaster.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "base_link_respondable", "bill_filtered"));
		tr.setOrigin(tf::Vector3(res[2]*2.0,res[3]*2.0,0.0));
		transformBroadcaster.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "bill_filtered", "bill_filtered_vel"));
		pred.userId = player;
		pred.position.x = res[0];
		pred.position.y = res[1];
		pred.velocity.x = res[2];
		pred.velocity.y = res[3];
		pred.unreliability = sqrt(res[4] + res[5]);

		geometry_msgs::PoseStamped playerPose;
		playerPose.header.stamp = ros::Time::now();
		playerPose.header.frame_id =  "base_footprint";
		playerPose.pose.position.x = res[0];
		playerPose.pose.position.y = res[1];
		playerPose.pose.position.z = 0;

		float angle = atan2(res[3],res[2]);
		tf::Quaternion orientation(tf::Vector3(0,0,1),angle);
		geometry_msgs::Quaternion odom_quat;
		tf::quaternionTFToMsg(orientation,odom_quat);
		playerPose.pose.orientation = odom_quat;
		PosePredictionPublisher.publish(playerPose);
	} else {
		pred.userId = -1;
		pred.position.x = 0;
		pred.position.y = 0;
		pred.velocity.x = 0;
		pred.velocity.y = 0;
		pred.unreliability = std::numeric_limits<double>::max();

	}
	//std::cout <<"unrel: "<<pred.unreliability<<" "<<res[4]<<" "<<res[5]<<std::endl;
	predictionPublisher.publish(pred);


}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "player_tracker");
	PlayerTracker pt;
	pt.spin();
	return 0;
}
