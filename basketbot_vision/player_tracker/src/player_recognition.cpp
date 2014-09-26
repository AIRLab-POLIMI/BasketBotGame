#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <player_tracker/PosPrediction.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include "playerEKF.h"
#include "pcl_ros/point_cloud.h"
#include <limits>

float threshold = 1.0;
unsigned int RATE = 30;

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

	tf::TransformBroadcaster transformBroadcaster;

	std::vector<PlayerInfo> potentialPlayers;
	std::vector<geometry_msgs::Point32> faces;
	ros::Time lastFaceUpdate;

	void playerPosCallback2(const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& msg);

	void facePosCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
	void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

	float calculateSpeed(unsigned int playerID);
	float calculateFaceDistance(unsigned int playerID);
	float calculateScore(unsigned int playerID);
	void publishPlayerInfo(int player);
	int getBestPlayer();

	int currentPlayer;
	bool oddIteration;

	float robotLinearSpeed;
	float robotAngularSpeed;
	ros::Time lastUpdate;
	double elapsedTime;
public:
	void spin();
	PlayerTracker();
};

void PlayerTracker::spin()
{
	ros::Rate r(RATE);
	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}


PlayerTracker::PlayerTracker()
{
	playerSubscriber2 = node.subscribe("COMPoints", 2, &PlayerTracker::playerPosCallback2,this);
	faceSubscriber = node.subscribe("/face_detector/faces_cloud", 2, &PlayerTracker::facePosCallback,this);
	odometrySubscriber = node.subscribe("/odom", 2, &PlayerTracker::odometryCallback,this);
	predictionPublisher = node.advertise<player_tracker::PosPrediction>("PosPrediction",10);

	currentPlayer = -1;
	oddIteration = false;
	potentialPlayers.resize(16);

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
	if(sqrt(res[4] + res[5])>=1.8)
		return 0;
	float speed = sqrt(res[2]*res[2] + res[3]*res[3]);
	return speed;
}
float PlayerTracker::calculateFaceDistance(unsigned int playerID)
{
	std::vector<float> res = potentialPlayers[playerID].playerFilter.getStatus();

	float distance = std::numeric_limits<float>::infinity();
	if(ros::Time::now()-lastFaceUpdate > ros::Duration(0.3))
		return distance;


	float currentX = res[0];
	float currentY = res[1];



	for(std::vector<geometry_msgs::Point32>::const_iterator it = faces.begin(); it!= faces.end(); it++) {

		float dx = it->z - res[0];
		float dy = -it->x - res[1];

		float d = sqrt(dx*dx + dy*dy);
		ROS_INFO("dati:  %f %f   %f %f ",it->z , res[0],-it->x , res[1]);
		distance = std::min(d,distance);
	}
	return distance;


}

float PlayerTracker::calculateScore(unsigned int playerID)
{

	float speed = calculateSpeed(playerID);
	float faceDistance = calculateFaceDistance(playerID);

	float speedScore = 0.75 * applySlope(speed,0.2,0.4);
	float faceScore = 0.75 * applySlope(faceDistance,0.4,0.2);

	ROS_INFO("giocatore %d, vel %f (%f),faccia %f(%f)",playerID,speed,speedScore,faceDistance,faceScore);
	return speedScore + faceScore;


}
int PlayerTracker::getBestPlayer()
{
	int ret = -1;
	float bestScore=0;
	for(unsigned int i = 0; i < potentialPlayers.size(); i++) {
		PlayerInfo &it = potentialPlayers[i];
		if(it.score > bestScore) {
			ret = i;
			bestScore = it.score;
		}
	}
	return ret;

}

void PlayerTracker::facePosCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	const std::vector<geometry_msgs::Point32> &points = msg->points;
	faces = points;
	lastFaceUpdate = ros::Time::now();
	for(std::vector<geometry_msgs::Point32>::const_iterator it = points.begin(); it!= points.end(); it++) {
		ROS_INFO("%f %f %f",it->x,it->y,it->z);

	}
}

void PlayerTracker::playerPosCallback2(const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& msg)
{
	ros::Duration d = ros::Time::now()-lastUpdate;
	elapsedTime = d.toSec();
	lastUpdate = ros::Time::now();

	oddIteration = !oddIteration;

	for(pcl::PointCloud<pcl::PointXYZL>::const_iterator it= msg->points.begin(); it!= msg->points.end(); ++it) {
		int id = it->label;
		ROS_INFO("poscall: %d   %f %f %f",id,it->x,it->y,it->z);
		if(!potentialPlayers[id].valid) {
			std::cout <<"added a new player"<<std::endl;
			playerEKF e;
			std::swap(potentialPlayers[id].playerFilter, e);
			potentialPlayers[id].valid = true;
			potentialPlayers[id].score = -1;
		}

		potentialPlayers[id].playerFilter.updateOdometry(robotLinearSpeed, robotAngularSpeed);

		float distanza = sqrt(it->x * it->x +it->z*it->z);
		float angolo = atan2(-it->x,it->z);
		if(distanza > 0.1)
			potentialPlayers[id].playerFilter.updatePlayerPos(distanza,angolo);


		potentialPlayers[id].score = calculateScore(id);


		potentialPlayers[id].oddIteration = oddIteration;
	}
	for(unsigned int i = 0; i < potentialPlayers.size(); i++) {
		PlayerInfo &it = potentialPlayers[i];
		if(it.oddIteration != oddIteration && it.valid) {
			it.valid = false;
			if(i == currentPlayer)
				currentPlayer = -1;
			std::cout << "lost a player"<<std::endl;
		}
	}

	int bestPlayer = getBestPlayer();
	if(bestPlayer>=0 && potentialPlayers[bestPlayer].score > threshold  ) {
		if(currentPlayer == -1 || potentialPlayers[bestPlayer].score > potentialPlayers[currentPlayer].score) {
			currentPlayer=bestPlayer;
			ROS_INFO("tracking player %d",currentPlayer);
		}
	}


	
		publishPlayerInfo(currentPlayer);
}



void PlayerTracker::publishPlayerInfo(int player)
{
	
	player_tracker::PosPrediction pred;


	if(player > -1)
	{
		PlayerInfo  &it = potentialPlayers[currentPlayer];
	std::vector<float> res = it.playerFilter.getStatus();
	tf::Transform tr;
	tr.setRotation( tf::Quaternion(0,0 , 0, 1) );
	tr.setOrigin(tf::Vector3(res[0],res[1],0.0));
	transformBroadcaster.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "base_link_respondable", "bill_filtered"));
	tr.setOrigin(tf::Vector3(res[2]*2.0,res[3]*2.0,0.0));
	transformBroadcaster.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "bill_filtered", "bill_filtered_vel"));
	pred.position.x = res[0];
	pred.position.y = res[1];
	pred.velocity.x = res[2];
	pred.velocity.y = res[3];
	pred.unreliability = sqrt(res[4] + res[5]);
	}
	else
	{
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
