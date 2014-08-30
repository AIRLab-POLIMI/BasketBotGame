#include <openni_tracker/COMList.h>
#include <tf/transform_broadcaster.h>

#include <ros/ros.h>

struct PlayerInfo {
	float X;
	float Y;
	float Z;
	bool valid;
	bool oddIteration;
	unsigned int score;
	PlayerInfo():valid(false),score(0) {};
};

class PlayerTracker
{
	ros::NodeHandle node;
	ros::Subscriber playerSubscriber;
	ros::Subscriber ballSubscriber;
	tf::TransformBroadcaster transformBroadcaster;
	void playerPosCallback(const openni_tracker::COMList::ConstPtr& msg);
	void ballPosCallback(const openni_tracker::COMList::ConstPtr& msg);

	std::vector<PlayerInfo> potentialPlayers;
	int currentPlayer;
	bool oddIteration;
public:
	void spin();
	PlayerTracker();
};

void PlayerTracker::spin()
{
	ros::Rate r(30);
	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}
PlayerTracker::PlayerTracker()
{
	playerSubscriber = node.subscribe("COMList", 2, &PlayerTracker::playerPosCallback,this);
	ballSubscriber = node.subscribe("ballsCOMList", 2, &PlayerTracker::playerPosCallback,this);

	currentPlayer = -1;
	oddIteration = false;
	potentialPlayers.resize(16);

}

void PlayerTracker::playerPosCallback(const openni_tracker::COMList::ConstPtr& msg)
{
	oddIteration = !oddIteration;
	std::vector<openni_tracker::COMData>::const_iterator it = msg->list.begin();
	for(; it!= msg->list.end(); ++it) {
		int id = it->id;

		if(!potentialPlayers[id].valid) {
			std::cout <<"added a new player"<<std::endl;

			//added a new player?

		}


		potentialPlayers[id].X = it->x;
		potentialPlayers[id].Y = it->y;
		potentialPlayers[id].Z = it->z;
		potentialPlayers[id].valid = true;
		potentialPlayers[id].oddIteration = oddIteration;


		//calculate reliability
	}

	for(unsigned int i = 0; i < potentialPlayers.size(); i++) {
		PlayerInfo &it = potentialPlayers[i];
		if(it.oddIteration != oddIteration && it.valid) {
			it.valid = false;
			if(i == currentPlayer)
				currentPlayer = -1;
			std::cout << "lost a player"<<std::endl;
		}

		//was valid?
	}

	if(currentPlayer == -1) {
		for(unsigned int i = 0; i < potentialPlayers.size(); i++) {
			PlayerInfo &it = potentialPlayers[i];
			if(it.valid) {
				currentPlayer = i;
				break;
			}
		}
	}
	if(currentPlayer != -1) {
		PlayerInfo  &it = potentialPlayers[currentPlayer];
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(it.Z,-it.X,-it.Y));
		transform.setRotation(tf::Quaternion(0,0,0,1));
		if(it.X != 0)
			//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link_respondable", "Bill_base"));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link_respondable", "Bill_base"));

	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "player_tracker");
	PlayerTracker pt;
	pt.spin();
	return 0;
}
