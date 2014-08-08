#include <openni_tracker/COMList.h>
#include <tf/transform_broadcaster.h>

#include <ros/ros.h>

struct PlayerInfo
{
	float X;
	float Y;
	float Z;
	bool valid;
	bool oddIteration;
	unsigned int score;
	PlayerInfo():valid(false),score(0){};
};

class PlayerTracker
{
	ros::NodeHandle node;
	ros::Subscriber playerSubscriber;
	ros::Subscriber ballSubscriber;
	tf::TransformBroadcaster transformBroadcaster;
	void playerPosCallback(const openni_tracker::COMList::ConstPtr& msg);
	
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
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
}
PlayerTracker::PlayerTracker()
{
	playerSubscriber = node.subscribe("COMList", 2, &PlayerTracker::playerPosCallback,this);
	currentPlayer = -1;
	oddIteration = false;
	
}

void PlayerTracker::playerPosCallback(const openni_tracker::COMList::ConstPtr& msg)
{
	oddIteration = !oddIteration;
	std::vector<openni_tracker::COMData>::const_iterator it = msg->list.begin();
	for(;it!= msg->list.end();++it)
	{
		int id = it->id;
		
		//added a new player?
		
		potentialPlayers[id].X = it->x;
		potentialPlayers[id].Y = it->y;
		potentialPlayers[id].Z = it->z;
		potentialPlayers[id].valid = true;
		potentialPlayers[id].oddIteration = oddIteration;
		
		
		//calculate reliability
	}
	for(std::vector<PlayerInfo>::iterator it = potentialPlayers.begin();it != potentialPlayers.end();++it)
	{
		if(it->oddIteration != oddIteration)
			it->valid = false;
			
		//was valid?
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "player_tracker");
	PlayerTracker pt;
	pt.spin();
	return 0;
}