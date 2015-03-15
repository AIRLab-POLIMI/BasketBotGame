#include <userpose_recognition.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "UserPoseDisplay.h"
#include <userpose_recognition/UserPose.h>
#include <player_tracker/PosPrediction.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
const unsigned int MAX_USERS=16;
const std::string userpose_config_path = ros::package::getPath("userpose_recognition") + "/config";
class UserposeRecognitionNode
{
	UserposeRecognition userposeRecognition;
	ros::Publisher userPosePublisher;
	ros::Subscriber posPredictionSubscriber;
	ros::NodeHandle nh;
	ros::NodeHandle pnh;
	image_transport::ImageTransport it;
	image_transport::Publisher debugPosePub;
	tf::TransformListener transformListener;
	bool isTransformAvailable(std::string a ,std::string b);
	void analyzeUser(int i);
	bool isUserAvailable(unsigned int id);
	std::vector<tf::StampedTransform> readUserTransforms(unsigned int id);
	UserPoseDisplay userPoseDisplay;
	static void keyCallback(void * ptr,int,int);
	bool save;
	int currentPlayer;
	void posPredictionCallback(const player_tracker::PosPrediction::ConstPtr& msg);
public:
	UserposeRecognitionNode();
	~UserposeRecognitionNode();
	void readFrames();

};

void UserposeRecognitionNode::keyCallback(void *ptr,int button, int index)
{
	UserposeRecognitionNode* that = (UserposeRecognitionNode*) ptr;
	std::cerr<<"keycallback"<<std::endl;

	if(button==2 ) {
		that->userposeRecognition.removeUserData(index);
		that->userPoseDisplay.loadData(that->userposeRecognition.getTrainingSet());
	}
	if(button==3) {
		std::cerr<<"userkeycallback"<<std::endl;
		that->save=true;

	}

}

bool UserposeRecognitionNode::isUserAvailable(unsigned int id)
{
	std::string intr[] = {"neck_","torso_","right_shoulder_","left_shoulder_","right_elbow_","left_elbow_"};
	int sizeIntr = sizeof(intr)/sizeof(intr[0]);


	if(!isTransformAvailable(std::string("neck_")<<id,std::string("torso_")<<id))
		return false;
	if(!isTransformAvailable(std::string("right_shoulder_")<<id,std::string("left_shoulder_")<<id))
		return false;
	if(!isTransformAvailable(std::string("right_elbow_")<<id,std::string("left_elbow_")<<id))
		return false;

	if(!isTransformAvailable(std::string("right_elbow_")<<id,std::string("left_shoulder_")<<id))
		return false;
	if(!isTransformAvailable(std::string("neck_")<<id,std::string("left_shoulder_")<<id))
		return false;

	return true;
}


UserposeRecognitionNode::UserposeRecognitionNode():nh(),pnh("~"),it(nh)
{
	save = false;
	userPoseDisplay.setMouseCallback(keyCallback,this);
	userposeRecognition.loadData(userpose_config_path + "/config.yaml");
	userPoseDisplay.loadData(userposeRecognition.getTrainingSet());
	userPosePublisher = nh.advertise<userpose_recognition::UserPose>("UserPose",10);
	currentPlayer=0;
	posPredictionSubscriber = nh.subscribe("PosPrediction", 2, &UserposeRecognitionNode::posPredictionCallback,this);
	debugPosePub = it.advertise("PoseDebug",1);
	bool nogui= false;
	ros::param::param<bool>("~nogui", nogui,false);
	if(!nogui)
	userPoseDisplay.init();

}
UserposeRecognitionNode::~UserposeRecognitionNode()
{
	userposeRecognition.saveData(userpose_config_path + "/config.yaml");
}

void UserposeRecognitionNode::readFrames()
{
	bool found = false;
	if(currentPlayer>0) {
		if(isUserAvailable(currentPlayer)) {
			analyzeUser(currentPlayer);
			found = true;
		}
	}
	if(currentPlayer==0)
		for(int i = 1; i<MAX_USERS; i++)
			if(isUserAvailable(i)) {
				std::cerr <<"utente disponibile: "<<i<<std::endl;
				analyzeUser(i);
				found = true;
			}
	userPoseDisplay.spinOnce();

	return;
}

void UserposeRecognitionNode::analyzeUser(int user)
{
	std::vector<tf::StampedTransform> transforms = readUserTransforms(user);
	UserPoseData datiUtente(transforms);
	userposeRecognition.analyzeData(user,datiUtente);
	int index;
	float distance;
	UserposeRecognition::Result result= userposeRecognition.readResult();
	index = result.best;
	distance = result.distance;
	if(result.confidence<0.5)
		index = -1;
	userPoseDisplay.setCurrent(distance,index);
	{
		cv::Mat immagine = userPoseDisplay.showTransforms(datiUtente);
		cv_bridge::CvImage out_msg;
		out_msg.image = immagine;
		out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
		debugPosePub.publish(out_msg.toImageMsg());
	}

	userpose_recognition::UserPose userPose;
	userPose.poseName = index>-1?userposeRecognition.getPoseName(index):"none";
	userPose.userId = user;
	userPose.poseId = index;
	userPose.confidence = result.confidence;
	userPose.distance = distance;
	userPosePublisher.publish(userPose);

	if(save) {
		save=false;
		userposeRecognition.saveUserData("nome",datiUtente);
		userPoseDisplay.loadData(userposeRecognition.getTrainingSet());
	}
}
bool UserposeRecognitionNode::isTransformAvailable(std::string a ,std::string b)
{
	tf::StampedTransform tr;

	try {
		transformListener.lookupTransform ( a,b,ros::Time(0),tr);
		if(ros::Time::now() - tr.stamp_  < ros::Duration(0.5))
			return true;

	} catch(...) {
		;
	}
	return false;
}
void UserposeRecognitionNode::posPredictionCallback(const player_tracker::PosPrediction::ConstPtr& msg)
{
	currentPlayer=msg->userId;
}
std::vector<tf::StampedTransform> UserposeRecognitionNode::readUserTransforms(unsigned int id)
{
	std::vector<tf::StampedTransform> transforms;
	tf::StampedTransform tmp;

	std::string baseFrame = UserPoseData::interestingTransforms[0]<<id;

	for(int i = 1; i<UserPoseData::interestingTransformsSize; i++) {
		transformListener.lookupTransform ( baseFrame,UserPoseData::interestingTransforms[i]<<id,ros::Time(0),tmp);
		transforms.push_back(tmp);
	}


	return transforms;
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "basketbot_pose_recognition");
	UserposeRecognitionNode u;
	ros::Rate r(20);
	while (ros::ok()) {

		ros::spinOnce();
		u.readFrames();
		r.sleep();
	}
	std::cerr<<"chiusura"<<std::endl;

}
