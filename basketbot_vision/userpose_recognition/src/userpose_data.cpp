#include "userpose_data.h"
#include <boost/math/constants/constants.hpp>
#include <stdlib.h> //rand()
std::string UserPoseData::interestingTransforms[] = {"torso_"/* parto qui*/,"right_shoulder_","right_elbow_","left_shoulder_","left_elbow_","left_hand_","right_hand_"};
int UserPoseData::interestingTransformsSize = sizeof(interestingTransforms )/sizeof(interestingTransforms[0] );

std::vector<tf::Transform> UserPoseData::readTransforms()
{
	std::vector<tf::Transform> result = transforms;
	
	if(transforms.size()<6)
		return result;
		
	result[1] = transforms[0] * transforms[1]; //right elbow
	result[3] = transforms[2] * transforms[3]; //left elbow
	result[4] = result[3] * transforms[4];
	result[5] = result[1] * transforms[5]; //right hand
	return result;

}
std::vector<float> UserPoseData::extractAngles(tf::Quaternion q,const char* order)
{
	tf::Vector3 v;
	tf::Matrix3x3 m(q);
	std::vector<float> result;

	for(int i = 0; order[i]!= 0 && i<3; i++) {
		unsigned int pos = order[i] - 'x';
		tf::Vector3 res = m.getColumn((pos+2)%3);
		res.m_floats[pos]=0;
		tf::Vector3 asseBase(0,0,0);
		asseBase.m_floats[(pos+2)%3]=1;
		tf::Vector3 asseInv(0,0,0);
		asseInv.m_floats[pos]=1;

		float angle = tf::tfAngle(asseBase,res);
		if(res.m_floats[(pos+1)%3]>0) angle*= -1.0;
		m=tf::Matrix3x3(tf::Quaternion(asseInv,-angle))*m;
		std::cout <<std::fixed<<"angle "<<order[i]<<" "<<angle* 180.0 /boost::math::constants::pi<float>()<<std::endl;
		result.push_back(angle);
	}

	return result;
}

UserPoseData::UserPoseData()
{

}
float UserPoseData::operator-(UserPoseData & altro)
{

	if(transforms.size()!= altro.transforms.size())
		return std::numeric_limits<float>::infinity();
		if(transforms.size()<4)
			return std::numeric_limits<float>::infinity();
	float distance = 0;

	for(int i = 0; i<4; ++i) {
		float distanceInc = transforms[i].getRotation().angleShortestPath(altro.transforms[i].getRotation());
		
		distance += distanceInc;

	}
	std::cerr<<std::fixed<<"dist: "<<distance<<std::endl;
	return distance;
}
tf::StampedTransform & UserPoseData::extractTransformByName(std::vector<tf::StampedTransform> & transforms,std::string name)
{

	for(int i=0; i<transforms.size(); i++) {
		int comp = transforms[i].child_frame_id_.compare(0,name.length(),name);
		std::cerr<<transforms[i].child_frame_id_<<"#"<<name.length()<<" ";

		if(!comp)
			return transforms[i];
	}
	std::cerr<<name<<std::endl;
	exit(1);
}

void gen_random(char *s, const int len) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i = 0; i < len; ++i) {
        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }

    s[len] = 0;
}
std::string UserPoseData::readName()
{
	return name;
}
UserPoseData::UserPoseData(std::vector<tf::StampedTransform> & transforms_,std::string name)
{
	
	if(name == "")
	{
		char buffer[10];
		gen_random(buffer,8);
		name=std::string(buffer);
	}
	this->name = name;
	
	//ci sarebbe da correggere i valori
	transforms.clear();
	transforms.push_back(transforms_[0]);
	transforms.push_back(transforms_[0].inverse()*transforms_[1]);
	transforms.push_back(transforms_[2]);
	transforms.push_back(transforms_[2].inverse()*transforms_[3]);
	transforms.push_back(transforms_[3].inverse()*transforms_[4]);
	transforms.push_back(transforms_[1].inverse()*transforms_[5]);
	
	std::vector<float>  gomito_dx = extractAngles(transforms[3].getRotation(),"yzx");
	std::cout <<"gomito: " << gomito_dx[0] <<" "<<gomito_dx[1]<<" "<<gomito_dx[2]<<std::endl;
	

}
