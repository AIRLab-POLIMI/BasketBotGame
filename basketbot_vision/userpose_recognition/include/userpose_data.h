#ifndef USERPOSE_DATA_H
#define USERPOSE_DATA_H
#include <vector>
#include <tf/transform_datatypes.h>
#include <yaml-cpp/yaml.h>
#include <string.h>



class UserPoseData
{
	friend class YAML::convert<UserPoseData>;
	std::vector<tf::Transform> transforms;
	std::string name;

	tf::StampedTransform & extractTransformByName(std::vector<tf::StampedTransform> &,std::string);
public:
	
	static std::string interestingTransforms[];
	static int interestingTransformsSize;
	static std::vector<float> extractAngles(tf::Quaternion q,const char* order);
	std::vector<tf::Transform> readTransforms();
	std::string readName();
	UserPoseData(std::vector<tf::StampedTransform> &,std::string name ="");
	UserPoseData();
	float operator-(UserPoseData &);
};

namespace YAML
{
template<>
struct convert<tf::Transform> {
	static Node encode(const tf::Transform& rhs) {
		Node node;
		tf::TransformData td;
		rhs.serialize(td);
		node = YAML::Binary((const unsigned char*) &td,sizeof(tf::TransformData));
		return node;
	}
	static bool decode(const Node& node, tf::Transform& rhs) {
		tf::TransformData td;
		Binary b = node.as<Binary>();
		if(sizeof(tf::TransformData) !=b.size())
		{
			std::cerr<<"non potevo caricare"<<std::endl;
			return false;
		}
		memcpy(&td,b.data(),b.size());
		rhs.deSerialize(td);
		return true;
	}
};
template<>
struct convert<UserPoseData> {
	static Node encode(const UserPoseData& rhs) {
		YAML::Node node;
		node["name"] = rhs.name;
		node["transforms"] = rhs.transforms;
			
		return node;
	}
	static bool decode(const Node& node, UserPoseData& rhs) {
		rhs.name = node["name"].as<std::string>();
		rhs.transforms = node["transforms"].as<std::vector<tf::Transform> >();			
		return true;
	}
};

}

#endif
