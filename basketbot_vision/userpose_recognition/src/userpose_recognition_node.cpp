#include <iostream>
#include <sstream>
#include <boost/math/constants/constants.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include "UserPoseDisplay.h"
#include "userpose_data.h"
#include <fstream>
#include "userpose_recognition.h"

namespace std
{
string operator<<(string s,int n)
{
	stringstream ss;
	ss<<s<<n;
	return ss.str();
}
}

bool UserposeRecognition::loadData(std::string file)
{
	try {

		YAML::Node config = YAML::LoadFile(file);
		trainingSet = config.as<std::vector<UserPoseData> >();

	} catch(...) {
		return false;
	}
	return true;
}
bool UserposeRecognition::saveData(std::string file)
{
	YAML::Node node;
	node = trainingSet;
	std::ofstream fout(file.c_str());
	fout << node;
	return true;
}


int UserposeRecognition::findNearest(UserPoseData & data,float *retDist)
{
	if(trainingSet.size() == 0)
		return -1;
	int ret = 0;
	float dist = data-trainingSet[0];
	for(int i = 1; i<trainingSet.size(); i++) {
		float d = data-trainingSet[i];
		if(d<dist) {
			dist = d;
			ret = i;
		}
	}
	if(retDist)
		*retDist = dist;
	return ret;

}

void printQuaternion(tf::Quaternion& q)
{
	std::cout << q[0]<<"\t"<<q[1]<<"\t"<<q[2]<<"\t"<<q[3]<<std::endl;
}

void UserposeRecognition::radiansToDegrees(std::vector<float> & v)
{
	for(int i=0; i<v.size(); i++)
		v[i] = v[i] * 180.0 /boost::math::constants::pi<float>();
}

UserposeRecognition::Result & UserposeRecognition::readResult()
{
	std::cerr << "migliore: "<<result.best
	          <<"  distanza: "<<result.distance
	          <<"  confidenza: "<<result.confidence
	          <<std::endl;
	return result;
}

void UserposeRecognition::analyzeData(unsigned int user,UserPoseData& datiUtente)
{

	result.best =findNearest(datiUtente,&result.distance);

	float confidence = 0.0;
	for(std::list<Result>::iterator it = lastResults.begin(); it!= lastResults.end(); it++)
		if(it->best == result.best)
			confidence +=1/ (it->distance);
	confidence += 1/result.distance;
	
	confidence = std::min(confidence/10.0,1.0);
	result.confidence = confidence;

	lastResults.push_back(result);
	if(lastResults.size()>10)
		lastResults.pop_front();
	
//	userPoseDisplay.setCurrent(distanza,migliore);
	/*std::cerr<<std::fixed<<"distanza: "<<currDistance<<std::endl;
	std::cerr <<"il migliore: "<<currBest<<std::endl;*/


}

void UserposeRecognition::saveUserData(std::string,UserPoseData & datiUtente)
{
	trainingSet.push_back(datiUtente);
}
std::vector<UserPoseData> & UserposeRecognition::getTrainingSet()
{
	return trainingSet;
}

void UserposeRecognition::removeUserData(int index)
{
	if(index >= 0 && index < trainingSet.size())
		trainingSet.erase(trainingSet.begin()+index);
}
UserposeRecognition::UserposeRecognition()
{


}
