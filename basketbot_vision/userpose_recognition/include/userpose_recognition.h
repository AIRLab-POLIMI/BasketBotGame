#ifndef USERPOSE_RECOGNITION_H
#define USERPOSE_RECOGNITION_H
#include <string>
#include <sstream>
#include <vector>
#include <tf/transform_datatypes.h>
#include "UserPoseDisplay.h"
#include <list>

namespace std
{
string operator<<(string s,int n);

}


class UserposeRecognition
{
	public:
	struct Result
	{
		float distance;
		int best;
		float confidence;
	};
private:
	Result result;
	std::list<Result> lastResults;
	std::vector<UserPoseData> trainingSet;

	void radiansToDegrees(std::vector<float> &);

	int findNearest(UserPoseData &,float *);
	
public:
	UserposeRecognition();
	void readFrames();
	void analyzeData(unsigned int,UserPoseData& );
	bool loadData(std::string);
	bool saveData(std::string);
	std::vector<UserPoseData> & getTrainingSet();
	std::string getPoseName(int index);
	void	saveUserData(std::string,UserPoseData &);
	void removeUserData(int index);
	Result & readResult();
};
#endif
