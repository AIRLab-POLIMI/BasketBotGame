#include<vector>

#define DEBUG_HOTSPOTS_MAP
#ifdef DEBUG_HOTSPOTS_MAP
#include <ros/ros.h>

#endif



class HotSpots
{
	float decayRate;
	float radius;
	float confRadius;
	std::vector<float> scores;
	unsigned int width;
	unsigned int depth;
	float step;

	float bestX;
	float bestY;
	float bestConfidence;


	unsigned int pointPosition(int,int);
	float readPoint(float x,float y);
	std::pair<unsigned int,unsigned int> worldToMatrixPos(float x,float y);
	unsigned int matrixToArrayPos(std::pair<unsigned int,unsigned int> coords);
	void generateBestMatch();
	float getDistance(int,int,int,int);
	void debugPrint();
	float getScore(unsigned int,unsigned int);
	float matrixToArrayPos(unsigned int a ,unsigned int b);

#ifdef DEBUG_HOTSPOTS_MAP
	ros::NodeHandle nh;
	ros::Publisher debugHotSpotsPublisher;
#endif 


public:
	bool recordPoint(float x,float y,float value=1.0);
	HotSpots();
	float getConfidence();
	std::pair<float,float> getBestMatch();
	void resetGrid(float width,float depth,float step);



};
