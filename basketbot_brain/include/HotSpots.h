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
	
	//Center of the map
	float center_x;
	float center_y;

	unsigned int pointPosition(int,int);
	float readPoint(float x,float y);
	std::pair<unsigned int,unsigned int> worldToMatrixPos(float x,float y);
	unsigned int matrixToArrayPos(std::pair<unsigned int,unsigned int> coords);
	void generateBestMatch();
	float getDistance(int,int,int,int);
	float getDistance(unsigned int,unsigned int);
	void debugPrint();
	float getScore(unsigned int,unsigned int);
	float matrixToArrayPos(unsigned int a ,unsigned int b);
	void moveMap(int X,int Y);
#ifdef DEBUG_HOTSPOTS_MAP
	ros::NodeHandle nh;
	ros::Publisher debugHotSpotsPublisher;
#endif 

   std::pair<float,float> worldToMatrixPos(unsigned int x,unsigned int y);
public:
	bool recordPoint(float x,float y,float value=1.0);
	HotSpots();
	float getConfidence();
	std::pair<float,float> getBestMatch();
	void initGrid(float width,float depth,float step);
    void setParameters(float hotSpotsRadius,float hotSpotsDecayRate,float hotSpotsConfidenceRadius);


};
