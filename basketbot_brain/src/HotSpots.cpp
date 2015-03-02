#include "HotSpots.h"
#include <math.h>
#include <string>
#include <iostream>

#ifdef DEBUG_HOTSPOTS_MAP
#include <nav_msgs/OccupancyGrid.h>
#endif
void HotSpots::moveMap(int offsetX,int offsetY)
{
	std::cerr<<"move: "<<offsetX <<"  "<<offsetY<<std::endl;
	std::vector<float> oldScores = scores;
	for(int i = 0; i<width*depth; i++)
		scores[i] = 0;
		
	for(int y = std::max(offsetY,0); y < std::min(offsetY+depth,depth); y++)
		for(int x = std::max(offsetX,0); x < std::min(offsetX+width,width); x++) 	
		{
			unsigned int oldPos = matrixToArrayPos(x,y);
			unsigned int newPos = matrixToArrayPos(x-offsetX,y-offsetY);
			std::cerr<<"old: "<<oldPos<<"  new: "<<newPos<<std::endl;
			scores[newPos] = oldScores[oldPos];
		}
	center_x += offsetX*step;
	center_y += offsetY * step;
	if(center_x > 1000.0)
	exit(1);
}
std::pair<unsigned int,unsigned int> HotSpots::worldToMatrixPos(float x,float y)
{
	x-=center_x;
	y-= center_y;
	std::cerr<<(depth*step/2.0)<<std::endl;
	int coordX = floor((x+(width*step/2.0))/step);
	int coordY = floor((y+(depth*step/2.0))/step);

	std::cerr<<"World: "<<x<<";"<<y<<";X: " <<coordX<<" Y: "<<coordY<<std::endl;

	if(coordX < 0 || coordY < 0 || coordX>=width || coordY >= depth)
	{
		int offsetX =0,offsetY=0;
		if(coordX<0)
			offsetX = coordX;
		if(coordY<0)
			offsetY = coordY;
		if(coordX>=(int)width)
			offsetX = coordX-width + 1;
		if(coordY>=(int)depth)
			offsetY = coordY-depth + 1;
		moveMap(offsetX,offsetY);
		coordX -= offsetX;
		coordY -= offsetY;
		
	}
	return std::pair<unsigned int,unsigned int> (coordX,coordY);
}

std::pair<float,float> HotSpots::worldToMatrixPos(unsigned int x,unsigned int y)
{
	float ret_x = center_x  -0.5*width*step + x*step;
	float ret_y = center_y  -0.5*depth*step + y*step;
	return std::pair<float,float>(ret_x,ret_y);
}

float HotSpots::getDistance(unsigned int,unsigned int)
{
	//DAFARE, usare una funzione vera
	exit(1);
	
}

float HotSpots::getDistance(int X1,int Y1,int X2,int Y2)
{
	float deltaX = fabs(step*(X1-X2));
	float deltaY = fabs(step*(Y1-Y2));

	return sqrt(deltaX * deltaX + deltaY*deltaY);
}


void HotSpots::initGrid(float X, float Y, float step)
{
	this->step = step;
	width = ceil(X/step);
	depth = ceil(Y/step);
	std::cerr << "depth: "<<depth<<std::endl;
	scores.resize(width*depth);
	for(int i = 0; i<width*depth; i++)
		scores[i] = 0;
}

void HotSpots::setParameters(float hotSpotsRadius,float hotSpotsDecayRate,float hotSpotsConfidenceRadius)
{
	decayRate=hotSpotsDecayRate;
	radius = hotSpotsRadius;
	confRadius = hotSpotsConfidenceRadius;
}
HotSpots::HotSpots()
{
	
	decayRate=0.5;
	radius = 1.1;
	confRadius = 1.0;
center_x =0.0;
center_y = 0.0;
#ifdef DEBUG_HOTSPOTS_MAP
	debugHotSpotsPublisher = nh.advertise<nav_msgs::OccupancyGrid>("debugSpots", 2);
	//debugPrint();
#endif

}
void HotSpots::debugPrint()
{
	/*for(int i = 0; i<scores.size(); i++) {
		std::cerr <<std::fixed<<scores[i]<<" ";
		if(i % depth == depth-1)
			std::cerr<<std::endl;
	}*/
#ifdef DEBUG_HOTSPOTS_MAP
	nav_msgs::OccupancyGrid::Ptr debugMap(new nav_msgs::OccupancyGrid());
	debugMap->header.stamp = ros::Time::now();
	debugMap->header.frame_id="/odom";
	debugMap->info.map_load_time = debugMap->header.stamp;

	debugMap->info.resolution = step;
	debugMap->info.width = width;
	debugMap->info.height = depth;
	debugMap->info.origin.position.x = center_x -1.0* width*step/2.0;
	debugMap->info.origin.position.y = center_y -1.0 * depth*step/2.0;
	debugMap->info.origin.position.z = 0;

	debugMap->info.origin.orientation.x = 0;
	debugMap->info.origin.orientation.y = 0;
	debugMap->info.origin.orientation.z = 0;
	debugMap->info.origin.orientation.w = 1.0;
	debugMap->data.resize(width*depth);

	for(unsigned int i = 0; i<width*depth; ++i) {
		int val = floor(std::min(scores[i] * 100.0f,100.0f));
		debugMap->data[i] = val;
	}
	debugHotSpotsPublisher.publish(debugMap);
		ROS_INFO_STREAM("DebugPrint");

#endif

}

float HotSpots::readPoint(float x, float y)
{
}

unsigned int HotSpots::matrixToArrayPos(std::pair<unsigned int,unsigned int> coords)
{

	return coords.second*width +coords.first;
}

std::pair<float,float> HotSpots::getBestMatch()
{
	return std::pair<float,float>(bestX,bestY);
}

float HotSpots::getConfidence()
{
	
	return bestConfidence;
}

void HotSpots::generateBestMatch()
{
	float best1 = 0.0;
	int best1X = 0;
	int best1Y = 0;
	for(int y = 0; y < depth; y++)
		for(int x = 0; x < width; x++) {
			if(getScore(x,y) > best1) {
				best1 = getScore(x,y);
				best1X = x;
				best1Y = y;
			}
		}

	float best2 = 0.0;

	for(int y = 0; y < depth; y++)
		for(int x = 0; x < width; x++) {

			//std::cerr <<"dist: "<<getDistance(x,y,bestX,bestY)<<"  score: "<<getScore(x,y)<<std::endl;
			if(getScore(x,y) > best2 && getDistance(x,y,best1X,best1Y) >= confRadius) {
				best2 = getScore(x,y);
			}
		}

	float delta = best1 - best2;

	bestConfidence = std::min(delta/1.0,1.0);
	bestX = center_x -1.0* width*step/2.0 +step*best1X;
	bestY = center_y -1.0* depth*step/2.0 +step*best1Y;
	std::cerr<<"confidence: "<<bestConfidence<<"  best1: "<<best1<<"  best2: "<<best2<<std::endl;

}
float HotSpots::matrixToArrayPos(unsigned int a ,unsigned int b)
{
	return matrixToArrayPos(std::pair<unsigned int,unsigned int>(a,b));
}
float HotSpots::getScore(unsigned int a ,unsigned int b)
{
	unsigned int pos = matrixToArrayPos(a,b);
	return scores[pos];


}
bool HotSpots::recordPoint(float x, float y, float value)
{
	std::cerr<<"RP"<<std::endl;
	std::pair<int,int> coords;
	try {
		coords = worldToMatrixPos(x,y);
	} catch(...) {
		
		return false;
	}
ROS_INFO_STREAM("HotSpots: ??: "<<coords.first<<" "<<coords.second);
	unsigned int pos =  matrixToArrayPos(coords);
	std::cerr<<"Hotspots X: "<<coords.first<<"  Y:" <<coords.second<<std::endl;
	for(int i = 0; i<scores.size(); i++)
		scores[i] = scores[i] * decayRate;

	unsigned int radiusInCells = ceil(radius/step);

	unsigned int minX = coords.first > radiusInCells? (coords.first-radiusInCells): 0U;
	unsigned int maxX = std::min(coords.first+radiusInCells,width-1);

	unsigned int minY = coords.second > radiusInCells? (coords.second-radiusInCells): 0U;
	unsigned int maxY = std::min(coords.second+radiusInCells,depth-1);


	for(int y = minY; y <= maxY; y++)
		for(int x = minX; x <= maxX; x++) {
			std::pair<unsigned int,unsigned int> coords_bis(x,y);
			float distance = getDistance(coords.first,coords.second,coords_bis.first,coords_bis.second);
			
			if(distance<radius) {

				unsigned int ppp = matrixToArrayPos(coords_bis);
				float riduzione = cos(distance*3.14/2.0/radius);
				scores[ppp] += value*riduzione;

				//std::cerr << "X: "<<x<<"   Y: "<<y<<"  D: "<< distance<<std::endl;
			}
		}

	generateBestMatch();

	debugPrint();

	return true;
}
